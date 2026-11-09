/* Runs the production coordinator and motor worker on host threads.
 * The HAL and FreeRTOS transport are simulated; STM32 timing is not measured. */
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "stubs/hal_stubs.h"
#include "stubs/FreeRTOS.h"
extern "C" {
#include "robot_control.h"
#include "motor_bus.h"
#include "logistics_task.h"
#include "web_protocol.h"
}

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

struct HostTask {
    std::mutex lock;
    std::condition_variable changed;
    uint32_t notifications = 0;
    std::string name;
};
struct HostQueue {
    std::mutex lock;
    std::condition_variable changed;
    std::deque<std::vector<uint8_t>> items;
    size_t capacity, item_size;
    HostQueue(size_t count, size_t size) : capacity(count), item_size(size) {}
};
struct HostMutex { std::recursive_timed_mutex lock; };
struct HostShutdown {};

static const auto started_at = Clock::now();
static std::atomic<int> scheduler_state{taskSCHEDULER_NOT_STARTED};
static std::atomic<bool> shutting_down{false};
static std::mutex registry_lock;
static std::vector<std::unique_ptr<HostTask>> tasks;
static std::vector<std::unique_ptr<HostQueue>> queues;
static std::vector<std::unique_ptr<HostMutex>> mutexes;
static thread_local HostTask *current_task;
static thread_local bool irq_masked;
static std::recursive_mutex irq_lock;
static std::atomic<unsigned> worker_errors{0}, failures{0};

#define CHECK(condition) do { if (!(condition)) { \
    std::printf("FAIL line %d: %s\n", __LINE__, #condition); ++failures; } } while (0)

static HostTask *bind_task(const char *name)
{
    auto task = std::make_unique<HostTask>();
    task->name = name;
    current_task = task.get();
    std::lock_guard<std::mutex> guard(registry_lock);
    tasks.push_back(std::move(task));
    return current_task;
}

template<class Predicate>
static bool wait_for(std::condition_variable &changed, std::unique_lock<std::mutex> &lock,
                     TickType_t ticks, Predicate predicate)
{
    if (shutting_down) throw HostShutdown{};
    if (predicate()) return true;
    if (!ticks) return false;
    const auto timeout = ticks == portMAX_DELAY ? 3000ms : std::chrono::milliseconds(ticks);
    bool ready = changed.wait_for(lock, timeout, [&] { return shutting_down || predicate(); });
    if (shutting_down) throw HostShutdown{};
    if (!ready && ticks == portMAX_DELAY)
        throw std::runtime_error("Production worker did not complete within the host timeout");
    return ready;
}

extern "C" void Host_Assert(const char *file, int line)
{
    throw std::runtime_error(std::string("configASSERT: ") + file + ':' + std::to_string(line));
}
extern "C" BaseType_t xTaskGetSchedulerState(void) { return scheduler_state; }
extern "C" TaskHandle_t xTaskGetCurrentTaskHandle(void)
{
    return current_task ? current_task : bind_task("Caller");
}
extern "C" uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks)
{
    auto *task = xTaskGetCurrentTaskHandle();
    std::unique_lock<std::mutex> lock(task->lock);
    if (!wait_for(task->changed, lock, ticks, [&] { return task->notifications != 0; })) return 0;
    uint32_t count = task->notifications;
    if (clear) task->notifications = 0;
    else --task->notifications;
    return count;
}
extern "C" BaseType_t xTaskNotifyGive(TaskHandle_t task)
{
    std::lock_guard<std::mutex> lock(task->lock);
    ++task->notifications;
    task->changed.notify_one();
    return pdPASS;
}
extern "C" void vTaskDelay(TickType_t ticks)
{
    if (shutting_down) throw HostShutdown{};
    std::this_thread::sleep_for(std::chrono::milliseconds(ticks));
    if (shutting_down) throw HostShutdown{};
}
extern "C" QueueHandle_t xQueueCreate(UBaseType_t count, UBaseType_t item_size)
{
    auto queue = std::make_unique<HostQueue>(count, item_size);
    auto result = queue.get();
    std::lock_guard<std::mutex> guard(registry_lock);
    queues.push_back(std::move(queue));
    return result;
}
extern "C" BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t ticks)
{
    std::unique_lock<std::mutex> lock(queue->lock);
    if (!wait_for(queue->changed, lock, ticks, [&] { return queue->items.size() < queue->capacity; })) return pdFALSE;
    const auto *bytes = static_cast<const uint8_t *>(item);
    queue->items.emplace_back(bytes, bytes + queue->item_size);
    queue->changed.notify_all();
    return pdPASS;
}
extern "C" BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks)
{
    std::unique_lock<std::mutex> lock(queue->lock);
    if (!wait_for(queue->changed, lock, ticks, [&] { return !queue->items.empty(); })) return pdFALSE;
    std::memcpy(item, queue->items.front().data(), queue->item_size);
    queue->items.pop_front();
    queue->changed.notify_all();
    return pdPASS;
}
extern "C" SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void)
{
    auto mutex = std::make_unique<HostMutex>();
    auto result = mutex.get();
    std::lock_guard<std::mutex> guard(registry_lock);
    mutexes.push_back(std::move(mutex));
    return result;
}
extern "C" BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t mutex, TickType_t ticks)
{
    auto timeout = ticks == portMAX_DELAY ? 3000ms : std::chrono::milliseconds(ticks);
    return mutex->lock.try_lock_for(timeout) ? pdTRUE : pdFALSE;
}
extern "C" BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t mutex)
{
    mutex->lock.unlock();
    return pdTRUE;
}
extern "C" uint32_t Host_GetPrimask(void) { return irq_masked ? 1U : 0U; }
extern "C" void Host_DisableIrq(void)
{
    if (!irq_masked) { irq_lock.lock(); irq_masked = true; }
}
extern "C" void Host_SetPrimask(uint32_t saved)
{
    if (!saved && irq_masked) { irq_masked = false; irq_lock.unlock(); }
}
extern "C" uint32_t HAL_GetTick(void)
{
    return 1U + static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - started_at).count());
}
extern "C" void HAL_Delay(uint32_t milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

UART_HandleTypeDef huart1 = {1};
static std::mutex wire_lock;
static std::vector<std::vector<uint8_t>> wire_log;
static std::atomic<unsigned> wrong_uart_owner{0};
extern "C" HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *port, uint8_t *data,
                                               uint16_t size, uint32_t timeout)
{
    CHECK(port == &huart1 && timeout == 10U && size > 0 && size <= 13U);
    if (scheduler_state == taskSCHEDULER_RUNNING && xTaskGetCurrentTaskHandle()->name != "MotorBus")
        ++wrong_uart_owner;
    std::lock_guard<std::mutex> guard(wire_lock);
    wire_log.emplace_back(data, data + size);
    return size > 1U && data[1] == 0xEEU ? HAL_ERROR : HAL_OK;
}

/* A small state-machine fixture records its thread owner and sends real bus
 * requests. The existing logistics_host.cpp tests the full movement logic. */
volatile uint8_t logistics_state, logistics_drop_index, logistics_error;
volatile float logistics_x_m, logistics_y_m;
static bool plan_ready;
static uint8_t accepted_plan[WEB_TASK_PLAN_SIZE];
static std::atomic<unsigned> automatic_steps{0}, automatic_starts{0}, plan_changes{0};
static std::atomic<unsigned> wrong_state_owner{0};
static std::atomic<bool> poll_feedback{true}, fail_start_uart{false};
static void require_automatic_owner()
{
    if (xTaskGetCurrentTaskHandle()->name != "Automatic") ++wrong_state_owner;
}
extern "C" void Logistics_Init(void)
{
    logistics_state = LOG_IDLE;
    logistics_drop_index = logistics_error = 0;
    logistics_x_m = logistics_y_m = 0;
    plan_ready = false;
    std::memset(accepted_plan, 0, sizeof(accepted_plan));
}
extern "C" uint8_t Logistics_IsRunning(void) { return logistics_state == LOG_PREPARE; }
extern "C" uint8_t Logistics_PlanReady(void) { return plan_ready; }
extern "C" uint8_t Logistics_SetPlan(const uint8_t *items, uint8_t length)
{
    require_automatic_owner();
    if (length != sizeof(accepted_plan) || items[0] == 0U) return 0U;
    std::memcpy(accepted_plan, items, length);
    plan_ready = true;
    ++plan_changes;
    return 1U;
}
extern "C" uint8_t Logistics_PlanMatches(const uint8_t *items, uint8_t length)
{
    return plan_ready && length == sizeof(accepted_plan) &&
           std::memcmp(items, accepted_plan, length) == 0;
}
extern "C" uint8_t Logistics_Start(void)
{
    require_automatic_owner();
    const uint8_t frame[] = {5U, static_cast<uint8_t>(fail_start_uart ? 0xEEU : 0xFDU), 0x6BU};
    if (!MotorBus_Transmit(frame, sizeof(frame), 1U)) return 0U;
    logistics_state = LOG_PREPARE;
    ++automatic_starts;
    return 1U;
}
extern "C" void Logistics_Cancel(void)
{
    require_automatic_owner();
    if (Logistics_IsRunning()) {
        const uint8_t frame[] = {1U, 0xFEU, 0x98U, 0U, 0x6BU};
        (void)MotorBus_Transmit(frame, sizeof(frame), 1U);
        logistics_state = LOG_ABORTED;
    }
}
extern "C" void Logistics_Task(void)
{
    require_automatic_owner();
    ++automatic_steps;
    if (poll_feedback) {
        const uint8_t poll[] = {1U, 0x36U, 0x6BU};
        (void)MotorBus_Transmit(poll, sizeof(poll), 1U);
    }
    if (Logistics_IsRunning()) logistics_x_m += 0.01f;
}
extern "C" uint8_t Logistics_GetLiftFeedback(int64_t *count, uint32_t *tick, uint32_t *serial)
{
    *count = 42; *tick = HAL_GetTick(); *serial = 7U;
    return 1U;
}

static std::thread run_worker(const char *name, void (*entry)(void *))
{
    return std::thread([=] {
        bind_task(name);
        try { entry(nullptr); }
        catch (const HostShutdown &) {}
        catch (const std::exception &error) { ++worker_errors; std::printf("Worker %s: %s\n", name, error.what()); }
    });
}

static void coordinator_tests()
{
    const uint8_t plan[] = {4U,1U,7U,4U,3U,3U};
    CHECK(RobotControl_SetPlan(nullptr, sizeof(plan)) == WEB_RESULT_BAD_ARGUMENT);
    CHECK(RobotControl_SetPlan(plan, 5U) == WEB_RESULT_BAD_ARGUMENT);
    CHECK(RobotControl_Start(plan, sizeof(plan)) == WEB_RESULT_INTERLOCK);
    CHECK(RobotControl_SetPlan(plan, sizeof(plan)) == WEB_RESULT_OK);
    CHECK(plan_changes == 1U);
    RobotStatus status;
    RobotControl_ReadStatus(&status);
    CHECK(status.plan_ready && !status.running);
    uint8_t changed_plan[sizeof(plan)];
    std::memcpy(changed_plan, plan, sizeof(plan)); changed_plan[0] = 1U;
    CHECK(RobotControl_Start(changed_plan, sizeof(changed_plan)) == WEB_RESULT_INTERLOCK);
    fail_start_uart = true;
    CHECK(RobotControl_Start(plan, sizeof(plan)) == WEB_RESULT_INTERLOCK);
    CHECK(automatic_starts == 0U);
    fail_start_uart = false;
    CHECK(RobotControl_Start(plan, sizeof(plan)) == WEB_RESULT_OK);
    CHECK(RobotControl_IsAutomatic());
    CHECK(RobotControl_Start(plan, sizeof(plan)) == WEB_RESULT_WRONG_MODE);
    CHECK(RobotControl_SetPlan(changed_plan, sizeof(changed_plan)) == WEB_RESULT_WRONG_MODE);
    CHECK(automatic_starts == 1U && plan_changes == 1U);

    unsigned before_steps = automatic_steps;
    RobotControl_ReadStatus(&status);
    float before_position = status.x_m;
    std::this_thread::sleep_for(45ms); // No webpage task or caller drives the automatic worker.
    RobotControl_ReadStatus(&status);
    CHECK(automatic_steps > before_steps && status.x_m > before_position && status.running);
    CHECK(RobotControl_Stop() == WEB_RESULT_OK);
    RobotControl_ReadStatus(&status);
    CHECK(!status.running && status.state == LOG_ABORTED && status.plan_ready);
    CHECK(RobotControl_Start(plan, sizeof(plan)) == WEB_RESULT_OK);
    CHECK(automatic_starts == 2U);
    CHECK(RobotControl_Stop() == WEB_RESULT_OK);
    int64_t count; uint32_t tick, serial;
    CHECK(RobotControl_GetLiftFeedback(&count, &tick, &serial) && count == 42 && serial == 7U);
    CHECK(wrong_state_owner == 0U);
    std::puts("PASS coordinator: requests execute on the automatic task, independent progress, plan retention, start deduplication and UART failure reply");
}

static void bus_tests()
{
    CHECK(!MotorBus_Transmit(nullptr, 3U, 0U));
    const uint8_t invalid[14] = {};
    CHECK(!MotorBus_Transmit(invalid, sizeof(invalid), 0U));
    const uint8_t failure[] = {1U,0xEEU,0x6BU};
    CHECK(!MotorBus_Transmit(failure, sizeof(failure), 0U));
    const uint8_t valid[] = {1U,0x36U,0x6BU};
    CHECK(MotorBus_Transmit(valid, sizeof(valid), 0U));

    std::atomic<bool> first_wheel_sent{false}, poll_attempted{false};
    std::thread poller([&] {
        bind_task("FeedbackCaller");
        while (!first_wheel_sent) std::this_thread::sleep_for(1ms);
        poll_attempted = true;
        CHECK(MotorBus_Transmit(valid, sizeof(valid), 0U));
    });
    size_t beginning;
    { std::lock_guard<std::mutex> guard(wire_lock); beginning = wire_log.size(); }
    MotorBus_Begin();
    for (uint8_t id = 1U; id <= 4U; ++id) {
        const uint8_t wheel[] = {id,0xF6U,0U,0U,20U,180U,1U,0x6BU};
        CHECK(MotorBus_Transmit(wheel, sizeof(wheel), 2U));
        if (id == 1U) {
            first_wheel_sent = true;
            while (!poll_attempted) std::this_thread::sleep_for(1ms);
        }
    }
    const uint8_t sync[] = {0U,0xFFU,0x66U,0x6BU};
    CHECK(MotorBus_Transmit(sync, sizeof(sync), 1U));
    MotorBus_End();
    poller.join();
    {
        std::lock_guard<std::mutex> guard(wire_lock);
        size_t first = beginning;
        while (first < wire_log.size() && wire_log[first][1] != 0xF6U) ++first;
        CHECK(first + 5U < wire_log.size());
        if (first + 5U < wire_log.size()) {
            for (uint8_t id = 1U; id <= 4U; ++id)
                CHECK(wire_log[first + id - 1U][0] == id && wire_log[first + id - 1U][1] == 0xF6U);
            CHECK(wire_log[first + 4U][1] == 0xFFU);
            CHECK(wire_log[first + 5U][1] == 0x36U);
        }
    }
    CHECK(wrong_uart_owner == 0U);
    std::puts("PASS motor bus: worker-only UART, recursive batch transaction, concurrent feedback serialization and error propagation");
}

static void shutdown_workers()
{
    poll_feedback = false;
    std::this_thread::sleep_for(20ms); // Let any in-flight synchronous bus reply complete.
    shutting_down = true;
    std::lock_guard<std::mutex> registry_guard(registry_lock);
    for (auto &queue : queues) queue->changed.notify_all();
    for (auto &task : tasks) task->changed.notify_all();
}

int main()
{
    bind_task("WebCaller");
    MotorBus_Init(); RobotControl_Init();
    const uint8_t plan[] = {4U,1U,7U,4U,3U,3U};
    CHECK(RobotControl_SetPlan(plan, sizeof(plan)) == WEB_RESULT_INTERLOCK);
    const uint8_t boot[] = {1U,0x36U,0x6BU};
    CHECK(MotorBus_Transmit(boot, sizeof(boot), 0U));
    scheduler_state = taskSCHEDULER_RUNNING;
    auto motor = run_worker("MotorBus", MotorBus_Run);
    auto automatic = run_worker("Automatic", RobotControl_Run);
    try { coordinator_tests(); bus_tests(); }
    catch (const std::exception &error) { ++failures; std::printf("FAIL exception: %s\n", error.what()); }
    shutdown_workers();
    automatic.join(); motor.join();
    CHECK(worker_errors == 0U);
    CHECK(wrong_uart_owner == 0U && wrong_state_owner == 0U);
    std::printf("RTOS runtime host regression: %u failure(s)\n", failures.load());
    return failures ? 1 : 0;
}
