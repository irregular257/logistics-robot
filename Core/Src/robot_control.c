#include "robot_control.h"
#include "logistics_task.h"
#include "web_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

typedef enum {
    ROBOT_SET_PLAN,
    ROBOT_START,
    ROBOT_STOP
} RobotCommand;

typedef struct {
    RobotCommand command;
    uint8_t plan[WEB_TASK_PLAN_SIZE];
    uint32_t issued_at;
    TaskHandle_t caller;
    volatile uint8_t *result;
} RobotRequest;

static QueueHandle_t command_queue;
static RobotStatus published_status;

static void publish_status(void)
{
    RobotStatus next;
    uint32_t saved;
    next.state = logistics_state;
    next.drop_index = logistics_drop_index;
    next.error = logistics_error;
    next.running = Logistics_IsRunning();
    next.plan_ready = Logistics_PlanReady();
    next.x_m = logistics_x_m;
    next.y_m = logistics_y_m;
    saved = __get_PRIMASK();
    __disable_irq();
    published_status = next;
    __set_PRIMASK(saved);
}

void RobotControl_Init(void)
{
    command_queue = xQueueCreate(2U, sizeof(RobotRequest));
    configASSERT(command_queue != NULL);
    Logistics_Init();
    publish_status();
}

void RobotControl_ReadStatus(RobotStatus *status)
{
    uint32_t saved;
    if (!status) return;
    saved = __get_PRIMASK();
    __disable_irq();
    *status = published_status;
    __set_PRIMASK(saved);
}

uint8_t RobotControl_IsAutomatic(void)
{
    RobotStatus status;
    RobotControl_ReadStatus(&status);
    return status.running;
}

static uint8_t submit_request(RobotCommand command, const uint8_t *plan, uint8_t length)
{
    RobotRequest request = {0};
    volatile uint8_t result = WEB_RESULT_INTERLOCK;
    if (command != ROBOT_STOP && (!plan || length != WEB_TASK_PLAN_SIZE))
        return WEB_RESULT_BAD_ARGUMENT;
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
        return WEB_RESULT_INTERLOCK;

    request.command = command;
    if (plan) memcpy(request.plan, plan, WEB_TASK_PLAN_SIZE);
    request.issued_at = HAL_GetTick();
    request.caller = xTaskGetCurrentTaskHandle();
    request.result = &result;
    if (xQueueSend(command_queue, &request, pdMS_TO_TICKS(50U)) != pdPASS)
        return WEB_RESULT_INTERLOCK;
    /* Await execution, rather than ACKing a queued command that might later
     * fail. The task and result stay alive until the reply arrives. */
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return result;
}

uint8_t RobotControl_SetPlan(const uint8_t *items, uint8_t length)
{
    return submit_request(ROBOT_SET_PLAN, items, length);
}

uint8_t RobotControl_Start(const uint8_t *items, uint8_t length)
{
    return submit_request(ROBOT_START, items, length);
}

uint8_t RobotControl_Stop(void)
{
    return submit_request(ROBOT_STOP, NULL, 0U);
}

uint8_t RobotControl_GetLiftFeedback(int64_t *count, uint32_t *tick, uint32_t *serial)
{
    return Logistics_GetLiftFeedback(count, tick, serial);
}

static uint8_t execute_request(const RobotRequest *request)
{
    if (request->command == ROBOT_STOP) {
        Logistics_Cancel();
        return WEB_RESULT_OK;
    }
    if ((uint32_t)(HAL_GetTick() - request->issued_at) >= 500U)
        return WEB_RESULT_STALE;
    if (Logistics_IsRunning()) return WEB_RESULT_WRONG_MODE;

    if (request->command == ROBOT_SET_PLAN)
        return Logistics_SetPlan(request->plan, sizeof(request->plan)) ?
               WEB_RESULT_OK : WEB_RESULT_BAD_ARGUMENT;
    if (request->command == ROBOT_START) {
        if (!Logistics_PlanMatches(request->plan, sizeof(request->plan)))
            return WEB_RESULT_INTERLOCK;
        return Logistics_Start() ? WEB_RESULT_OK : WEB_RESULT_INTERLOCK;
    }
    return WEB_RESULT_UNSUPPORTED;
}

void RobotControl_Run(void *argument)
{
    RobotRequest request;
    (void)argument;
    for (;;) {
        if (xQueueReceive(command_queue, &request, 0U) == pdPASS) {
            uint8_t result = execute_request(&request);
            publish_status();
            *request.result = result;
            xTaskNotifyGive(request.caller);
        }
        Logistics_Task();
        publish_status();
        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}
