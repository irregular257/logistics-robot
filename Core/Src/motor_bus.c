#include "motor_bus.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>

#define MOTOR_FRAME_CAPACITY 13U

typedef struct {
    uint8_t data[MOTOR_FRAME_CAPACITY];
    uint16_t size;
    uint32_t gap_ms;
    TaskHandle_t caller;
    volatile uint8_t *result;
} MotorRequest;

static QueueHandle_t motor_queue;
static SemaphoreHandle_t transaction_lock;

void MotorBus_Init(void)
{
    motor_queue = xQueueCreate(4U, sizeof(MotorRequest));
    transaction_lock = xSemaphoreCreateRecursiveMutex();
    configASSERT(motor_queue != NULL);
    configASSERT(transaction_lock != NULL);
}

void MotorBus_Begin(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;
    configASSERT(xSemaphoreTakeRecursive(transaction_lock, portMAX_DELAY) == pdTRUE);
}

void MotorBus_End(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;
    configASSERT(xSemaphoreGiveRecursive(transaction_lock) == pdTRUE);
}

uint8_t MotorBus_Transmit(const uint8_t *data, uint16_t size, uint32_t gap_ms)
{
    MotorRequest request;
    volatile uint8_t result = 0U;

    if (!data || size == 0U || size > MOTOR_FRAME_CAPACITY) return 0U;
    /* Boot initialization runs before tasks exist. At runtime only MotorBus_Run
     * writes UART1. Every caller waits for completion, so its reply storage and
     * command remain valid and no motion backlog survives an acknowledged stop. */
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        result = HAL_UART_Transmit(&huart1, (uint8_t *)data, size, 10U) == HAL_OK;
        HAL_Delay(gap_ms);
        return result;
    }

    MotorBus_Begin();
    memcpy(request.data, data, size);
    request.size = size;
    request.gap_ms = gap_ms;
    request.caller = xTaskGetCurrentTaskHandle();
    request.result = &result;
    configASSERT(xQueueSend(motor_queue, &request, portMAX_DELAY) == pdPASS);
    /* Notification slot zero is reserved for synchronous bus/control replies.
     * The same task can have only one such request outstanding. */
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    MotorBus_End();
    return result;
}

void MotorBus_Run(void *argument)
{
    MotorRequest request;
    (void)argument;
    for (;;) {
        if (xQueueReceive(motor_queue, &request, portMAX_DELAY) == pdPASS) {
            uint8_t ok = HAL_UART_Transmit(&huart1, request.data, request.size, 10U) == HAL_OK;
            /* One extra tick preserves the minimum gap when this call begins
             * just before a tick boundary. */
            if (request.gap_ms != 0U) vTaskDelay(pdMS_TO_TICKS(request.gap_ms) + 1U);
            *request.result = ok;
            xTaskNotifyGive(request.caller);
        }
    }
}
