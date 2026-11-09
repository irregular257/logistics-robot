#ifndef RTOS_REVIEW_FREERTOS_H
#define RTOS_REVIEW_FREERTOS_H

#include <stdint.h>
#include <stddef.h>

typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef struct HostTask *TaskHandle_t;
typedef struct HostQueue *QueueHandle_t;
typedef struct HostMutex *SemaphoreHandle_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(value) ((TickType_t)(value))
#define taskSCHEDULER_NOT_STARTED 0
#define taskSCHEDULER_RUNNING 2
#define configASSERT(condition) ((condition) ? (void)0 : Host_Assert(__FILE__, __LINE__))

#ifdef __cplusplus
extern "C" {
#endif
void Host_Assert(const char *file, int line);
BaseType_t xTaskGetSchedulerState(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks);
BaseType_t xTaskNotifyGive(TaskHandle_t task);
void vTaskDelay(TickType_t ticks);
QueueHandle_t xQueueCreate(UBaseType_t count, UBaseType_t item_size);
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t ticks);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks);
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void);
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t mutex, TickType_t ticks);
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t mutex);
#ifdef __cplusplus
}
#endif
#endif
