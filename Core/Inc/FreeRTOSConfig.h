#ifndef ROBOT_FREERTOS_CONFIG_H
#define ROBOT_FREERTOS_CONFIG_H

#include <stdint.h>

extern uint32_t SystemCoreClock;
void AppRtos_AssertFailed(const char *file, unsigned long line);

#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION   1
#define configCPU_CLOCK_HZ                       (SystemCoreClock)
/* HAL and the kernel share SysTick; both require an uninterrupted 1 ms tick. */
#define configTICK_RATE_HZ                       1000U
#define configUSE_TICKLESS_IDLE                  0
#define configMAX_PRIORITIES                     8U
#define configMINIMAL_STACK_SIZE                 256U
#define configMAX_TASK_NAME_LEN                  16U
#define configUSE_16_BIT_TICKS                    0
#define configIDLE_SHOULD_YIELD                   1

#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configSUPPORT_STATIC_ALLOCATION          0
#define configTOTAL_HEAP_SIZE                    (32U * 1024U)
#define configAPPLICATION_ALLOCATED_HEAP         0
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_TASK_NOTIFICATIONS             1
#define configQUEUE_REGISTRY_SIZE                8U
#define configUSE_QUEUE_SETS                     0
#define configUSE_CO_ROUTINES                    0
#define configUSE_TIMERS                         0

#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_MALLOC_FAILED_HOOK             1
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_TRACE_FACILITY                 0
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_STATS_FORMATTING_FUNCTIONS     0
#define configUSE_NEWLIB_REENTRANT               0

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark      1

/* STM32F407 implements four priority bits. IRQs using FromISR APIs must have
 * a numerical priority of 5 or greater. USART IRQs use priority 6. */
#define configPRIO_BITS                              4U
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15U
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5U
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8U - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8U - configPRIO_BITS))

#define configASSERT(condition) \
    do { if (!(condition)) AppRtos_AssertFailed(__FILE__, __LINE__); } while (0)

/* SysTick keeps a small HAL/kernel wrapper in stm32f4xx_it.c. The port owns
 * the context-switch exception handlers directly. */
#define vPortSVCHandler      SVC_Handler
#define xPortPendSVHandler   PendSV_Handler

#endif
