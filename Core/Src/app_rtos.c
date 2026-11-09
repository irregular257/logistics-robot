#include "app_rtos.h"
#include "main.h"
#include "usart.h"
#include "web_control.h"
#include "motor_bus.h"
#include "robot_control.h"
#include "FreeRTOS.h"
#include "task.h"

/* Inspect these variables in Keil after a scheduler assertion or fault. */
volatile const char *app_rtos_fault_file;
volatile unsigned long app_rtos_fault_line;

static void web_task(void *argument)
{
    (void)argument;
    for (;;) {
        (void)WebControl_Task();
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
}

void AppRtos_Start(void)
{
    configASSERT(xTaskCreate(MotorBus_Run, "MotorBus", 512U, NULL, 4U, NULL) == pdPASS);
    configASSERT(xTaskCreate(RobotControl_Run, "Automatic", 1024U, NULL, 3U, NULL) == pdPASS);
    configASSERT(xTaskCreate(web_task, "WebControl", 1024U, NULL, 2U, NULL) == pdPASS);
    vTaskStartScheduler();
    AppRtos_AssertFailed(__FILE__, __LINE__);
}

static void fault_stop(void)
{
    uint8_t address;
    __disable_irq();
    /* Best-effort stop even if the scheduler or HAL lock has failed. Bounded
     * polling works with interrupts disabled and does not depend on SysTick. */
    if ((USART1->CR1 & USART_CR1_UE) != 0U) {
        for (address = 1U; address <= 5U; ++address) {
            uint8_t byte;
            uint8_t frame[5] = {address, 0xFEU, 0x98U, 0U, 0x6BU};
            for (byte = 0U; byte < sizeof(frame); ++byte) {
                uint32_t remaining = 100000U;
                while ((USART1->SR & USART_SR_TXE) == 0U && --remaining != 0U) {}
                if (remaining == 0U) break;
                USART1->DR = frame[byte];
            }
        }
    }
    for (;;) {}
}

void AppRtos_AssertFailed(const char *file, unsigned long line)
{
    app_rtos_fault_file = file;
    app_rtos_fault_line = line;
    fault_stop();
}

void vApplicationMallocFailedHook(void)
{
    AppRtos_AssertFailed("FreeRTOS allocation", 0U);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    AppRtos_AssertFailed(task_name, 0U);
}
