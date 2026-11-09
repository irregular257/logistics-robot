#ifndef RTOS_REVIEW_HAL_H
#define RTOS_REVIEW_HAL_H
#define __MAIN_H
#include <stdint.h>
#include <stddef.h>

typedef int HAL_StatusTypeDef;
typedef struct { int instance; } UART_HandleTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1

#ifdef __cplusplus
extern "C" {
#endif
extern UART_HandleTypeDef huart1;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t milliseconds);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *port, uint8_t *data,
                                  uint16_t size, uint32_t timeout);
uint32_t Host_GetPrimask(void);
void Host_DisableIrq(void);
void Host_SetPrimask(uint32_t saved);
#ifdef __cplusplus
}
#endif
#define __get_PRIMASK() Host_GetPrimask()
#define __disable_irq() Host_DisableIrq()
#define __set_PRIMASK(value) Host_SetPrimask(value)
#endif
