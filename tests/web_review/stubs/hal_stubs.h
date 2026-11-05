#ifndef WEB_REVIEW_HAL_STUBS_H
#define WEB_REVIEW_HAL_STUBS_H
#include <stdint.h>
#include <stddef.h>
#define __MAIN_H
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
#define HAL_ERROR 1
typedef struct {int instance;} UART_HandleTypeDef;
typedef struct {int instance;} TIM_HandleTypeDef;
extern UART_HandleTypeDef huart1, huart4;
extern TIM_HandleTypeDef htim1;
extern uint8_t vision_follow_enable;
#define TIM_CHANNEL_1 1
#define TIM_CHANNEL_2 2
#define TIM_CHANNEL_3 3
#define GRAB 10
#define OPEN 70
#define IN 54
#define OUT 170
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define __DMB() ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(x) ((void)0)
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef Duoji_StartPWM(uint32_t);
HAL_StatusTypeDef JiaZhua_SetAngle(uint16_t);
HAL_StatusTypeDef Tray_SetAngle(uint16_t);
HAL_StatusTypeDef Duoji_SetPresets(uint16_t,uint16_t,uint16_t,uint16_t);
uint16_t Duoji_GetClawPreset(uint8_t);
uint16_t Duoji_GetGimbalPreset(uint8_t);
void motor_speed(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t);
void motor_sync(void);
void ZhuaZi(uint16_t);
void YunTai(uint16_t);
void go(float,float,float);
#endif
