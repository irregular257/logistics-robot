#ifndef SERVO_REVIEW_MAIN_H
#define SERVO_REVIEW_MAIN_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef enum {
    HAL_TIM_CHANNEL_STATE_RESET = 0,
    HAL_TIM_CHANNEL_STATE_READY = 1,
    HAL_TIM_CHANNEL_STATE_BUSY = 2
} HAL_TIM_ChannelStateTypeDef;
typedef struct {
    uint32_t CR1, CCER, BDTR, CCR1, CCR2, CCR3;
} TIM_TypeDef;
typedef struct {
    TIM_TypeDef *Instance;
    uint32_t State;
    HAL_TIM_ChannelStateTypeDef ChannelState[3];
} TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define TIM_CHANNEL_4 12U
#define TIM_CCER_CC1E (1U << 0)
#define TIM_CCER_CC2E (1U << 4)
#define TIM_CCER_CC3E (1U << 8)
#define TIM_BDTR_MOE (1U << 15)
#define TIM_CR1_CEN 1U
#define __HAL_TIM_SET_COMPARE(h,c,v) \
    ((c) == TIM_CHANNEL_1 ? ((h)->Instance->CCR1 = (v)) : \
     (c) == TIM_CHANNEL_2 ? ((h)->Instance->CCR2 = (v)) : \
     ((h)->Instance->CCR3 = (v)))
extern TIM_HandleTypeDef htim1;
HAL_TIM_ChannelStateTypeDef HAL_TIM_GetChannelState(const TIM_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
#ifdef __cplusplus
}
#endif
#endif
