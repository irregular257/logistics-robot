#include "duiji.h"
#include "main.h"
#include "tim.h"

typedef struct {
    uint16_t claw_close, claw_open, gimbal_in, gimbal_out;
} Duoji_Presets;

static Duoji_Presets presets = {GRAB, OPEN, IN, OUT};

HAL_StatusTypeDef Duoji_SetPresets(uint16_t claw_close, uint16_t claw_open,
                                  uint16_t gimbal_in, uint16_t gimbal_out)
{
    Duoji_Presets next;
    if (claw_close > DUOJI_MAX_ANGLE || claw_open > DUOJI_MAX_ANGLE ||
        gimbal_in > DUOJI_MAX_ANGLE || gimbal_out > DUOJI_MAX_ANGLE)
        return HAL_ERROR;
    next.claw_close = claw_close;
    next.claw_open = claw_open;
    next.gimbal_in = gimbal_in;
    next.gimbal_out = gimbal_out;
    /* Web calibration is accepted only while automatic control is idle.
     * Validate all four values before publishing the new presets. */
    presets = next;
    return HAL_OK;
}

uint16_t Duoji_GetClawPreset(uint8_t open)
{
    return open ? presets.claw_open : presets.claw_close;
}

uint16_t Duoji_GetGimbalPreset(uint8_t extended)
{
    return extended ? presets.gimbal_out : presets.gimbal_in;
}

static uint32_t Duoji_AngleToPulse(uint16_t angle)
{
    if (angle > DUOJI_MAX_ANGLE) angle = DUOJI_MAX_ANGLE;
    return DUOJI_PULSE_MIN_US +
           (DUOJI_PULSE_SPAN_US * angle + DUOJI_MAX_ANGLE / 2U) /
           DUOJI_MAX_ANGLE;
}

/* The base interrupt and the three PWM channels share TIM1. HAL's PWM
 * channel state remains BUSY while output is active, so starting a channel
 * again after switching between manual and automatic control is not an error.
 * Only accept BUSY when the channel and timer are actually outputting PWM. */
HAL_StatusTypeDef Duoji_StartPWM(uint32_t channel)
{
    uint32_t enable_mask;
    HAL_TIM_ChannelStateTypeDef state;

    switch (channel) {
    case TIM_CHANNEL_1: enable_mask = TIM_CCER_CC1E; break;
    case TIM_CHANNEL_2: enable_mask = TIM_CCER_CC2E; break;
    case TIM_CHANNEL_3: enable_mask = TIM_CCER_CC3E; break;
    default: return HAL_ERROR;
    }

    state = HAL_TIM_GetChannelState(&htim1, channel);
    if (state == HAL_TIM_CHANNEL_STATE_READY) {
        return HAL_TIM_PWM_Start(&htim1, channel);
    }
    if (state == HAL_TIM_CHANNEL_STATE_BUSY &&
        (htim1.Instance->CCER & enable_mask) != 0U &&
        (htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U &&
        (htim1.Instance->CR1 & TIM_CR1_CEN) != 0U) {
        return HAL_OK;
    }
    return HAL_ERROR;
}



void ZhuaZi(uint16_t angle)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Duoji_AngleToPulse(angle));
}

HAL_StatusTypeDef JiaZhua_SetAngle(uint16_t angle)
{
    if (angle > DUOJI_MAX_ANGLE) return HAL_ERROR;
    ZhuaZi(angle);
    return Duoji_StartPWM(TIM_CHANNEL_1);
}


void YunTai(uint16_t angle)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Duoji_AngleToPulse(angle));
}


void ZhuanPan(float angle)
{
	/* Preserve the float API used by legacy tasks, and clamp before conversion. */
	if (!(angle >= 0.0f)) angle = 0.0f;
	if (angle > (float)DUOJI_MAX_ANGLE) angle = (float)DUOJI_MAX_ANGLE;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                         DUOJI_PULSE_MIN_US +
                         (uint32_t)(DUOJI_PULSE_SPAN_US * angle /
                                    (float)DUOJI_MAX_ANGLE + 0.5f));
}

HAL_StatusTypeDef Tray_SetAngle(uint16_t angle)
{
    if (angle > DUOJI_MAX_ANGLE) return HAL_ERROR;
    ZhuanPan((float)angle);
    return Duoji_StartPWM(TIM_CHANNEL_2);
}

