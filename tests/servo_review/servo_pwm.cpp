/* Run the production servo driver against HAL's per-channel start semantics. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
extern "C" {
#include "../../Hardware/duoji/duiji.h"
}
static TIM_TypeDef timer_registers;
TIM_HandleTypeDef htim1;
static unsigned starts;
static bool start_fails;
HAL_TIM_ChannelStateTypeDef HAL_TIM_GetChannelState(const TIM_HandleTypeDef *timer, uint32_t channel)
{
    return timer->ChannelState[channel / 4U];
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    ++starts;
    if (start_fails || timer->ChannelState[channel / 4U] != HAL_TIM_CHANNEL_STATE_READY)
        return HAL_ERROR;
    timer->ChannelState[channel / 4U] = HAL_TIM_CHANNEL_STATE_BUSY;
    timer->Instance->CCER |= 1U << channel;
    timer->Instance->BDTR |= TIM_BDTR_MOE;
    timer->Instance->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}
static void reset(void)
{
    memset(&timer_registers, 0, sizeof(timer_registers));
    memset(&htim1, 0, sizeof(htim1));
    htim1.Instance = &timer_registers;
    htim1.State = 2U; /* Base_Start_IT has already made the base timer BUSY. */
    for (unsigned i = 0; i < 3; ++i) htim1.ChannelState[i] = HAL_TIM_CHANNEL_STATE_READY;
    starts = 0;
    start_fails = false;
}
int main(void)
{
    reset();
    const uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
    for (unsigned i = 0; i < 3; ++i) {
        assert(Duoji_StartPWM(channels[i]) == HAL_OK);
        assert(starts == i + 1U);
        assert(Duoji_StartPWM(channels[i]) == HAL_OK);
        assert(starts == i + 1U);
    }
    /* Strict HAL rejects the active channel; the helper accepts active PWM. */
    assert(HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) == HAL_ERROR);
    const unsigned active_starts = starts;
    assert(Duoji_StartPWM(TIM_CHANNEL_1) == HAL_OK);
    assert(starts == active_starts);
    /* A BUSY software state alone must not claim there is an output signal. */
    timer_registers.CCER &= ~TIM_CCER_CC1E;
    assert(Duoji_StartPWM(TIM_CHANNEL_1) == HAL_ERROR);
    timer_registers.CCER |= TIM_CCER_CC1E;
    timer_registers.BDTR &= ~TIM_BDTR_MOE;
    assert(Duoji_StartPWM(TIM_CHANNEL_1) == HAL_ERROR);
    timer_registers.BDTR |= TIM_BDTR_MOE;
    timer_registers.CR1 &= ~TIM_CR1_CEN;
    assert(Duoji_StartPWM(TIM_CHANNEL_1) == HAL_ERROR);
    reset();
    start_fails = true;
    assert(Duoji_StartPWM(TIM_CHANNEL_3) == HAL_ERROR);
    assert(htim1.ChannelState[2] == HAL_TIM_CHANNEL_STATE_READY);
    const unsigned failed_starts = starts;
    assert(Duoji_StartPWM(TIM_CHANNEL_4) == HAL_ERROR);
    assert(starts == failed_starts);
    htim1.ChannelState[0] = HAL_TIM_CHANNEL_STATE_RESET;
    assert(Duoji_StartPWM(TIM_CHANNEL_1) == HAL_ERROR);
    /* The 270-degree commands now use one-microsecond timer ticks. */
    assert(OUT == 170U && IN == 54U && OPEN == 70U && GRAB == 10U);
    ZhuaZi(GRAB); assert(timer_registers.CCR1 == 574U);
    ZhuaZi(OPEN); assert(timer_registers.CCR1 == 1019U);
    YunTai(IN); assert(timer_registers.CCR3 == 900U);
    YunTai(OUT); assert(timer_registers.CCR3 == 1759U);
    ZhuanPan(HONG); assert(timer_registers.CCR2 == 648U);
    ZhuanPan(LAN); assert(timer_registers.CCR2 == 1574U);
    ZhuanPan(LV); assert(timer_registers.CCR2 == 2396U);
    ZhuanPan(OUT1); assert(timer_registers.CCR2 == 1389U);
    /* One-degree adjustment used to disappear in the 100-us quantization. */
    for (uint16_t angle = 0U; angle <= DUOJI_MAX_ANGLE; ++angle) {
        const uint32_t old_claw = timer_registers.CCR1;
        const uint32_t old_tray = timer_registers.CCR2;
        const uint32_t old_gimbal = timer_registers.CCR3;
        YunTai(angle);
        assert(timer_registers.CCR1 == old_claw);
        assert(timer_registers.CCR2 == old_tray);
        assert(timer_registers.CCR3 >= 500U && timer_registers.CCR3 <= 2500U);
        if (angle > 0U) assert(timer_registers.CCR3 > old_gimbal);
    }
    YunTai(0U); assert(timer_registers.CCR3 == 500U);
    YunTai(135U); assert(timer_registers.CCR3 == 1500U);
    YunTai(270U); assert(timer_registers.CCR3 == 2500U);
    YunTai(65535U); assert(timer_registers.CCR3 == 2500U);
    ZhuaZi(65535U); assert(timer_registers.CCR1 == 2500U);
    ZhuanPan(-1.0f); assert(timer_registers.CCR2 == 500U);
    ZhuanPan(271.0f); assert(timer_registers.CCR2 == 2500U);
    reset();
    assert(JiaZhua_SetAngle(70U) == HAL_OK);
    assert(timer_registers.CCR1 == 1019U && starts == 1U);
    assert(JiaZhua_SetAngle(71U) == HAL_OK);
    assert(timer_registers.CCR1 == 1026U && starts == 1U);
    const uint32_t accepted_claw = timer_registers.CCR1;
    assert(JiaZhua_SetAngle(271U) == HAL_ERROR);
    assert(timer_registers.CCR1 == accepted_claw && starts == 1U);
    timer_registers.CCER &= ~TIM_CCER_CC1E;
    assert(JiaZhua_SetAngle(100U) == HAL_ERROR);
    reset(); start_fails = true;
    assert(JiaZhua_SetAngle(100U) == HAL_ERROR);
    assert((timer_registers.CCER & TIM_CCER_CC1E) == 0U);
    /* The tray API starts CH2 once, validates its limits, and leaves CH1/3. */
    reset();
    timer_registers.CCR1 = 777U; timer_registers.CCR3 = 1234U;
    for (uint16_t angle = 0U; angle <= DUOJI_MAX_ANGLE; ++angle) {
        const uint32_t previous = timer_registers.CCR2;
        assert(Tray_SetAngle(angle) == HAL_OK);
        assert(starts == 1U);
        assert(timer_registers.CCR1 == 777U && timer_registers.CCR3 == 1234U);
        assert(timer_registers.CCR2 >= 500U && timer_registers.CCR2 <= 2500U);
        if (angle > 0U) assert(timer_registers.CCR2 > previous);
    }
    assert(timer_registers.CCR2 == 2500U);
    assert(Tray_SetAngle(271U) == HAL_ERROR);
    assert(Tray_SetAngle(65535U) == HAL_ERROR);
    assert(timer_registers.CCR2 == 2500U && starts == 1U);
    timer_registers.CCER &= ~TIM_CCER_CC2E;
    assert(Tray_SetAngle(135U) == HAL_ERROR);
    reset(); start_fails = true;
    assert(Tray_SetAngle(135U) == HAL_ERROR);
    assert((timer_registers.CCER & TIM_CCER_CC2E) == 0U);
    /* Saving calibration alone must not move any servo or enable its PWM. */
    reset();
    assert(Duoji_GetClawPreset(0U) == 10U);
    assert(Duoji_GetClawPreset(1U) == 70U);
    assert(Duoji_GetGimbalPreset(0U) == 54U);
    assert(Duoji_GetGimbalPreset(1U) == 170U);
    const TIM_TypeDef before_presets = timer_registers;
    assert(Duoji_SetPresets(23U, 111U, 49U, 170U) == HAL_OK);
    assert(Duoji_GetClawPreset(0U) == 23U);
    assert(Duoji_GetClawPreset(1U) == 111U);
    assert(Duoji_GetGimbalPreset(0U) == 49U);
    assert(Duoji_GetGimbalPreset(1U) == 170U);
    assert(memcmp(&timer_registers, &before_presets, sizeof(before_presets)) == 0);
    assert(starts == 0U);
    for (unsigned invalid = 0U; invalid < 4U; ++invalid) {
        uint16_t args[] = {30U, 120U, 40U, 160U};
        args[invalid] = 271U;
        assert(Duoji_SetPresets(args[0], args[1], args[2], args[3]) == HAL_ERROR);
        assert(Duoji_GetClawPreset(0U) == 23U);
        assert(Duoji_GetClawPreset(1U) == 111U);
        assert(Duoji_GetGimbalPreset(0U) == 49U);
        assert(Duoji_GetGimbalPreset(1U) == 170U);
        assert(memcmp(&timer_registers, &before_presets, sizeof(before_presets)) == 0);
        assert(starts == 0U);
    }
    assert(Duoji_SetPresets(0U, 270U, 0U, 270U) == HAL_OK);
    assert(Duoji_GetClawPreset(0U) == 0U && Duoji_GetClawPreset(1U) == 270U);
    assert(Duoji_GetGimbalPreset(0U) == 0U && Duoji_GetGimbalPreset(1U) == 270U);
    puts("Servo PWM lifecycle, HAL failures, 270-degree mapping, claw/tray APIs and atomic preset validation passed.");
    return 0;
}
