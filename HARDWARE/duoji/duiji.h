#ifndef __DUIJI_H
#define __DUIJI_H
#include "main.h"



#define OPEN 70 //?????? ????????????
#define GRAB 10  //???? ????????????
#define HONG 20 
#define LAN 145 
#define LV  256 
#define OUT1 120//??????????
#define OUT 170
#define IN  54 

/* TIM1 runs at 1 MHz: 500..2500 ticks are 0.5..2.5 ms, at 50 Hz.
 * The limits describe the servo command range, not mechanical clearances. */
#define DUOJI_MAX_ANGLE 270U
#define DUOJI_PULSE_MIN_US 500U
#define DUOJI_PULSE_SPAN_US 2000U





void ZhuaZi(uint16_t xian);
/* Idempotent start for the existing TIM1 servo PWM channels. */
HAL_StatusTypeDef Duoji_StartPWM(uint32_t channel);
/* Validate a diagnostic claw angle, then enable PE9/TIM1_CH1 PWM. */
HAL_StatusTypeDef JiaZhua_SetAngle(uint16_t angle);
/* Validate a tray angle, then enable PE11/TIM1_CH2 PWM. */
HAL_StatusTypeDef Tray_SetAngle(uint16_t angle);
/* Runtime calibration is RAM-only and does not move any servo by itself. */
HAL_StatusTypeDef Duoji_SetPresets(uint16_t claw_close, uint16_t claw_open,
                                  uint16_t gimbal_in, uint16_t gimbal_out);
uint16_t Duoji_GetClawPreset(uint8_t open);
uint16_t Duoji_GetGimbalPreset(uint8_t extended);
void YunTai(uint16_t target);  // ????????? target
void ZhuanPan(float angle);
void ZhuaZi1(uint16_t angle);
void YunTai1(uint16_t angle);





#endif


