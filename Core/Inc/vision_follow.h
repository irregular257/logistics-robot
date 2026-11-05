#ifndef VISION_FOLLOW_H
#define VISION_FOLLOW_H

#include "main.h"

/* Keil Watch controls: enable=1, color=1(red)/2(green)/3(blue)/4(any). */
extern volatile uint8_t vision_follow_enable;
extern volatile uint8_t vision_follow_color;
/* 0=off, 1=waiting/start input released, 2=following, 3=target lost, 4=busy. */
extern volatile uint8_t vision_follow_state;
extern volatile uint16_t vision_follow_x;
extern volatile uint16_t vision_follow_y;
/* Optical-axis depth; use distance_mm only while distance_valid == 1.
 * 0=OK, 1=no target, 2=uncalibrated, 3=clipped/small blob,
 * 4=outside calibrated limits, 5=wrong resolution, 6=waiting/off, 7=stale.
 */
extern volatile uint16_t vision_distance_mm;
extern volatile uint8_t vision_distance_valid;
extern volatile uint8_t vision_distance_reason;

#define VISION_FOLLOW_TARGET_X         160.0f
#define VISION_FOLLOW_TARGET_Y         120.0f
#define VISION_FOLLOW_DEADBAND_PX      5.0f
#define VISION_FOLLOW_GAIN_RPS_PER_PX  0.002f
#define VISION_FOLLOW_MAX_RPS          0.12f
#define VISION_FOLLOW_MAX_OMEGA        0.12f
#define VISION_FOLLOW_PERIOD_MS        60U
#define VISION_FOLLOW_TIMEOUT_MS       250U

/* Original downward/grasping camera mapping from jiaozhun5().
 * Calibrate these four coefficients after changing the camera mounting pose.
 * ex = target_x - observed_x; ey = target_y - observed_y.
 */
#define VISION_FOLLOW_EX_TO_VX         1.0f
#define VISION_FOLLOW_EY_TO_VX         0.0f
#define VISION_FOLLOW_EX_TO_VY         0.0f
#define VISION_FOLLOW_EY_TO_VY        -1.0f

/* Called from the USART6 receive callback; never drives motors in interrupt. */
void Openmv_ReceiveByte(uint8_t byte);
/* Returns 1 while owning the chassis, including its stop/exit cycle. */
uint8_t VisionFollow_Task(void);

/* Observation-only mode for the logistics state machine. Never drives motors. */
typedef struct { uint16_t x,y; uint8_t color,shape; uint32_t tick; } VisionObservation;
uint8_t VisionObserve_Start(uint8_t color);
/* Color+shape request F7 token color shape CRC8 FE. Reply F6 includes the
 * token and both attributes; old F9 frames cannot satisfy this mode.
 * Colors: red1/green2/blue3/black4/yellow7. Shapes: hex1/cylinder3/
 * pentagonal prism4/cuboid5/model label b奶娃6. */
uint8_t VisionObserve_StartTarget(uint8_t color, uint8_t shape);
uint8_t VisionObserve_Stop(void);
uint8_t VisionObserve_Read(VisionObservation *out, uint32_t max_age_ms);

#endif
