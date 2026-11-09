#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>

typedef struct {
    uint8_t state;
    uint8_t drop_index;
    uint8_t error;
    uint8_t running;
    uint8_t plan_ready;
    float x_m;
    float y_m;
} RobotStatus;

void RobotControl_Init(void);
void RobotControl_Run(void *argument);
void RobotControl_ReadStatus(RobotStatus *status);
uint8_t RobotControl_IsAutomatic(void);
/* Commands return WEB_RESULT_* after the automatic task has handled them.
 * Only that task may change the logistics state machine and accepted plan.
 * The web command dispatcher is the sole command producer. */
uint8_t RobotControl_SetPlan(const uint8_t *items, uint8_t length);
uint8_t RobotControl_Start(const uint8_t *items, uint8_t length);
uint8_t RobotControl_Stop(void);
uint8_t RobotControl_GetLiftFeedback(int64_t *count, uint32_t *tick, uint32_t *serial);

#endif
