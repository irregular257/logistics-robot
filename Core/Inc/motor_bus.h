#ifndef ROBOT_MOTOR_BUS_H
#define ROBOT_MOTOR_BUS_H

#include <stdint.h>

void MotorBus_Init(void);
void MotorBus_Run(void *argument);
/* Hold a transaction across all wheel targets and their synchronization frame.
 * Transmit also acquires this recursive lock for a standalone frame. */
void MotorBus_Begin(void);
void MotorBus_End(void);
uint8_t MotorBus_Transmit(const uint8_t *data, uint16_t size, uint32_t gap_ms);

#endif
