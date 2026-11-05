#ifndef ROBOT_LOGISTICS_TASK_H
#define ROBOT_LOGISTICS_TASK_H
#include "main.h"
#include "logistics_config.h"
typedef enum {
    LOG_IDLE, LOG_PREPARE, LOG_ALIGN, LOG_PICK_LOWER, LOG_GRIP,
    LOG_PICK_RAISE, LOG_FOLD, LOG_OUTBOUND, LOG_UNFOLD,
    LOG_PLACE_LOWER, LOG_RELEASE, LOG_PLACE_RAISE, LOG_RETURN,
    LOG_DONE, LOG_ABORTED, LOG_FAULT,
    /* Append to preserve existing UART status numbers. */
    LOG_STORE_LOWER, LOG_STORE_RELEASE, LOG_STORE_RAISE, LOG_LOAD_CLEAR,
    LOG_LOAD_HOME, LOG_FETCH_TURN, LOG_FETCH_LOWER, LOG_FETCH_GRIP,
    LOG_FETCH_RAISE, LOG_DELIVERY_FOLD
} LogisticsState;
enum { LOG_ERROR_NONE, LOG_ERROR_FEEDBACK, LOG_ERROR_IMU,
       LOG_ERROR_MOTOR_UART, LOG_ERROR_VISION, LOG_ERROR_ALIGN_LIMIT,
       LOG_ERROR_MOVE_TIMEOUT, LOG_ERROR_LIFT_TIMEOUT, LOG_ERROR_HEADING,
       LOG_ERROR_SERVO, LOG_ERROR_LIFT_NO_PROGRESS, LOG_ERROR_PLAN };
extern volatile uint8_t logistics_state, logistics_drop_index, logistics_error;
extern volatile float logistics_x_m, logistics_y_m;
void Logistics_Init(void);
/* Three [color, shape] pairs: colors 1/2/3/4/7, shapes 1/3/4/5/6.
 * Model combinations: shape 1 black only; shape 3 excludes yellow;
 * shape 4 red/green/yellow; shapes 5/6 accept all five listed colors.
 * SetPlan is rejected while running. Cancel retains the plan; ClearPlan
 * cancels any running task before invalidating it for a new control session. */
uint8_t Logistics_SetPlan(const uint8_t *items, uint8_t length);
uint8_t Logistics_PlanMatches(const uint8_t *items, uint8_t length);
uint8_t Logistics_PlanReady(void);
void Logistics_ClearPlan(void);
uint8_t Logistics_Start(void);
void Logistics_Cancel(void);
void Logistics_Task(void);
uint8_t Logistics_IsRunning(void);
/* Own length-based parser; a 0x6B inside encoder data is NOT a delimiter. */
void Logistics_MotorRxByteISR(uint8_t byte);
/* Atomic copy of axis 5 encoder feedback. Returns 0 until a sample is received.
 * Caller checks the timestamp; this is relative feedback, not a homed height. */
uint8_t Logistics_GetLiftFeedback(int64_t *count, uint32_t *tick, uint32_t *serial);
#endif
