#ifndef robot_WEB_CONTROL_H
#define robot_WEB_CONTROL_H

#include "main.h"

/* Set to 0 to restore the legacy application and VisionFollow_Task. */
#ifndef WEB_CONTROL_ENABLED
#define WEB_CONTROL_ENABLED 1
#endif

/* No homing or absolute travel limit is commissioned. Manual lift is a finite
 * relative step, supervised by encoder feedback; release before another step. */
#ifndef WEB_LIFT_JOG_ENABLED
#define WEB_LIFT_JOG_ENABLED 1
#endif

#define WEB_LINK_LEASE_MS 500U
#define WEB_JOG_LEASE_MS 350U
#define WEB_STATUS_PERIOD_MS 200U
#define WEB_MOTOR_PERIOD_MS 60U
/* Use the original chassis driver's acceleration setting for moving commands. */
#define WEB_CHASSIS_ACCEL 180U
/* Manual web control: twice the previous translation and turn command speeds. */
#define WEB_CHASSIS_RPS_PER_PERCENT 0.01f
#define WEB_TURN_PER_PERCENT 0.02f
#define WEB_LIFT_RPM 15U
#define WEB_LIFT_ACCEL 80U
#define WEB_LIFT_STEP_M 0.005f
#define WEB_LIFT_STEP_TOLERANCE_M 0.0005f
#define WEB_LIFT_JOG_MAX_MS 1000U
#define WEB_LIFT_NO_PROGRESS_MS 500U
#define WEB_LIFT_FEEDBACK_TIMEOUT_MS 600U

/* STATUS fault field; these are controller faults, not motor feedback. */
#define WEB_FAULT_NONE 0U
#define WEB_FAULT_LINK_TIMEOUT 1U
#define WEB_FAULT_RX_OVERFLOW 2U
#define WEB_FAULT_UART 3U
#define WEB_FAULT_INTERLOCK 4U
#define WEB_FAULT_JOG_TIMEOUT 5U
#define WEB_FAULT_MOTOR_UART 6U
#define WEB_FAULT_AUTOMATIC 7U
#define WEB_FAULT_LIFT_FEEDBACK 8U
#define WEB_FAULT_LIFT_NO_PROGRESS 9U
#define WEB_FAULT_LIFT_TIMEOUT 10U

void WebControl_Init(void);
uint8_t WebControl_Task(void);
/* Called by UART4 callbacks only; never send motor commands or ACKs here. */
void WebControl_RxCompleteISR(void);
void WebControl_UartErrorISR(void);

#endif
