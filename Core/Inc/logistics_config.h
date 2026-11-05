#ifndef ROBOT_LOGISTICS_CONFIG_H
#define ROBOT_LOGISTICS_CONFIG_H

/* Metres. Car front points toward field +Y at start; X points right. */
#define LOG_FIELD_M                 2.00f
#define LOG_DOCK_MARGIN_M           0.30f
#define LOG_PICK_X_M                LOG_DOCK_MARGIN_M
#define LOG_PICK_Y_M                LOG_DOCK_MARGIN_M
#define LOG_FAR_M                   (LOG_FIELD_M - LOG_DOCK_MARGIN_M)
#define LOG_CORRIDOR_X_M            (LOG_FIELD_M * 0.5f)
#define LOG_WHEEL_CIRCUMFERENCE_M   0.2293f
#define LOG_ENCODER_COUNTS_REV      65536.0f
#define LOG_LIFT_METRES_REV         (0.165f / 3.7f)
#define LOG_LIFT_PULSES_METRE        (3200.0f / LOG_LIFT_METRES_REV)
/* Reused from the original grasp routine. Start at the transport height.
 * No automatic lift homing exists: set the reference physically before START. */
#define LOG_PICK_LOWER_M            0.078f
#define LOG_PLACE_LOWER_M           0.078f
/* Original function.c: lower into the tray by 0.042 m; retrieve by 0.050 m.
 * Both are relative to the same transport height. Calibrate for actual cargo. */
#define LOG_STORE_LOWER_M           0.042f
#define LOG_FETCH_LOWER_M           0.050f
/* Three tray positions already defined by duiji.h; slot order is load order. */
#define LOG_TRAY_SLOT_ANGLES        {HONG, LV, LAN}
#define LOG_LIFT_RPM                60U
#define LOG_LIFT_DOWN_DIRECTION     1U
#define LOG_LIFT_TOLERANCE_M        0.0015f
#define LOG_LIFT_TIMEOUT_MS         12000U
/* Motion supervision, not a force sensor or a substitute for driver protection. */
#define LOG_LIFT_NO_PROGRESS_MS     1000U
#define LOG_LIFT_PROGRESS_COUNTS    128
#define LOG_POSITION_TOLERANCE_M    0.010f
#define LOG_MAX_RPS                 0.60f
#define LOG_POSITION_GAIN          1.5f
#define LOG_HEADING_GAIN            0.20f
#define LOG_MAX_OMEGA               0.15f
#define LOG_MAX_HEADING_ERROR_DEG   20.0f
#define LOG_IMU_TIMEOUT_MS          500U
#define LOG_FEEDBACK_TIMEOUT_MS     600U
#define LOG_SEGMENT_TIMEOUT_MS      45000U
#define LOG_CONTROL_PERIOD_MS      70U
#define LOG_POLL_PERIOD_MS          25U
#define LOG_SERVO_WAIT_MS           800U
#define LOG_GRIP_WAIT_MS            500U
#define LOG_VISION_TIMEOUT_MS       250U
#define LOG_ALIGN_TIMEOUT_MS        20000U
#define LOG_ALIGN_MAX_OFFSET_M      0.08f
/* Original state.c uses jiaozhun5(165,133). These are grasp pixels, not
 * necessarily the geometric image centre. Recalibrate after moving camera. */
#define LOG_ALIGN_TARGET_X          165.0f
#define LOG_ALIGN_TARGET_Y          133.0f
#define LOG_ALIGN_DEADBAND_PX       8.0f
#define LOG_ALIGN_MAX_RPS           0.08f
/* Camera mounting convention reused from vision_follow.h. Calibrate it. */
#define LOG_ALIGN_EX_TO_FORWARD     1.0f
#define LOG_ALIGN_EY_TO_FORWARD     0.0f
#define LOG_ALIGN_EX_TO_LEFT        0.0f
#define LOG_ALIGN_EY_TO_LEFT       -1.0f
/* Explicit [color, shape] pairs in load/delivery order; no implicit targets. */
#define LOG_PLAN_ITEM_COUNT         3U
#define LOG_PLAN_BYTES              (LOG_PLAN_ITEM_COUNT * 2U)
#endif
