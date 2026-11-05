#include "web_control.h"
#include "web_protocol.h"
#include "usart.h"
#include "tim.h"
#include "duiji.h"
#include "vision_follow.h"
#include "logistics_task.h"
#include <math.h>
#include <string.h>

#if WEB_CONTROL_ENABLED

#define RX_CAPACITY 256U
#define ACK_CACHE_COUNT 8U
#define PARSER_GAP_MS 40U
#define RX_FAULT_OVERFLOW 1U
#define RX_FAULT_UART 2U

extern int start, zero;

static uint8_t rx_byte;
static uint8_t rx_bytes[RX_CAPACITY];
static uint32_t rx_ticks[RX_CAPACITY];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_faults;
static uint8_t parser[WEB_MAX_FRAME];
static uint8_t parser_count;
static uint32_t parser_tick;
static uint32_t active_session, link_tick, jog_tick, motor_tick, status_tick;
static uint16_t highest_sequence, status_sequence;
static uint8_t session_valid, mode, fault, claw_target = 2U, jog_active;
static uint8_t motion_dirty;
static uint16_t gimbal_target = WEB_GIMBAL_UNKNOWN_ANGLE;
static uint16_t claw_angle_target = WEB_GIMBAL_UNKNOWN_ANGLE;
static uint16_t tray_target = WEB_TRAY_UNKNOWN_ANGLE;
static int8_t jog[4];
static uint8_t manual_lift_active, manual_lift_latched, manual_lift_direction;
static int64_t manual_lift_origin, manual_lift_progress_count;
static uint32_t manual_lift_start_tick, manual_lift_progress_tick, manual_lift_serial;

typedef struct {
    WebFrame request;
    uint8_t result, valid;
} AckEntry;
static AckEntry ack_cache[ACK_CACHE_COUNT];
static uint8_t ack_next;
static WebFrame pending_jog_ack;
static uint8_t pending_jog_valid;

static uint32_t lock_irq(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}

static void unlock_irq(uint32_t saved)
{
    if (saved == 0U) __enable_irq();
}

static void discard_rx(void)
{
    uint32_t saved = lock_irq();
    rx_tail = rx_head;
    unlock_irq(saved);
    parser_count = 0U;
}

/* The existing F3/AB, F6, FD, FF/66 commands identify the Emm V5 protocol.
 * Its immediate-stop command includes the sync flag:
 * [address, FE, 98, sync(0=immediate), 6B]. It also cancels position motion.
 * All five axes are stopped, including finite manual lift motion. */
static uint8_t motor_send(uint8_t *data, uint8_t size)
{
    uint8_t ok = HAL_UART_Transmit(&huart1, data, size, 10U) == HAL_OK;
    if (!ok) fault = WEB_FAULT_MOTOR_UART;
    HAL_Delay(1U);
    return ok;
}

/* Network control uses bounded copies of motor_speed/motor_sync because the
 * legacy helpers each permit a 1000 ms UART timeout. The byte formats match. */
static uint8_t bounded_speed(uint8_t id, uint8_t direction, uint16_t rpm, uint8_t accel)
{
    uint8_t ok;
    uint8_t command[8] = {id, 0xF6U, direction, (uint8_t)(rpm >> 8),
                         (uint8_t)rpm, accel, 1U, 0x6BU};
    ok = motor_send(command, sizeof(command));
    HAL_Delay(9U); /* preserve the legacy speed command's 10 ms frame gap */
    return ok;
}

static uint8_t bounded_sync(void)
{
    uint8_t ok;
    uint8_t command[4] = {0U, 0xFFU, 0x66U, 0x6BU};
    ok = motor_send(command, sizeof(command));
    HAL_Delay(9U); /* preserve the legacy sync command's 10 ms frame gap */
    return ok;
}

static uint8_t stop_all(void)
{
    uint8_t id;
    uint8_t ok = 1U;
    uint8_t command[5] = {0U, 0xFEU, 0x98U, 0U, 0x6BU};
    jog_active = 0U;
    motion_dirty = 0U;
    manual_lift_active = 0U;
    memset(jog, 0, sizeof(jog));
    for (id = 1U; id <= 5U; ++id) {
        command[0] = id;
        if (!motor_send(command, sizeof(command))) ok = 0U;
    }
    /* Zero-speed commands are an additional fallback, not the position cancel. */
    for (id = 1U; id <= 5U; ++id)
        if (!bounded_speed(id, 0U, 0U, 0U)) ok = 0U;
    if (!bounded_sync()) ok = 0U;
    return ok;
}

static void stop_idle(uint8_t reason)
{
    Logistics_Cancel();
    manual_lift_latched = 0U;
    mode = WEB_MODE_IDLE;
    fault = reason;
    (void)stop_all();
    start = 0;
    zero = 0;
    vision_follow_enable = 0U;
}

static uint8_t lift_feedback(int64_t *count, uint32_t *serial)
{
    uint32_t tick;
    if (!Logistics_GetLiftFeedback(count, &tick, serial)) return 0U;
    /* Read time after copying ISR data so a new sample cannot appear future. */
    return (uint32_t)(HAL_GetTick() - tick) < WEB_LIFT_FEEDBACK_TIMEOUT_MS;
}

static uint8_t lift_available(void)
{
    int64_t count;
    uint32_t serial;
    return WEB_LIFT_JOG_ENABLED && fault == WEB_FAULT_NONE &&
           lift_feedback(&count, &serial);
}

static uint8_t begin_manual_lift(void)
{
    uint32_t pulse, serial;
    int64_t count;
    uint8_t command[13] = {5U, 0xFDU, 0U, 0U, 0U, WEB_LIFT_ACCEL,
                           0U, 0U, 0U, 0U, 0U, 1U, 0x6BU};
    if (!lift_feedback(&count, &serial)) {
        fault = WEB_FAULT_LIFT_FEEDBACK;
        return 0U;
    }
    manual_lift_origin = manual_lift_progress_count = count;
    manual_lift_serial = serial;
    manual_lift_start_tick = manual_lift_progress_tick = HAL_GetTick();
    manual_lift_direction = jog[3] > 0 ? (1U - LOG_LIFT_DOWN_DIRECTION) :
                                        LOG_LIFT_DOWN_DIRECTION;
    pulse = (uint32_t)(WEB_LIFT_STEP_M * LOG_LIFT_PULSES_METRE + 0.5f);
    command[2] = manual_lift_direction;
    command[3] = (uint8_t)(WEB_LIFT_RPM >> 8);
    command[4] = (uint8_t)WEB_LIFT_RPM;
    command[6] = (uint8_t)(pulse >> 24); command[7] = (uint8_t)(pulse >> 16);
    command[8] = (uint8_t)(pulse >> 8); command[9] = (uint8_t)pulse;
    /* A finite relative position request provides a second motion bound.
     * Repeated web JOGs only renew the lease, never issue another position. */
    if (!motor_send(command, sizeof(command)) || !bounded_sync()) return 0U;
    manual_lift_active = manual_lift_latched = 1U;
    return 1U;
}

static void check_manual_lift(void)
{
    int64_t count, progress;
    uint32_t serial, now;
    float distance;
    if (!manual_lift_active) return;
    if (!lift_feedback(&count, &serial)) {
        stop_idle(WEB_FAULT_LIFT_FEEDBACK);
        return;
    }
    now = HAL_GetTick();
    if ((uint32_t)(now - manual_lift_start_tick) >= WEB_LIFT_JOG_MAX_MS) {
        stop_idle(WEB_FAULT_LIFT_TIMEOUT);
        return;
    }
    distance = (float)(count - manual_lift_origin) *
               LOG_LIFT_METRES_REV / LOG_ENCODER_COUNTS_REV;
    if (manual_lift_direction) distance = -distance;
    if (distance >= WEB_LIFT_STEP_M - WEB_LIFT_STEP_TOLERANCE_M) {
        if (!stop_all()) mode = WEB_MODE_IDLE;
        /* Keep the press latched after reaching the target. Only release or
         * a different non-lift command can make the next press move again. */
        return;
    }
    if (serial != manual_lift_serial) {
        manual_lift_serial = serial;
        progress = count - manual_lift_progress_count;
        if (manual_lift_direction) progress = -progress;
        if (progress <= -LOG_LIFT_PROGRESS_COUNTS) {
            stop_idle(WEB_FAULT_LIFT_NO_PROGRESS);
            return;
        }
        if (progress >= LOG_LIFT_PROGRESS_COUNTS) {
            manual_lift_progress_count = count;
            manual_lift_progress_tick = now;
        }
    }
    if ((uint32_t)(now - manual_lift_progress_tick) >= WEB_LIFT_NO_PROGRESS_MS)
        stop_idle(WEB_FAULT_LIFT_NO_PROGRESS);
}

static void send_frame(const WebFrame *frame)
{
    uint8_t data[WEB_MAX_FRAME];
    uint8_t count = Web_Encode(frame, data);
    if (count && HAL_UART_Transmit(&huart4, data, count, 10U) != HAL_OK)
        rx_faults |= RX_FAULT_UART;
}

static void send_ack(const WebFrame *request, uint8_t result)
{
    WebFrame reply;
    reply.session = request->session;
    reply.sequence = request->sequence;
    reply.command = WEB_CMD_ACK;
    reply.length = 4U;
    reply.payload[0] = request->command;
    reply.payload[1] = result;
    reply.payload[2] = mode;
    reply.payload[3] = fault;
    send_frame(&reply);
}

static void send_status(uint32_t now)
{
    WebFrame reply;
    uint8_t i;
    if (!session_valid || (uint32_t)(now - status_tick) < WEB_STATUS_PERIOD_MS)
        return;
    status_tick = now;
    reply.session = active_session;
    reply.sequence = status_sequence++;
    reply.command = WEB_CMD_STATUS;
    reply.length = WEB_TASK_STATUS_SIZE;
    reply.payload[0] = mode;
    reply.payload[1] = fault;
    reply.payload[2] = claw_target; /* commanded target, never grip feedback */
    reply.payload[3] = lift_available() ? 1U : 0U;
    for (i = 0U; i < 4U; ++i) reply.payload[4U + i] = (uint8_t)jog[i];
    reply.payload[8]=logistics_state;
    reply.payload[9]=logistics_drop_index;
    reply.payload[10]=logistics_error;
    reply.payload[11]=1U;
    Web_Put16(reply.payload+12,(uint16_t)(int16_t)(logistics_x_m*100.0f));
    Web_Put16(reply.payload+14,(uint16_t)(int16_t)(logistics_y_m*100.0f));
    Web_Put16(reply.payload+16,gimbal_target); /* target, not measured position */
    Web_Put16(reply.payload+18,claw_angle_target);
    Web_Put16(reply.payload+20,Duoji_GetClawPreset(0U));
    Web_Put16(reply.payload+22,Duoji_GetClawPreset(1U));
    Web_Put16(reply.payload+24,Duoji_GetGimbalPreset(0U));
    Web_Put16(reply.payload+26,Duoji_GetGimbalPreset(1U));
    Web_Put16(reply.payload+28,tray_target); /* target, not measured position */
    reply.payload[30] = WEB_TASK_PROTOCOL_VERSION;
    reply.payload[31] = Logistics_PlanReady();
    send_frame(&reply);
}

static uint8_t sequence_newer(uint16_t candidate)
{
    uint16_t delta = (uint16_t)(candidate - highest_sequence);
    return delta != 0U && delta < 0x8000U;
}

static int cached_result(const WebFrame *request)
{
    uint8_t i;
    for (i = 0U; i < ACK_CACHE_COUNT; ++i) {
        AckEntry *entry = &ack_cache[i];
        if (entry->valid && entry->request.sequence == request->sequence &&
            entry->request.command == request->command &&
            entry->request.length == request->length &&
            memcmp(entry->request.payload, request->payload, request->length) == 0)
            return entry->result;
    }
    return -1;
}

static void cache_ack(const WebFrame *request, uint8_t result)
{
    ack_cache[ack_next].request = *request;
    ack_cache[ack_next].result = result;
    ack_cache[ack_next].valid = 1U;
    ack_next = (uint8_t)((ack_next + 1U) % ACK_CACHE_COUNT);
}

static uint8_t same_request(const WebFrame *a, const WebFrame *b)
{
    return a->session == b->session && a->sequence == b->sequence &&
           a->command == b->command && a->length == b->length &&
           memcmp(a->payload, b->payload, a->length) == 0;
}

static void finish_jog_ack(uint8_t result)
{
    if (!pending_jog_valid) return;
    cache_ack(&pending_jog_ack, result);
    send_ack(&pending_jog_ack, result);
    pending_jog_valid = 0U;
}

static uint8_t apply_motion(void)
{
    uint8_t i;
    float forward, left, rotation, wheel[4], magnitude;
#if WEB_LIFT_JOG_ENABLED
    if (jog[3] != 0) {
        return begin_manual_lift();
    }
#endif
    /* Manual calibration for the reported vehicle: the legacy lateral and
     * yaw wheel patterns produced each other's motion. Keep protocol x=right+
     * and yaw=CCW+, with their existing, separate speed scales. This mapping
     * applies only to web jogging; automatic motion/odometry need their own
     * physical commissioning before using an automatic route. */
    forward = (float)jog[1] * WEB_CHASSIS_RPS_PER_PERCENT * 60.0f;
    left = -(float)jog[0] * WEB_CHASSIS_RPS_PER_PERCENT * 60.0f;
    rotation = (float)jog[2] * WEB_TURN_PER_PERCENT * 60.0f * (0.105f + 0.1075f);
    wheel[0] = forward - left - rotation;
    wheel[1] = -(forward + left + rotation);
    wheel[2] = forward - left + rotation;
    wheel[3] = -(forward + left - rotation);
    for (i = 0U; i < 4U; ++i) {
        magnitude = fabsf(wheel[i]);
        if (!bounded_speed((uint8_t)(i + 1U), wheel[i] < 0.0f ? 1U : 0U,
                           (uint16_t)magnitude, WEB_CHASSIS_ACCEL)) return 0U;
    }
    return bounded_sync();
}

static void handle_frame(const WebFrame *request, uint32_t received_at)
{
    uint8_t result = WEB_RESULT_OK;
    uint8_t i, chassis, changed;
    uint32_t now = HAL_GetTick();
    int duplicate;
    int8_t next_jog[4];

    /* STOP is unconditional for every CRC-valid protocol frame, even with
     * an old session/sequence or malformed payload. It never arms a session. */
    if (request->command == WEB_CMD_STOP) {
        stop_idle(WEB_FAULT_NONE);
        finish_jog_ack(WEB_RESULT_INTERLOCK);
        if (fault == WEB_FAULT_MOTOR_UART) result = WEB_RESULT_INTERLOCK;
        else if (request->length != 0U) result = WEB_RESULT_BAD_ARGUMENT;
        if (session_valid && request->session == active_session &&
            sequence_newer(request->sequence)) {
            highest_sequence = request->sequence;
            link_tick = received_at;
            cache_ack(request, result);
        }
        send_ack(request, result);
        return;
    }

    if (request->command == WEB_CMD_HELLO) {
        if (request->length != 0U) {
            send_ack(request, WEB_RESULT_BAD_ARGUMENT);
            return;
        }
        if (session_valid && request->session == active_session) {
            /* A retry only receives an ACK: no stopping, arming or leases. */
            send_ack(request, fault == WEB_FAULT_MOTOR_UART ?
                     WEB_RESULT_INTERLOCK : WEB_RESULT_OK);
            return;
        }
        stop_idle(WEB_FAULT_NONE);
        pending_jog_valid = 0U;
        Logistics_ClearPlan();
        discard_rx(); /* old queued actions must never follow a new handshake */
        active_session = request->session;
        highest_sequence = request->sequence;
        status_sequence = 0U;
        session_valid = 1U;
        link_tick = received_at;
        status_tick = HAL_GetTick();
        memset(ack_cache, 0, sizeof(ack_cache));
        ack_next = 0U;
        send_ack(request, fault == WEB_FAULT_MOTOR_UART ?
                 WEB_RESULT_INTERLOCK : WEB_RESULT_OK);
        return;
    }

    if (!session_valid || request->session != active_session) {
        send_ack(request, WEB_RESULT_STALE);
        return;
    }
    if (!sequence_newer(request->sequence)) {
        if (pending_jog_valid && same_request(request, &pending_jog_ack)) return;
        duplicate = cached_result(request);
        /* A retry never executes and never refreshes either lease. */
        send_ack(request, duplicate >= 0 ? (uint8_t)duplicate : WEB_RESULT_STALE);
        return;
    }
    highest_sequence = request->sequence;

    switch (request->command) {
    case WEB_CMD_HEARTBEAT:
        if (request->length != 0U) result = WEB_RESULT_BAD_ARGUMENT;
        else link_tick = received_at; /* no jog_tick update */
        break;
    case WEB_CMD_MODE:
        if (request->length != 1U || request->payload[0] > WEB_MODE_MANUAL) {
            result = WEB_RESULT_BAD_ARGUMENT;
        } else if ((uint32_t)(now - received_at) >= WEB_LINK_LEASE_MS) {
            result = WEB_RESULT_STALE;
        } else {
            stop_idle(WEB_FAULT_NONE);
            finish_jog_ack(WEB_RESULT_INTERLOCK);
            if (fault == WEB_FAULT_MOTOR_UART) result = WEB_RESULT_INTERLOCK;
            else mode = request->payload[0];
            link_tick = received_at;
        }
        break;
    case WEB_CMD_JOG:
        if (request->length != 4U) {
            result = WEB_RESULT_BAD_ARGUMENT;
            stop_idle(WEB_FAULT_INTERLOCK);
            break;
        }
        for (i = 0U; i < 4U; ++i) next_jog[i] = (int8_t)request->payload[i];
        chassis = next_jog[0] != 0 || next_jog[1] != 0 || next_jog[2] != 0;
        if (next_jog[0] < -50 || next_jog[0] > 50 ||
            next_jog[1] < -50 || next_jog[1] > 50 ||
            next_jog[2] < -50 || next_jog[2] > 50 ||
            next_jog[3] < -1 || next_jog[3] > 1) {
            result = WEB_RESULT_BAD_ARGUMENT;
            stop_idle(WEB_FAULT_INTERLOCK);
        } else if ((chassis && next_jog[3] != 0) ||
                   (!WEB_LIFT_JOG_ENABLED && next_jog[3] != 0)) {
            result = WEB_RESULT_INTERLOCK;
            stop_idle(WEB_FAULT_INTERLOCK);
        } else if (mode != WEB_MODE_MANUAL) {
            result = WEB_RESULT_WRONG_MODE;
        } else if ((uint32_t)(now - received_at) >= WEB_JOG_LEASE_MS) {
            result = WEB_RESULT_STALE;
            stop_idle(WEB_FAULT_JOG_TIMEOUT);
        } else {
            if (next_jog[3] != 0 && manual_lift_latched) {
                uint8_t direction = next_jog[3] > 0 ? (1U - LOG_LIFT_DOWN_DIRECTION) :
                                                    LOG_LIFT_DOWN_DIRECTION;
                if (direction != manual_lift_direction) {
                    result = WEB_RESULT_INTERLOCK;
                    stop_idle(WEB_FAULT_INTERLOCK);
                } else {
                    jog_tick = link_tick = received_at;
                    /* At the end of this press keep ACKing, without rearming. */
                }
                break;
            }
            if (next_jog[3] != 0 && !lift_available()) {
                result = WEB_RESULT_INTERLOCK;
                stop_idle(WEB_FAULT_LIFT_FEEDBACK);
                break;
            }
            if (next_jog[3] == 0) manual_lift_latched = 0U;
            changed = !jog_active || memcmp(jog, next_jog, sizeof(jog)) != 0;
            if ((!chassis && next_jog[3] == 0) ||
                (jog_active && (jog[3] != 0) != (next_jog[3] != 0))) {
                if (!stop_all()) {
                    mode = WEB_MODE_IDLE;
                    result = WEB_RESULT_INTERLOCK;
                    break;
                }
            }
            finish_jog_ack(WEB_RESULT_STALE);
            memcpy(jog, next_jog, sizeof(jog));
            jog_active = chassis || next_jog[3] != 0;
            jog_tick = received_at;
            link_tick = received_at;
            fault = WEB_FAULT_NONE;
            if (jog_active && (changed || motion_dirty)) {
                /* New targets wait for motor I/O. An unchanged fresh JOG only
                 * renews the lease; continuous speed mode retains its target. */
                motion_dirty = 1U;
                pending_jog_ack = *request;
                pending_jog_valid = 1U;
                return;
            }
        }
        break;
    case WEB_CMD_CLAW:
        if (request->length != 1U || request->payload[0] > 1U) {
            result = WEB_RESULT_BAD_ARGUMENT;
        } else if (mode != WEB_MODE_MANUAL) {
            result = WEB_RESULT_WRONG_MODE;
        } else if (jog_active || pending_jog_valid) {
            result = WEB_RESULT_INTERLOCK;
        } else if ((uint32_t)(now - received_at) >= WEB_JOG_LEASE_MS) {
            result = WEB_RESULT_STALE;
        } else {
            uint16_t angle = Duoji_GetClawPreset(request->payload[0]);
            if (JiaZhua_SetAngle(angle) != HAL_OK) {
                claw_target = 2U;
                claw_angle_target = WEB_GIMBAL_UNKNOWN_ANGLE;
                result = WEB_RESULT_INTERLOCK;
                stop_idle(WEB_FAULT_INTERLOCK);
            } else {
                claw_target = request->payload[0];
                claw_angle_target = angle;
                link_tick = received_at;
            }
        }
        break;
    case WEB_CMD_CLAW_ANGLE:
        if (request->length != 2U || Web_Get16(request->payload) > WEB_CLAW_MAX_ANGLE) {
            result = WEB_RESULT_BAD_ARGUMENT;
        } else if (mode != WEB_MODE_MANUAL) {
            result = WEB_RESULT_WRONG_MODE;
        } else if (jog_active || pending_jog_valid || fault != WEB_FAULT_NONE) {
            result = WEB_RESULT_INTERLOCK;
        } else if ((uint32_t)(now - received_at) >= WEB_JOG_LEASE_MS) {
            result = WEB_RESULT_STALE;
        } else {
            uint16_t angle = Web_Get16(request->payload);
            if (JiaZhua_SetAngle(angle) != HAL_OK) {
                claw_target = 2U; claw_angle_target = WEB_GIMBAL_UNKNOWN_ANGLE;
                result = WEB_RESULT_INTERLOCK;
                stop_idle(WEB_FAULT_INTERLOCK);
            } else {
                claw_angle_target = angle;
                claw_target = angle == Duoji_GetClawPreset(0U) ? 0U :
                              angle == Duoji_GetClawPreset(1U) ? 1U : 2U;
                link_tick = received_at;
            }
        }
        break;
    case WEB_CMD_SERVO_PRESETS:
        if (request->length != 8U ||
            Web_Get16(request->payload) > WEB_CLAW_MAX_ANGLE ||
            Web_Get16(request->payload + 2) > WEB_CLAW_MAX_ANGLE ||
            Web_Get16(request->payload + 4) > WEB_GIMBAL_MAX_ANGLE ||
            Web_Get16(request->payload + 6) > WEB_GIMBAL_MAX_ANGLE) {
            result = WEB_RESULT_BAD_ARGUMENT;
        } else if (mode != WEB_MODE_IDLE && mode != WEB_MODE_MANUAL) {
            result = WEB_RESULT_WRONG_MODE;
        } else if (jog_active || pending_jog_valid || fault != WEB_FAULT_NONE) {
            result = WEB_RESULT_INTERLOCK;
        } else if ((uint32_t)(now - received_at) >= WEB_JOG_LEASE_MS) {
            result = WEB_RESULT_STALE;
        } else if (Duoji_SetPresets(Web_Get16(request->payload), Web_Get16(request->payload + 2),
                                   Web_Get16(request->payload + 4), Web_Get16(request->payload + 6)) != HAL_OK) {
            result = WEB_RESULT_BAD_ARGUMENT;
        } else {
            claw_target = claw_angle_target == Duoji_GetClawPreset(0U) ? 0U :
                          claw_angle_target == Duoji_GetClawPreset(1U) ? 1U : 2U;
            link_tick = received_at;
        }
        break;
    case WEB_CMD_GIMBAL:
        if (request->length != 2U ||
            Web_Get16(request->payload) > WEB_GIMBAL_MAX_ANGLE) {
            result = WEB_RESULT_BAD_ARGUMENT;
        } else if (mode != WEB_MODE_MANUAL) {
            result = WEB_RESULT_WRONG_MODE;
        } else if (jog_active || pending_jog_valid) {
            result = WEB_RESULT_INTERLOCK;
        } else if ((uint32_t)(now - received_at) >= WEB_JOG_LEASE_MS) {
            result = WEB_RESULT_STALE;
        } else {
            uint16_t angle = Web_Get16(request->payload);
            /* Set the compare value before first enabling its PWM output. */
            YunTai(angle);
            if (Duoji_StartPWM(TIM_CHANNEL_3) != HAL_OK) {
                gimbal_target = WEB_GIMBAL_UNKNOWN_ANGLE;
                result = WEB_RESULT_INTERLOCK;
                stop_idle(WEB_FAULT_INTERLOCK);
            } else {
                gimbal_target = angle;
                link_tick = received_at;
            }
        }
        break;
    case WEB_CMD_TRAY_ANGLE:
        if (request->length != 2U ||
            Web_Get16(request->payload) > WEB_TRAY_MAX_ANGLE) {
            result = WEB_RESULT_BAD_ARGUMENT;
        } else if (mode != WEB_MODE_MANUAL) {
            result = WEB_RESULT_WRONG_MODE;
        } else if (jog_active || pending_jog_valid || manual_lift_active ||
                   fault != WEB_FAULT_NONE) {
            result = WEB_RESULT_INTERLOCK;
        } else if ((uint32_t)(now - received_at) >= WEB_JOG_LEASE_MS) {
            result = WEB_RESULT_STALE;
        } else {
            uint16_t angle = Web_Get16(request->payload);
            if (Tray_SetAngle(angle) != HAL_OK) {
                tray_target = WEB_TRAY_UNKNOWN_ANGLE;
                result = WEB_RESULT_INTERLOCK;
                stop_idle(WEB_FAULT_INTERLOCK);
            } else {
                tray_target = angle;
                link_tick = received_at;
            }
        }
        break;
    case WEB_CMD_TASK:
        if (request->length != WEB_TASK_PLAN_SIZE) result = WEB_RESULT_BAD_ARGUMENT;
        else if (mode == WEB_MODE_AUTO || Logistics_IsRunning()) result = WEB_RESULT_WRONG_MODE;
        else if ((uint32_t)(now-received_at) >= WEB_LINK_LEASE_MS) result = WEB_RESULT_STALE;
        else if (fault != WEB_FAULT_NONE || jog_active || pending_jog_valid || manual_lift_active)
            result = WEB_RESULT_INTERLOCK;
        else if (!Logistics_SetPlan(request->payload, request->length)) result = WEB_RESULT_BAD_ARGUMENT;
        else link_tick = received_at;
        break;
    case WEB_CMD_START:
        if (request->length!=(1U+WEB_TASK_PLAN_SIZE) || request->payload[0]!=1U) result=WEB_RESULT_BAD_ARGUMENT;
        else if (mode==WEB_MODE_AUTO || jog_active || pending_jog_valid) result=WEB_RESULT_WRONG_MODE;
        else if ((uint32_t)(now-received_at)>=WEB_LINK_LEASE_MS) result=WEB_RESULT_STALE;
        else if (fault!=WEB_FAULT_NONE) result=WEB_RESULT_INTERLOCK;
        else if (!Logistics_PlanMatches(request->payload+1U, WEB_TASK_PLAN_SIZE)) result=WEB_RESULT_INTERLOCK;
        else {
            /* Automatic preparation may command YunTai even if start fails. */
            gimbal_target=WEB_GIMBAL_UNKNOWN_ANGLE;
            claw_angle_target=WEB_GIMBAL_UNKNOWN_ANGLE;
            tray_target=WEB_TRAY_UNKNOWN_ANGLE;
            if (!stop_all() || !Logistics_Start()) result=WEB_RESULT_INTERLOCK;
            else {
                mode=WEB_MODE_AUTO;
                claw_target=2U; gimbal_target=WEB_GIMBAL_UNKNOWN_ANGLE;
            }
            link_tick=received_at;
        }
        break;
    case WEB_CMD_PAUSE:
        if (request->length!=0U) result=WEB_RESULT_BAD_ARGUMENT;
        else { stop_idle(WEB_FAULT_NONE); link_tick=received_at; }
        break;
    default:
        result = WEB_RESULT_UNSUPPORTED;
        break;
    }
    cache_ack(request, result);
    send_ack(request, result);
}

static void parser_shift(void)
{
    --parser_count;
    memmove(parser, parser + 1, parser_count);
}

static void parse_byte(uint8_t byte, uint32_t received_at)
{
    uint8_t total;
    WebFrame frame;
    if (parser_count && (uint32_t)(received_at - parser_tick) > PARSER_GAP_MS)
        parser_count = 0U;
    parser_tick = received_at;
    parser[parser_count++] = byte;
    while (parser_count >= 2U) {
        if (parser[0] != 0xA5U || parser[1] != 0x5AU) {
            parser_shift();
            continue;
        }
        if (parser_count < 4U) return;
        if (parser[2] != WEB_VERSION || parser[3] > WEB_MAX_PAYLOAD) {
            parser_shift();
            continue;
        }
        total = (uint8_t)(13U + parser[3]);
        if (parser_count < total) return;
        if (Web_Get16(parser + total - 2U) !=
            Web_Crc16(parser + 2, (uint8_t)(total - 4U))) {
            parser_shift();
            continue;
        }
        frame.session = Web_Get32(parser + 4);
        frame.sequence = Web_Get16(parser + 8);
        frame.command = parser[10];
        frame.length = parser[3];
        memcpy(frame.payload, parser + 11, frame.length);
        /* CRC resynchronization can find a short frame inside a rejected
         * longer candidate. Preserve its trailing bytes, including STOP. */
        parser_count = (uint8_t)(parser_count - total);
        memmove(parser, parser + total, parser_count);
        if (!rx_faults) handle_frame(&frame, received_at);
    }
}

static void check_leases(uint32_t now)
{
    if (session_valid && (uint32_t)(now - link_tick) >= WEB_LINK_LEASE_MS) {
        stop_idle(WEB_FAULT_LINK_TIMEOUT);
        session_valid = 0U;
        discard_rx();
    } else if (jog_active && (uint32_t)(now - jog_tick) >= WEB_JOG_LEASE_MS) {
        stop_idle(WEB_FAULT_JOG_TIMEOUT);
        discard_rx();
    }
}

static void handle_rx_fault(void)
{
    uint8_t pending;
    uint32_t saved = lock_irq();
    pending = rx_faults;
    rx_faults = 0U;
    rx_tail = rx_head;
    unlock_irq(saved);
    parser_count = 0U;
    session_valid = 0U;
    pending_jog_valid = 0U;
    stop_idle((pending & RX_FAULT_UART) ? WEB_FAULT_UART : WEB_FAULT_RX_OVERFLOW);
    discard_rx();
    if (pending & RX_FAULT_UART) {
        HAL_UART_AbortReceive(&huart4);
        __HAL_UART_CLEAR_OREFLAG(&huart4);
        if (HAL_UART_Receive_IT(&huart4, &rx_byte, 1U) != HAL_OK)
            rx_faults |= RX_FAULT_UART;
    }
}

void WebControl_Init(void)
{
    Logistics_Init();
    rx_head = 0U;
    rx_tail = 0U;
    rx_faults = 0U;
    parser_count = 0U;
    session_valid = 0U;
    mode = WEB_MODE_IDLE;
    fault = WEB_FAULT_NONE;
    claw_target = 2U;
    claw_angle_target = WEB_GIMBAL_UNKNOWN_ANGLE;
    (void)Duoji_SetPresets(GRAB, OPEN, IN, OUT);
    motion_dirty = 0U;
    gimbal_target = WEB_GIMBAL_UNKNOWN_ANGLE;
    tray_target = WEB_TRAY_UNKNOWN_ANGLE;
    pending_jog_valid = 0U;
    status_tick = motor_tick = HAL_GetTick();
    stop_idle(WEB_FAULT_NONE);
    if (HAL_UART_Receive_IT(&huart4, &rx_byte, 1U) != HAL_OK)
        rx_faults |= RX_FAULT_UART;
}

void WebControl_RxCompleteISR(void)
{
    uint16_t next = (uint16_t)((rx_head + 1U) % RX_CAPACITY);
    if (!rx_faults) {
        if (next == rx_tail) rx_faults |= RX_FAULT_OVERFLOW;
        else {
            rx_bytes[rx_head] = rx_byte;
            rx_ticks[rx_head] = HAL_GetTick();
            __DMB();
            rx_head = next;
        }
    }
    if (HAL_UART_Receive_IT(&huart4, &rx_byte, 1U) != HAL_OK)
        rx_faults |= RX_FAULT_UART;
}

void WebControl_UartErrorISR(void)
{
    rx_faults |= RX_FAULT_UART;
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    /* The main task also aborts/re-arms after HAL finishes its error handling. */
    (void)HAL_UART_Receive_IT(&huart4, &rx_byte, 1U);
}

uint8_t WebControl_Task(void)
{
    uint16_t budget = RX_CAPACITY;
    uint8_t byte;
    uint32_t received_at, now;
    uint8_t motion_ok;

    if (rx_faults) handle_rx_fault();
    if (start || zero || vision_follow_enable) {
        stop_idle(WEB_FAULT_INTERLOCK);
        discard_rx();
    }
    check_leases(HAL_GetTick());
    check_manual_lift();
    while (budget-- && rx_tail != rx_head && !rx_faults) {
        byte = rx_bytes[rx_tail];
        received_at = rx_ticks[rx_tail];
        __DMB();
        rx_tail = (uint16_t)((rx_tail + 1U) % RX_CAPACITY);
        parse_byte(byte, received_at);
        check_leases(HAL_GetTick());
    }
    if (rx_faults) handle_rx_fault();
    now = HAL_GetTick();
    check_leases(now);
    if (jog_active && motion_dirty && mode == WEB_MODE_MANUAL &&
        (uint32_t)(now - motor_tick) >= WEB_MOTOR_PERIOD_MS) {
        motor_tick = now;
        motion_ok = apply_motion();
        if (motion_ok) motion_dirty = 0U;
        if (!motion_ok) stop_idle(fault != WEB_FAULT_NONE ? fault : WEB_FAULT_MOTOR_UART);
        if (rx_faults) handle_rx_fault();
        check_leases(HAL_GetTick());
        finish_jog_ack(motion_ok && jog_active ? WEB_RESULT_OK : WEB_RESULT_INTERLOCK);
    }
    if (pending_jog_valid && !jog_active) finish_jog_ack(WEB_RESULT_INTERLOCK);
    Logistics_Task();
    check_manual_lift();
    if (mode==WEB_MODE_AUTO) {
        if (logistics_state==LOG_FAULT) stop_idle(WEB_FAULT_AUTOMATIC);
        else if (logistics_state==LOG_DONE) mode=WEB_MODE_IDLE;
    }
    /* Recheck after bounded motor I/O and the automatic state-machine step. */
    if (rx_faults) handle_rx_fault();
    check_leases(HAL_GetTick());
    send_status(HAL_GetTick());
    start = 0;
    zero = 0;
    vision_follow_enable = 0U;
    return 1U;
}

#else
void WebControl_Init(void) {}
void WebControl_RxCompleteISR(void) {}
void WebControl_UartErrorISR(void) {}
uint8_t WebControl_Task(void) { return 0U; }
#endif
