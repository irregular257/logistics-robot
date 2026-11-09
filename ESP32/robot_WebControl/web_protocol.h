#ifndef robot_WEB_PROTOCOL_H
#define robot_WEB_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>

/* ESP32 <-> STM32 UART4, 115200 8N1. All multi-byte fields little-endian.
 * A5 5A version len session[4] sequence[2] command payload[len] CRC16[2]
 * CRC16/CCITT-FALSE covers version..payload. Total length = 13 + len.
 */
#define WEB_VERSION 1U
#define WEB_MAX_PAYLOAD 32U
#define WEB_MAX_FRAME (13U + WEB_MAX_PAYLOAD)
#define WEB_CMD_HELLO 0x01U
#define WEB_CMD_HEARTBEAT 0x02U
#define WEB_CMD_STOP 0x03U
#define WEB_CMD_MODE 0x04U
#define WEB_CMD_JOG 0x05U
#define WEB_CMD_CLAW 0x06U
#define WEB_CMD_TASK 0x07U
#define WEB_CMD_START 0x08U
#define WEB_CMD_PAUSE 0x09U
#define WEB_CMD_GIMBAL 0x0AU
#define WEB_CMD_CLAW_ANGLE 0x0BU
#define WEB_CMD_SERVO_PRESETS 0x0CU
#define WEB_CMD_TRAY_ANGLE 0x0DU
#define WEB_CMD_MANUAL_RELEASE 0x0EU
#define WEB_CMD_TABLE_STATUS 0x0FU
#define WEB_CMD_TABLE_REQUEST 0x82U
#define WEB_TABLE_REQUEST_SIZE 9U
#define WEB_TABLE_STATUS_SIZE 15U
#define WEB_TABLE_PERIOD_MS 50U
#define WEB_TABLE_OP_START 1U
#define WEB_TABLE_OP_HOLD 2U
#define WEB_TABLE_OP_PICK_DONE 3U
#define WEB_TABLE_OP_ABORT 4U
#define WEB_TABLE_IDLE 0U
#define WEB_TABLE_MOVING 1U
#define WEB_TABLE_SETTLING 2U
#define WEB_TABLE_READY 3U
#define WEB_TABLE_HELD 4U
#define WEB_TABLE_DONE 5U
#define WEB_TABLE_FAULT 6U
/* TABLE_REQUEST STM32->ESP32: [op,taskId LE32,window LE32]. Repeats every
 * 50 ms; identical op/task/window requests MUST be idempotent at the bridge.
 * TABLE_STATUS ESP32->STM32: [state,picked,fault,taskId LE32,window LE32,
 * tableBoot LE32]. 50 ms fresh telemetry; nonzero fault on wireless loss.
 * UART headers/session/sequence/CRC are unchanged. STATUS needs no UART ACK.
 * HELD confirms completed stop at that exact task/window. Grasp never starts
 * from a send-success callback. PICK_DONE follows arm clearance, once/item. */
#define WEB_GIMBAL_MIN_ANGLE 0U
#define WEB_GIMBAL_MAX_ANGLE 270U
#define WEB_CLAW_MIN_ANGLE 0U
#define WEB_CLAW_MAX_ANGLE 270U
#define WEB_SERVO_STATUS_SIZE 28U
#define WEB_TRAY_MIN_ANGLE 0U
#define WEB_TRAY_MAX_ANGLE 270U
#define WEB_TRAY_STATUS_SIZE 30U
#define WEB_TASK_STATUS_SIZE 32U
#define WEB_TASK_PROTOCOL_VERSION 1U
#define WEB_TASK_PLAN_SIZE 6U
#define WEB_TRAY_UNKNOWN_ANGLE 0xFFFFU
#define WEB_GIMBAL_UNKNOWN_ANGLE 0xFFFFU
#define WEB_CMD_STATUS 0x80U
#define WEB_CMD_ACK 0x81U
#define WEB_RESULT_OK 0U
#define WEB_RESULT_BAD_ARGUMENT 1U
#define WEB_RESULT_WRONG_MODE 2U
#define WEB_RESULT_UNSUPPORTED 3U
#define WEB_RESULT_STALE 4U
#define WEB_RESULT_INTERLOCK 5U
#define WEB_MODE_IDLE 0U
#define WEB_MODE_MANUAL 1U
#define WEB_MODE_AUTO 2U
/* JOG payload: signed int8 [x(right+), y(forward+), turn(CCW+), lift(up+)].
 * x/y/turn range -50..50 percent; lift -1/0/+1. Chassis and lift exclusive.
 * CLAW payload 0=close, 1=open. MODE payload 0=idle, 1=manual.
 * GIMBAL/CLAW_ANGLE/TRAY_ANGLE payload: LE16 absolute target, 0..270 degrees.
 * SERVO_PRESETS payload: four LE16 values [claw close, claw open,
 * gimbal retract, gimbal extend]. Stores calibration only, no servo motion.
 * ACK payload [request command, result, current mode, fault]; header seq echoes request.
 * STATUS payload [mode,fault,claw(0/1/2 unknown),lift enabled(0/1),x,y,turn,lift].
 * TASK: six bytes [color1,shape1,color2,shape2,color3,shape3], in pickup order.
 * Colors: 1 red, 2 green, 3 blue, 4 black, 7 yellow.
 * Shapes: 1 hexagonal prism, 3 cylinder, 4 pentagonal prism, 5 cuboid, 6 b奶娃. No wildcard.
 * TASK only stores a validated plan; it never moves an actuator.
 * START: payload [1, six plan bytes] confirms car at pickup, front toward +Y,
 * lift at transport height, and must match the plan acknowledged by TASK.
 * PAUSE: no payload; cancels automatic motion (no mid-route resume).
 * STATUS extended to 16 bytes: original 8, task state, drop index(0..2),
 * task error, automatic supported(1), X cm signed LE16, Y cm signed LE16.
 * STATUS extended to 18 bytes: original 16 plus GIMBAL target LE16.
 * 0xFFFF means unknown, including after automatic control takes ownership.
 * STATUS extended to 28 bytes: original 18, claw target LE16,
 * claw close/open and gimbal retract/extend presets (four LE16 values).
 * 0xFFFF claw target means unknown. Targets are commands, not measured angles.
 * 18 bytes advertises legacy GIMBAL support (54..229); 28 bytes additionally
 * advertises 0..270 degree GIMBAL/CLAW_ANGLE and SERVO_PRESETS support.
 * STATUS extended to 30 bytes: original 28 plus TRAY_ANGLE target LE16.
 * 30 bytes additionally advertises TRAY_ANGLE support; 0xFFFF means unknown.
 * STATUS extended to 32 bytes: original 30, task protocol version(1), plan ready(0/1).
 * The first/second/third loaded item goes to UL/UR/LR respectively.
 * MANUAL_RELEASE: no payload; releases manual control only. Automatic execution
 * and its plan persist across web sessions, heartbeats and reconnects.
 * STOP/PAUSE explicitly cancel automatic execution. START requires all plan bytes.
 */
typedef struct {
    uint32_t session;
    uint16_t sequence;
    uint8_t command;
    uint8_t length;
    uint8_t payload[WEB_MAX_PAYLOAD];
} WebFrame;

static inline uint16_t Web_Get16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t Web_Get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline void Web_Put16(uint8_t *p, uint16_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v >> 8);
}
static inline void Web_Put32(uint8_t *p, uint32_t v) {
    p[0]=(uint8_t)v; p[1]=(uint8_t)(v >> 8);
    p[2]=(uint8_t)(v >> 16); p[3]=(uint8_t)(v >> 24);
}
static inline uint16_t Web_Crc16(const uint8_t *data, uint8_t size) {
    uint16_t crc=0xFFFFU;
    uint8_t i,b;
    for (i=0; i<size; ++i) {
        crc^=(uint16_t)((uint16_t)data[i] << 8);
        for (b=0; b<8; ++b)
            crc=(uint16_t)((crc & 0x8000U) ? ((crc << 1)^0x1021U) : (crc << 1));
    }
    return crc;
}
static inline uint8_t Web_Encode(const WebFrame *f, uint8_t *out) {
    uint8_t i,n;
    if (f->length>WEB_MAX_PAYLOAD) return 0;
    out[0]=0xA5; out[1]=0x5A; out[2]=WEB_VERSION; out[3]=f->length;
    Web_Put32(out+4,f->session); Web_Put16(out+8,f->sequence); out[10]=f->command;
    for(i=0;i<f->length;++i) out[11+i]=f->payload[i];
    n=(uint8_t)(11+f->length);
    Web_Put16(out+n,Web_Crc16(out+2,(uint8_t)(n-2)));
    return (uint8_t)(n+2);
}
#endif
