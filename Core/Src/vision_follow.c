#include "vision_follow.h"
#include "stm32f4xx_it.h"
#include "motion.h"
#include "state.h"
#include "jy61p.h"
#include "usart.h"
#include <math.h>
#include <string.h>

#define TRACK_HEADER 0xF9U
#define TRACK_SIZE 14U
#define MATERIAL_HEADER 0xF6U
#define MATERIAL_SIZE 16U
#define MATERIAL_REQUEST_HEADER 0xF7U
#define RANGE_HEADER 0xF8U
#define RANGE_SIZE 9U
#define PARSER_GAP_MS 30U

extern uint8_t Openmv_Column_RxPacket[4];
extern uint8_t Openmv_Annular_RxPacket[3];
extern uint8_t task1[1];
extern int zero;

volatile uint8_t vision_follow_enable = 0;
volatile uint8_t vision_follow_color = 1;
volatile uint8_t vision_follow_state = 0;
volatile uint16_t vision_follow_x = 0;
volatile uint16_t vision_follow_y = 0;
volatile uint16_t vision_distance_mm = 0;
volatile uint8_t vision_distance_valid = 0;
volatile uint8_t vision_distance_reason = 6;

typedef struct {
    uint32_t tick;
    uint16_t x, y, width, height;
    uint8_t valid, color, shape, sequence, seen;
    uint16_t distance_mm;
    uint8_t distance_valid, distance_reason, distance_seen;
} TrackingSample;

static volatile TrackingSample latest;
static uint8_t rx_buffer[MATERIAL_SIZE];
static uint8_t rx_count;
static uint32_t rx_tick;
static volatile uint8_t active, active_color;
static uint8_t exit_wait, moving;
static uint32_t last_control_tick;
static float heading;
static uint8_t observe_only;
static volatile uint8_t material_mode, active_shape, material_token;

static uint8_t crc8(const uint8_t *data, uint8_t size)
{
    uint8_t crc = 0, i, bit;
    for (i = 0; i < size; ++i) {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit) {
            crc = (uint8_t)((crc & 0x80U) ? (crc << 1) ^ 0x07U : crc << 1);
        }
    }
    return crc;
}

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint8_t frame_size(uint8_t head)
{
    switch (head) {
    case 0xFF: case 0x01: return 4;
    case 0x03: case 0x04: return 3;
    case 0xFA: return 6;
    case TRACK_HEADER: return TRACK_SIZE;
    case MATERIAL_HEADER: return MATERIAL_SIZE;
    case RANGE_HEADER: return RANGE_SIZE;
    default: return 0;
    }
}

static void publish_frame(uint8_t size, uint32_t now)
{
    uint16_t x, y, width, height;
    switch (rx_buffer[0]) {
    case 0xFF:
        memcpy(Openmv_Column_RxPacket, rx_buffer + 1, 2);
        break;
    case 0x01:
        memcpy(Openmv_Annular_RxPacket, rx_buffer + 1, 2);
        break;
    case 0x03:
        if (rx_buffer[1] >= 1 && rx_buffer[1] <= 3) task1[0] = 1;
        break;
    case 0x04:
        Openmv_Column_RxPacket[0] = rx_buffer[1];
        break;
    case 0xFA:
        memcpy(Openmv_Column_RxPacket, rx_buffer + 1, 4);
        break;
    case TRACK_HEADER:
        if (!active || material_mode || (latest.seen && latest.sequence == rx_buffer[1])) break;
        x = read_u16(rx_buffer + 4);
        y = read_u16(rx_buffer + 6);
        width = read_u16(rx_buffer + 8);
        height = read_u16(rx_buffer + 10);
        if (rx_buffer[2] > 1) break;
        if (rx_buffer[2] && (x >= 320 || y >= 240 || !width || width > 320 ||
            !height || height > 240 || rx_buffer[3] < 1 || rx_buffer[3] > 3 ||
            (active_color != 4 && rx_buffer[3] != active_color))) break;
        latest.valid = rx_buffer[2];
        latest.color = rx_buffer[3];
        latest.shape = 0U;
        latest.x = x; latest.y = y;
        latest.width = width; latest.height = height;
        latest.sequence = rx_buffer[1];
        latest.tick = now;
        latest.seen = 1;
        latest.distance_mm = 0;
        latest.distance_valid = latest.distance_seen = 0;
        latest.distance_reason = 6;
        break;
    case MATERIAL_HEADER:
        if (!active || !observe_only || !material_mode || rx_buffer[5] != material_token ||
            (latest.seen && latest.sequence == rx_buffer[1])) break;
        x = read_u16(rx_buffer + 6); y = read_u16(rx_buffer + 8);
        width = read_u16(rx_buffer + 10); height = read_u16(rx_buffer + 12);
        if (rx_buffer[2] > 1U) break;
        if (rx_buffer[2] && (rx_buffer[3] != active_color || rx_buffer[4] != active_shape ||
            x >= 320U || y >= 240U || !width || width > 320U || !height || height > 240U)) break;
        if (!rx_buffer[2] && (rx_buffer[3] || rx_buffer[4] || x || y || width || height)) break;
        latest.valid = rx_buffer[2]; latest.color = rx_buffer[3]; latest.shape = rx_buffer[4];
        latest.x = x; latest.y = y; latest.width = width; latest.height = height;
        latest.sequence = rx_buffer[1]; latest.tick = now; latest.seen = 1U;
        latest.distance_mm = 0; latest.distance_valid = latest.distance_seen = 0;
        latest.distance_reason = 6;
        break;
    case RANGE_HEADER:
        /* A range frame belongs to the immediately preceding tracking sample.
         * It cannot refresh that sample's watchdog or repeat its range update.
         */
        if (!active || material_mode || !latest.seen || latest.distance_seen ||
            latest.sequence != rx_buffer[1] ||
            (uint32_t)(now - latest.tick) > VISION_FOLLOW_TIMEOUT_MS) break;
        if (rx_buffer[2] > 1 || rx_buffer[6] > 5 ||
            rx_buffer[3] != (latest.valid ? latest.color : 0)) break;
        width = read_u16(rx_buffer + 4); /* Distance payload, in mm. */
        if (rx_buffer[2]) {
            if (!latest.valid || !width || rx_buffer[6] != 0) break;
        } else if (width || rx_buffer[6] == 0) break;
        latest.distance_valid = rx_buffer[2];
        latest.distance_mm = width;
        latest.distance_reason = rx_buffer[6];
        latest.distance_seen = 1;
        break;
    default: break;
    }
    (void)size;
}

void Openmv_ReceiveByte(uint8_t byte)
{
    uint32_t now = HAL_GetTick();
    uint8_t size;
    if (rx_count && (uint32_t)(now - rx_tick) > PARSER_GAP_MS) rx_count = 0;
    rx_tick = now;
    if (rx_count == sizeof(rx_buffer)) rx_count = 0;
    rx_buffer[rx_count++] = byte;
    while (rx_count) {
        size = frame_size(rx_buffer[0]);
        if (size && rx_count < size) break;
        if (size && rx_buffer[size - 1] == 0xFE &&
            ((rx_buffer[0] != TRACK_HEADER && rx_buffer[0] != RANGE_HEADER && rx_buffer[0] != MATERIAL_HEADER) ||
             crc8(rx_buffer, (uint8_t)(size - 2)) == rx_buffer[size - 2])) {
            publish_frame(size, now);
            rx_count = (uint8_t)(rx_count - size);
            memmove(rx_buffer, rx_buffer + size, rx_count);
        } else {
            --rx_count;
            memmove(rx_buffer, rx_buffer + 1, rx_count);
        }
    }
}

static TrackingSample sample_snapshot(void)
{
    TrackingSample sample;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    sample = latest;
    __set_PRIMASK(irq);
    return sample;
}

static void reset_sample(void)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    memset((void *)&latest, 0, sizeof(latest));
    rx_count = 0;
    __set_PRIMASK(irq);
    vision_distance_valid = 0;
    vision_distance_mm = 0;
    vision_distance_reason = 6;
}

uint8_t VisionObserve_Start(uint8_t color)
{
    uint8_t command[2]={'a',(uint8_t)(30U+color)};
    if (color<1U || color>4U) return 0U;
    material_mode = active_shape = 0U;
    vision_follow_enable=0U; moving=0U; exit_wait=0U;
    observe_only=1U; active=1U; active_color=color;
    reset_sample();
    return HAL_UART_Transmit(&huart6,command,2U,10U)==HAL_OK;
}
uint8_t VisionObserve_StartTarget(uint8_t color, uint8_t shape)
{
    uint8_t command[6] = {MATERIAL_REQUEST_HEADER, 0, color, shape, 0, 0xFEU};
    if (!(color == 1U || color == 2U || color == 3U || color == 4U || color == 7U) ||
        !(shape == 1U || shape == 3U || shape == 4U || shape == 5U || shape == 6U)) return 0U;
    if ((shape == 1U && color != 4U) || (shape == 3U && color == 7U) ||
        (shape == 4U && color != 1U && color != 2U && color != 7U)) return 0U;
    vision_follow_enable = 0U; moving = 0U; exit_wait = 0U;
    observe_only = material_mode = active = 1U;
    active_color = color; active_shape = shape;
    ++material_token; command[1] = material_token;
    command[4] = crc8(command, 4U);
    reset_sample();
    if (HAL_UART_Transmit(&huart6,command,sizeof(command),10U) == HAL_OK) return 1U;
    observe_only = material_mode = active = active_color = active_shape = 0U;
    return 0U;
}
uint8_t VisionObserve_Stop(void)
{
    uint8_t command=30U;
    if (!observe_only) return 1U;
    observe_only=0U; active=active_color=material_mode=active_shape=0U;
    reset_sample();
    return HAL_UART_Transmit(&huart6,&command,1U,10U)==HAL_OK;
}
uint8_t VisionObserve_Read(VisionObservation *out, uint32_t max_age_ms)
{
    TrackingSample sample=sample_snapshot();
    if (!observe_only || !sample.seen || !sample.valid ||
        (uint32_t)(HAL_GetTick()-sample.tick)>max_age_ms) return 0U;
    out->x=sample.x; out->y=sample.y; out->color=sample.color; out->shape=sample.shape; out->tick=sample.tick;
    return 1U;
}

static void update_distance_watch(TrackingSample sample, uint32_t now)
{
    uint8_t fresh = sample.seen &&
        (uint32_t)(now - sample.tick) <= VISION_FOLLOW_TIMEOUT_MS;
    vision_distance_valid = active && fresh && sample.valid && sample.distance_valid;
    vision_distance_mm = vision_distance_valid ? sample.distance_mm : 0;
    vision_distance_reason = !active || !sample.seen ? 6 : !fresh ? 7 :
        sample.distance_seen ? sample.distance_reason : 6;
}

static float clamp(float value, float magnitude)
{
    if (value > magnitude) return magnitude;
    if (value < -magnitude) return -magnitude;
    return value;
}

static void stop_chassis(void)
{
    if (moving) {
        go(0, 0, 0);
        moving = 0;
    }
}

uint8_t VisionFollow_Task(void)
{
    TrackingSample sample;
    uint32_t now = HAL_GetTick();
    uint8_t start_pressed = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET;
    float ex, ey, vx_cmd, vy_cmd, speed, angle, error, omega;

    if (observe_only) return 1U;

    update_distance_watch(sample_snapshot(), now);

    if (active && (!vision_follow_enable || zero)) {
        vision_follow_enable = 0;
        go(0, 0, 0);
        moving = 0;
        Openmv_SendByte(30);
        active = 0; exit_wait = 1;
        reset_sample();
        vision_follow_state = 0;
        return 1;
    }
    if (exit_wait) {
        if (!start_pressed) exit_wait = 0;
        return 1; /* Require release before the original route can start. */
    }
    if (!vision_follow_enable) return 0;
    if (!active) {
        if (!Motor_TaskIsIdle()) {
            vision_follow_enable = 0;
            vision_follow_state = 4;
            return 0;
        }
        active = 1;
        active_color = 0;
        heading = Yaw;
        go(0, 0, 0);
        moving = 0;
    }
    if (vision_follow_color < 1 || vision_follow_color > 4) {
        stop_chassis();
        active_color = 0;
        reset_sample();
        vision_follow_state = 1;
        return 1;
    }
    if (active_color != vision_follow_color) {
        stop_chassis();
        active_color = vision_follow_color;
        reset_sample();
        Openmv_SendByte('a');
        Openmv_SendByte((uint8_t)(30 + active_color));
        last_control_tick = HAL_GetTick();
    }
    sample = sample_snapshot();
    vision_follow_x = sample.x;
    vision_follow_y = sample.y;
    now = HAL_GetTick();
    if (!start_pressed || !sample.seen || !sample.valid ||
        (uint32_t)(now - sample.tick) > VISION_FOLLOW_TIMEOUT_MS) {
        stop_chassis();
        vision_follow_state = start_pressed ? 3 : 1;
        if (!start_pressed) heading = Yaw;
        return 1;
    }
    if ((uint32_t)(now - last_control_tick) < VISION_FOLLOW_PERIOD_MS) return 1;
    last_control_tick = now;
    ex = VISION_FOLLOW_TARGET_X - sample.x;
    ey = VISION_FOLLOW_TARGET_Y - sample.y;
    if (fabsf(ex) <= VISION_FOLLOW_DEADBAND_PX) ex = 0;
    if (fabsf(ey) <= VISION_FOLLOW_DEADBAND_PX) ey = 0;
    vx_cmd = VISION_FOLLOW_GAIN_RPS_PER_PX *
        (VISION_FOLLOW_EX_TO_VX * ex + VISION_FOLLOW_EY_TO_VX * ey);
    vy_cmd = VISION_FOLLOW_GAIN_RPS_PER_PX *
        (VISION_FOLLOW_EX_TO_VY * ex + VISION_FOLLOW_EY_TO_VY * ey);
    speed = sqrtf(vx_cmd * vx_cmd + vy_cmd * vy_cmd);
    angle = speed > 0 ? atan2f(vy_cmd, vx_cmd) * 57.2957795f : 0;
    speed = clamp(speed, VISION_FOLLOW_MAX_RPS);
    error = heading - Yaw;
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    omega = fabsf(error) <= 0.3f ? 0 : clamp(0.2f * error, VISION_FOLLOW_MAX_OMEGA);
    go(speed, angle, omega);
    moving = speed > 0 || omega != 0;
    vision_follow_state = 2;
    return 1;
}
