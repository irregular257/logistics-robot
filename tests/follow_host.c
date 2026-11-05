/* Hardware stubs only; tests below execute the actual firmware implementation. */
#define __MAIN_H
#include <stdint.h>
#include <string.h>
typedef int HAL_StatusTypeDef;
typedef struct { int port; } UART_HandleTypeDef;
UART_HandleTypeDef huart6={6};
#define HAL_OK 0
#define GPIOE ((void *)1)
#define GPIO_PIN_1 2U
#define GPIO_PIN_RESET 0
static uint32_t mock_tick;
static int mock_pressed, mock_idle;
static float output_speed, output_angle, output_omega;
static unsigned motor_calls, command_count;
static uint8_t commands[64];
uint8_t Openmv_Column_RxPacket[4];
uint8_t Openmv_Annular_RxPacket[3];
uint8_t task1[1];
int zero;
float Yaw, Pitch, Roll;
uint32_t HAL_GetTick(void) { return mock_tick; }
int HAL_GPIO_ReadPin(void *port, uint32_t pin) { (void)port; (void)pin; return mock_pressed ? 0 : 1; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
void go(float speed, float angle, float omega) {
    output_speed = speed; output_angle = angle; output_omega = omega; ++motor_calls;
}
void Openmv_SendByte(uint8_t byte) { if (command_count < sizeof(commands)) commands[command_count++] = byte; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *port,uint8_t *data,uint16_t size,uint32_t timeout) {
    uint16_t i; (void)port; (void)timeout;
    for(i=0;i<size;++i) Openmv_SendByte(data[i]);
    return HAL_OK;
}
uint8_t Motor_TaskIsIdle(void) { return (uint8_t)mock_idle; }
#include "../Core/Src/vision_follow.c"

#define EXPORT __declspec(dllexport)
EXPORT void host_reset(void) {
    memset((void *)&latest, 0, sizeof(latest));
    memset(rx_buffer, 0, sizeof(rx_buffer)); rx_count = 0; rx_tick = 0;
    active = active_color = exit_wait = moving = observe_only = 0;
    material_mode = active_shape = material_token = 0; heading = 0; last_control_tick = 0;
    vision_follow_enable = 0; vision_follow_color = 1; vision_follow_state = 0;
    vision_distance_mm = 0; vision_distance_valid = 0; vision_distance_reason = 6;
    mock_tick = 0; mock_pressed = 0; mock_idle = 1; Yaw = 0; zero = 0;
    output_speed = output_angle = output_omega = 0; motor_calls = command_count = 0;
    memset(Openmv_Column_RxPacket, 0, sizeof(Openmv_Column_RxPacket));
    memset(Openmv_Annular_RxPacket, 0, sizeof(Openmv_Annular_RxPacket)); task1[0] = 0;
}
EXPORT void host_mode(int enable, int color) { vision_follow_enable = (uint8_t)enable; vision_follow_color = (uint8_t)color; }
EXPORT void host_key(int pressed) { mock_pressed = pressed; }
EXPORT void host_idle(int idle) { mock_idle = idle; }
EXPORT void host_time(uint32_t value) { mock_tick = value; }
EXPORT void host_yaw(float value) { Yaw = value; }
EXPORT void host_byte(uint8_t value) { Openmv_ReceiveByte(value); }
EXPORT int host_step(void) { return VisionFollow_Task(); }
EXPORT int host_state(void) { return vision_follow_state; }
EXPORT int host_enabled(void) { return vision_follow_enable; }
EXPORT int host_seen(void) { return latest.seen; }
EXPORT int host_x(void) { return latest.x; }
EXPORT int host_y(void) { return latest.y; }
EXPORT int host_distance(void) { return vision_distance_mm; }
EXPORT int host_distance_valid(void) { return vision_distance_valid; }
EXPORT int host_distance_reason(void) { return vision_distance_reason; }
EXPORT int host_column(int i) { return Openmv_Column_RxPacket[i]; }
EXPORT int host_ring(int i) { return Openmv_Annular_RxPacket[i]; }
EXPORT int host_confirmed(void) { return task1[0]; }
EXPORT int host_command(int i) { return i < (int)command_count ? commands[i] : -1; }
EXPORT int host_calls(void) { return (int)motor_calls; }
EXPORT float host_speed(void) { return output_speed; }
EXPORT float host_angle(void) { return output_angle; }
EXPORT float host_omega(void) { return output_omega; }
EXPORT int host_observe_start(int color) { return VisionObserve_Start((uint8_t)color); }
EXPORT int host_target_start(int color, int shape) { return VisionObserve_StartTarget((uint8_t)color,(uint8_t)shape); }
EXPORT int host_target_color(void) { VisionObservation sample; return VisionObserve_Read(&sample,250U) ? sample.color : 0; }
EXPORT int host_target_shape(void) { VisionObservation sample; return VisionObserve_Read(&sample,250U) ? sample.shape : 0; }
EXPORT int host_observe_stop(void) { return VisionObserve_Stop(); }
EXPORT int host_observe_read(void) { VisionObservation sample; return VisionObserve_Read(&sample,250U); }
