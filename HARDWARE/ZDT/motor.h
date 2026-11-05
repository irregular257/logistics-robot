#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct {
    uint8_t command_type; // 1: distance, 2: speed, 3: sync, 4: init, 5-8: read commands
    uint8_t id;
    uint8_t turn;
    uint16_t speed;
    uint8_t accel;
    uint32_t pulse;
    uint8_t mode;
    uint8_t sync;
} MotorCommand;

void process_motor_commands(void);

void motor_init(void);
void motor_distance(uint8_t id,uint8_t turn,uint16_t speed,uint8_t accel,uint32_t pulse,uint8_t mode,uint8_t sync);
void motor_speed(uint8_t id,uint8_t turn,uint16_t speed,uint8_t accel,uint8_t sync);
void motor_readdistance(uint8_t id);
void motor_read_setdistance(uint8_t id);
void motor_readspeed(uint8_t id);
void motor_readstate(uint8_t id);
void motor_sync(void);
void motor_zero(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
