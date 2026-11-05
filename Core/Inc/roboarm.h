#ifndef __ROBOARM_H__
#define __ROBOARM_H__

#ifdef __cplusplus


extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define PULSES_PER_ROTATION 3200
// 电机转动到指定位置所需的圈数
#define ROTATIONS_TO_TOP 3.7
// 电机转动到指定位置对应的高度，单位为米
#define HEIGHT_TO_TOP 0.165
// 声明每米对应的脉冲数，实际定义在源文件中
extern float pulses_per_meter;
/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */

typedef enum {
	S_VER   = 0,			/* 读取固件版本和对应的硬件版本 */
	S_RL    = 1,			/* 读取读取相电阻和相电感 */
	S_PID   = 2,			/* 读取PID参数 */
	S_VBUS  = 3,			/* 读取总线电压 */
	S_CPHA  = 5,			/* 读取相电流 */
	S_ENCL  = 7,			/* 读取经过线性化校准后的编码器值 */
	S_TPOS  = 8,			/* 读取电机目标位置角度 */
	S_VEL   = 9,			/* 读取电机实时转速 */
	S_CPOS  = 10,			/* 读取电机实时位置角度 */
	S_PERR  = 11,			/* 读取电机位置误差角度 */
	S_FLAG  = 13,			/* 读取使能/到位/堵转状态标志位 */
	S_Conf  = 14,			/* 读取驱动参数 */
	S_State = 15,			/* 读取系统状态参数 */
	S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
	S_MCH   = 17			/* 读脉冲 */
}SysParams_t;

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
// 函数声明
// 控制电机向上运动的函数
void motor_distanceUP(uint16_t speed, uint8_t accel, uint32_t pulse, uint8_t mode, uint8_t sync);
// 控制电机向下运动的函数
void motor_distanceDOWN(uint16_t speed, uint8_t accel, uint32_t pulse, uint8_t mode, uint8_t sync);
// 根据输入距离和方向控制电机运动的函数
void distance(float dist, uint8_t direction,uint16_t speed1,uint8_t accel);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s); // 读取参数


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ROBOARM_H__ */
