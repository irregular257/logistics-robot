#include "roboarm.h"
#include "motor.h"
#include "usart.h"

// 200*3.7/0.165
float pulses_per_meter = PULSES_PER_ROTATION * ROTATIONS_TO_TOP / HEIGHT_TO_TOP;

unsigned char DOWN[13] = {0x05, 0xFD, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x0C, 0x80, 0x00, 0x01, 0x6B};
unsigned char UP[13] = {0x05, 0xFD, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x0C, 0x80, 0x00, 0x01, 0x6B};

void motor_distanceUP(uint16_t speed, uint8_t accel, uint32_t pulse, uint8_t mode, uint8_t sync) {
    UP[3] = speed >> 8;
    UP[4] = speed;
    UP[5] = accel;
    UP[6] = pulse >> 24;
    UP[7] = pulse >> 16;
    UP[8] = pulse >> 8;
    UP[9] = pulse;
    UP[10] = mode;
    UP[11] = sync;

    HAL_UART_Transmit(&huart1, UP, 13, 1000);
	  HAL_Delay(10);
}

void motor_distanceDOWN(uint16_t speed, uint8_t accel, uint32_t pulse, uint8_t mode, uint8_t sync) {
    DOWN[3] = speed >> 8;
    DOWN[4] = speed;
    DOWN[5] = accel;
    DOWN[6] = pulse >> 24;
    DOWN[7] = pulse >> 16;
    DOWN[8] = pulse >> 8;
    DOWN[9] = pulse;
    DOWN[10] = mode;
    DOWN[11] = sync;

    HAL_UART_Transmit(&huart1, DOWN, 13, 1000);
	  HAL_Delay(10);
}



/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
		case S_MCH  : cmd[i] = 0x32; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(&huart1, (uint8_t *)cmd,i,1000);
}


void distance(float dist, uint8_t direction,uint16_t speed1,uint8_t accel) {
    // 计算 speed/60 的结果
    uint16_t speed = speed1*60;

    // 默认参数

    uint8_t mode = 0;
    uint8_t sync = 0;


    uint32_t pulse = (uint32_t)(dist * pulses_per_meter);
	

    if (direction == 1) {  // 向上运动
        if(pulse > 0) {
            motor_distanceUP(speed, accel, pulse, mode, sync);
        }
    } else if (direction == 0) {  // 向下运动
        if(pulse > 0) {
            motor_distanceDOWN(speed, accel, pulse, mode, sync);
        }
    }
}    
