#include "motor.h"
#include "usart.h"
#include "main.h"
unsigned char RE[6] = {0x01, 0xF3, 0xAB, 0x01, 0x00, 0x6B};
unsigned char SE[13] = {0x01, 0xFD, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x0C, 0x80, 0x00, 0x01, 0x6B};
unsigned char SE1[8] = {0x01, 0xF6, 0x01, 0x00, 0x3C, 0x00, 0x01, 0x6B};
unsigned char TB[4] = {0x00, 0xFF, 0x66, 0x6B};
unsigned char READ[3] = {0x00, 0x36, 0x6B};
unsigned char READ_set[3] = {0x00, 0x33, 0x6B};
unsigned char READ_speed[3] = {0x00, 0x35, 0x6B};
unsigned char READ_state[3] = {0x00, 0x3A, 0x6B};
unsigned char TD[4] = {0x00, 0x0A, 0x6D, 0x6B};
void motor_init()
{
    RE[0] = 0x01;
    HAL_UART_Transmit(&huart1, RE, 6, 1000);
    HAL_Delay(1);
    RE[0] = 0x02;
    HAL_UART_Transmit(&huart1, RE, 6, 1000);
    HAL_Delay(1);
    RE[0] = 0x03;
    HAL_UART_Transmit(&huart1, RE, 6, 1000);
    HAL_Delay(1);
    RE[0] = 0x04;
    HAL_UART_Transmit(&huart1, RE, 6, 1000);
    HAL_Delay(1);
    RE[0] = 0x05;
    HAL_UART_Transmit(&huart1, RE, 6, 1000);
    HAL_Delay(10);
}
void motor_distance(uint8_t id, uint8_t turn, uint16_t speed, uint8_t accel, uint32_t pulse, uint8_t mode, uint8_t sync)
{
    SE[0] = id;
    SE[2] = turn;
    SE[3] = speed >> 8;
    SE[4] = speed;
    SE[5] = accel;
    SE[6] = pulse >> 24;
    SE[7] = pulse >> 16;
    SE[8] = pulse >> 8;
    SE[9] = pulse;
    SE[10] = mode;
    SE[11] = sync;
    HAL_UART_Transmit(&huart1, SE, 13, 1000);
    HAL_Delay(10);
    /*
     HAL_UART_Transmit(&huart2,SE,13,1000);
     HAL_Delay(10);
     HAL_UART_Transmit(&huart2,SE,13,1000);
     HAL_Delay(10);
     HAL_UART_Transmit(&huart2,SE,13,1000);
     HAL_Delay(10);
     HAL_UART_Transmit(&huart2,TB,4,1000);
     HAL_Delay(10);
     */
    /*
      HAL_UART_Transmit(&huart2,SE,13,1000);
    	HAL_Delay(10);
    	HAL_UART_Transmit(&huart2,SE1,13,1000);
    	HAL_Delay(10);
    	HAL_UART_Transmit(&huart2,TB,4,1000);
    	HAL_Delay(10);
    */
}
void motor_speed(uint8_t id, uint8_t turn, uint16_t speed, uint8_t accel, uint8_t sync)
{
    SE1[0] = id;
    SE1[2] = turn;
    SE1[3] = speed >> 8;
    SE1[4] = speed;
    SE1[5] = accel;
    SE1[6] = sync;
    HAL_UART_Transmit(&huart1, SE1, 8, 1000);
    //NonBlockingDelay(5);
    HAL_Delay(10);
}
void motor_readdistance(uint8_t id)
{
    //使用此方法读出的单位为圈数
    //读取的为实时位置值
    READ[0] = id;
    HAL_UART_Transmit(&huart1, READ, 3, 1000);
    HAL_Delay(1);
}
void motor_read_setdistance(uint8_t id)
{
    //读取的为设定位置值
    READ_set[0] = id;
    HAL_UART_Transmit(&huart1, READ_set, 3, 1000);
    HAL_Delay(1);
}
void motor_readspeed(uint8_t id)
{
    //使用此方法读出的单位为圈数
    //读取的为实时位置值
    READ_speed[0] = id;
    HAL_UART_Transmit(&huart1, READ_speed, 3, 1000);
    HAL_Delay(1);
}
void motor_readstate(uint8_t id)
{
    //读取的第三位为电机状态校验位
    READ_state[0] = id;
    HAL_UART_Transmit(&huart1, READ_state, 3, 1000);
    HAL_Delay(1);
}
void motor_sync()
{
    HAL_UART_Transmit(&huart1, TB, 4, 1000);
    HAL_Delay(10);
}

void motor_zero()
{
    HAL_UART_Transmit(&huart1, TD, 4, 1000);
    HAL_Delay(1);
}



