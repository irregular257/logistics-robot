/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jy61p.h"
#include "string.h"
#include "motion.h"
#include "pid.h"
#include "mctrl.h"
#include "vision_follow.h"
#include "web_control.h"
#include "logistics_task.h"
#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t RxState;
uint8_t pRxPacket;
uint8_t Openmv_Column_RxPacket[4];
uint8_t Openmv_Annular_RxPacket[3];
uint8_t RxData = 0;
//uint8_t a1;
int32_t pulse_count = 0;
uint8_t receive_byte = 0;
//extern uint8_t task[];

uint8_t rx_data[8];
uint8_t RxState = 0;      // 当前状态
//uint8_t RxBuffer[8];      // 接收缓冲区
uint8_t aRxBuffer;      // 接收缓冲区
uint8_t RxIndex = 0;      // 缓冲区索引
uint8_t San_Data[8];
int ScanFlag = 0;
uint8_t a, b, c, d, e, f, g;
uint8_t task1[1] = {99};

extern tPid pid_omega;
extern float set_x;
extern float set_y;
extern double vx;
extern double vy;
extern double w;
extern float set_angle;
uint8_t g_usart_receivedata = 0;
#define RXBUFFERSIZE  256     //最大接收字节数
#define MAX_CATEGORY_SIZE 8

// 分类数组定义
uint8_t category_01_speed[MAX_CATEGORY_SIZE];
uint8_t category_01_distance[MAX_CATEGORY_SIZE];
uint8_t category_01_setdistance[MAX_CATEGORY_SIZE];
uint8_t category_01_state[MAX_CATEGORY_SIZE];
uint8_t category_01_len = 0;

uint8_t category_02_speed[MAX_CATEGORY_SIZE];
uint8_t category_02_distance[MAX_CATEGORY_SIZE];
uint8_t category_02_setdistance[MAX_CATEGORY_SIZE];
uint8_t category_02_state[MAX_CATEGORY_SIZE];
uint8_t category_02_len = 0;

uint8_t category_03_speed[MAX_CATEGORY_SIZE];
uint8_t category_03_distance[MAX_CATEGORY_SIZE];
uint8_t category_03_setdistance[MAX_CATEGORY_SIZE];
uint8_t category_03_state[MAX_CATEGORY_SIZE];
uint8_t category_03_len = 0;

uint8_t category_04_speed[MAX_CATEGORY_SIZE];
uint8_t category_04_distance[MAX_CATEGORY_SIZE];
uint8_t category_04_setdistance[MAX_CATEGORY_SIZE];
uint8_t category_04_state[MAX_CATEGORY_SIZE];
uint8_t category_04_len = 0;

unsigned char RxBufferww[RXBUFFERSIZE];   //接收数据
uint8_t aRxBuffer;		//接收中断缓冲
uint8_t Uart1_Rx_Cnt = 0;		//接收缓冲计数
uint8_t flag = 0;

dataall bj1 = {0, 0, 0, 0, 0};
dataall bj2 = {0, 0, 0, 0, 0};
dataall bj3 = {0, 0, 0, 0, 0};
dataall bj4 = {0, 0, 0, 0, 0};
extern int control_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void UART2_StartReceive(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }

    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
/* Provided by the FreeRTOS Cortex-M4F port. */

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */
    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */
    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
/* Provided by the FreeRTOS Cortex-M4F port. */

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */
    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
    /* USER CODE BEGIN SysTick_IRQn 1 */
    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
    /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
    /* USER CODE END CAN1_RX0_IRQn 0 */
    HAL_CAN_IRQHandler(&hcan1);
    /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
    /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
    /* USER CODE BEGIN CAN1_RX1_IRQn 0 */
    /* USER CODE END CAN1_RX1_IRQn 0 */
    HAL_CAN_IRQHandler(&hcan1);
    /* USER CODE BEGIN CAN1_RX1_IRQn 1 */
    /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
    /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
    HAL_TIM_IRQHandler(&htim1);
    /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
    /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
    /* USER CODE BEGIN USART1_IRQn 0 */
    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */
    /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
    /* USER CODE BEGIN USART2_IRQn 0 */
    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler(&huart2);
    /* USER CODE BEGIN USART2_IRQn 1 */
    /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */
    /* USER CODE END USART3_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
    /* USER CODE BEGIN USART3_IRQn 1 */
    /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
    /* USER CODE BEGIN UART4_IRQn 0 */
    /* USER CODE END UART4_IRQn 0 */
    HAL_UART_IRQHandler(&huart4);
    /* USER CODE BEGIN UART4_IRQn 1 */
    /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
    /* USER CODE BEGIN UART5_IRQn 0 */
    /* USER CODE END UART5_IRQn 0 */
    HAL_UART_IRQHandler(&huart5);
    /* USER CODE BEGIN UART5_IRQn 1 */
    /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
    /* USER CODE BEGIN USART6_IRQn 0 */
    /* USER CODE END USART6_IRQn 0 */
    HAL_UART_IRQHandler(&huart6);
    /* USER CODE BEGIN USART6_IRQn 1 */
    /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */





void HAL_UART_RxCpltCallback(UART_HandleTypeDef
                             *huart)//1,2,3为抓取任务判断命令帧头0x03，5为绿色色环校准命令帧头0x01，6,7,8返回色块圆心坐标帧头0xFF
{
    if (huart == &huart4) {
        WebControl_RxCompleteISR();
        return;
    }

    if (huart == &huart6) {
        HAL_UART_Receive_IT(&huart6, &RxData, 1); // 重新启动接收中断

        Openmv_ReceiveByte(RxData);
    }

    if (huart == &huart1) {
        Logistics_MotorRxByteISR(aRxBuffer);
        if (Uart1_Rx_Cnt >= 255) { //溢出判断
            Uart1_Rx_Cnt = 0;
            memset(RxBufferww, 0x00, sizeof(RxBufferww));
            /* Drop overflow; never transmit on the motor bus inside an ISR. */
        } else {
            RxBufferww[Uart1_Rx_Cnt++] = aRxBuffer;   //接收数据转存

            if (aRxBuffer == 0x6B) {
                uint16_t data_len = Uart1_Rx_Cnt - 1;

                if (data_len > 0) {
                    // 提取分类标识符（首字节）
                    uint8_t category = RxBufferww[0];
                    uint8_t second_byte = RxBufferww[1];

                    // 根据标识符分类存储
                    if (second_byte == 0x35) {
                        switch (category) {
                        case 0x01:
                            category_01_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_01_speed, RxBufferww, category_01_len);
                            break;

                        case 0x02:
                            category_02_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_02_speed, RxBufferww, category_02_len);
                            break;

                        case 0x03:
                            category_03_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_03_speed, RxBufferww, category_03_len);
                            break;

                        case 0x04:
                            category_04_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_04_speed, RxBufferww, category_04_len);
                            break;

                        // 添加其他分类标识符处理
                        // case 0xXX: ...

                        default:
                            // 未知标识符处理
                            break;
                        }
                    }

                    if (second_byte == 0x33) {
                        switch (category) {
                        case 0x01:
                            category_01_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_01_setdistance, RxBufferww, category_01_len);
                            break;

                        case 0x02:
                            category_02_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_02_setdistance, RxBufferww, category_02_len);
                            break;

                        case 0x03:
                            category_03_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_03_setdistance, RxBufferww, category_03_len);
                            break;

                        case 0x04:
                            category_04_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_04_setdistance, RxBufferww, category_04_len);
                            break;

                        default:
                            // 未知标识符处理
                            break;
                        }
                    }

                    if (second_byte == 0x36) {
                        switch (category) {
                        case 0x01:
                            category_01_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_01_distance, RxBufferww, category_01_len);
                            break;

                        case 0x02:
                            category_02_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_02_distance, RxBufferww, category_02_len);
                            break;

                        case 0x03:
                            category_03_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_03_distance, RxBufferww, category_03_len);
                            break;

                        case 0x04:
                            category_04_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_04_distance, RxBufferww, category_04_len);
                            break;

                        default:
                            // 未知标识符处理
                            break;
                        }
                    }

                    if (second_byte == 0x3A) {
                        switch (category) {
                        case 0x01:
                            category_01_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_01_state, RxBufferww, category_01_len);
                            break;

                        case 0x02:
                            category_02_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_02_state, RxBufferww, category_02_len);
                            break;

                        case 0x03:
                            category_03_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_03_state, RxBufferww, category_03_len);
                            break;

                        case 0x04:
                            category_04_len = (data_len > MAX_CATEGORY_SIZE) ? MAX_CATEGORY_SIZE : data_len;
                            memcpy(category_04_state, RxBufferww, category_04_len);
                            break;

                        default:
                            // 未知标识符处理
                            break;
                        }
                    }
                }

                Uart1_Rx_Cnt = 0;
                memset(RxBufferww, 0x00, sizeof(RxBufferww)); //清空数组
            }

            //data_cal();
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);   //再开启接收中断
    }

    if (huart == &huart5) {
        jy61p_ReceiveData(g_usart_receivedata);
        HAL_UART_Receive_IT(&huart5, &g_usart_receivedata, 1);
    }

    if (huart == &huart2) {
        HAL_UART_Receive_IT(&huart2, San_Data, 8);
        ScanFlag = 1;
        a = San_Data[0];
        b = San_Data[1];
        c = San_Data[2];
        d = San_Data[4];
        e = San_Data[5];
        f = San_Data[6];
        g = San_Data[7];
    }
}
// 发送单字节函数
void Openmv_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart6, &Byte, 1, 100);  // 阻塞发送，超时100ms
}

// 发送数组函数
void Openmv_SendArray(uint8_t *Array, int16_t Length)
{
    HAL_UART_Transmit(&huart6, Array, Length, 100);
}

// 错误处理回调（处理溢出等错误）
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4) {
        WebControl_UartErrorISR();
        return;
    }
    if (huart == &huart6) {
        // 处理溢出错误
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_OREFLAG(huart);  // 清除ORE标志 [[1]]
        }

        HAL_UART_Receive_IT(&huart6, &RxData, 1);  // 重启接收
    }

    if (RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) { //判断是否是空闲中断
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);                     //清楚空闲中断标志（否则会一直不断进入中断）
        HAL_UART_RxCpltCallback(&huart5);                          //调用中断处理函数
    }
}


void data_cal(void)
{
    bj1.dirsign = category_01_distance[2];
    bj1.speed = 256 * category_01_speed[3] + category_01_speed[4];
    bj1.distance = 16777216 * category_01_distance[3] + 65536 * category_01_distance[4] + 256 * category_01_distance[5] +
                   category_01_distance[6];
    bj1.setdistance = 16777216 * category_01_setdistance[3] + 65536 * category_01_setdistance[4] + 256 *
                      category_01_setdistance[5] + category_01_setdistance[6];
    bj1.state = category_01_state[2];
    bj2.dirsign = category_02_distance[2];
    bj2.speed = 256 * category_02_speed[3] + category_02_speed[4];
    bj2.distance = 16777216 * category_02_distance[3] + 65536 * category_02_distance[4] + 256 * category_02_distance[5] +
                   category_02_distance[6];
    bj2.setdistance = 16777216 * category_02_setdistance[3] + 65536 * category_02_setdistance[4] + 256 *
                      category_02_setdistance[5] + category_02_setdistance[6];
    bj2.state = category_02_state[2];
    bj3.dirsign = category_03_distance[2];
    bj3.speed = 256 * category_03_speed[3] + category_03_speed[4];
    bj3.distance = 16777216 * category_03_distance[3] + 65536 * category_03_distance[4] + 256 * category_03_distance[5] +
                   category_03_distance[6];
    bj3.setdistance = 16777216 * category_03_setdistance[3] + 65536 * category_03_setdistance[4] + 256 *
                      category_03_setdistance[5] + category_03_setdistance[6];
    bj3.state = category_03_state[2];
    bj4.dirsign = category_04_distance[2];
    bj4.speed = 256 * category_04_speed[3] + category_04_speed[4];
    bj4.distance = 16777216 * category_04_distance[3] + 65536 * category_04_distance[4] + 256 * category_04_distance[5] +
                   category_04_distance[6];
    bj4.setdistance = 16777216 * category_04_setdistance[3] + 65536 * category_04_setdistance[4] + 256 *
                      category_04_setdistance[5] + category_04_setdistance[6];
    bj4.state = category_04_state[2];
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1) {
        control_flag = 1;
        pid_omega.target_val = set_angle;
    }
}

/* USER CODE END 1 */
