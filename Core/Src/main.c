/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "mctrl.h"
#include "stm32f4xx_it.h"
#include "roboarm.h"
#include "control.h"
#include "motion.h"
#include "function.h"
#include "duiji.h"
#include "jy61p.h"
#include "state.h"
#include "pid.h"
#include "tjc_usart_hmi.h"
#include "dm_imu.h"
#include "vision_follow.h"
#include "web_control.h"
#include "logistics_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t RxData;
extern uint8_t Openmv_Column_RxPacket[2]; //视觉色块数据接收数组
extern uint8_t Openmv_Annular_RxPacket[2]; ////视觉色环数据接收数组
extern uint8_t g_usart_receivedata;
extern uint8_t aRxBuffer;
extern uint8_t San_Data[8]; //扫码数据接收数组
char str[100];
extern int zero, start; //清零，状态机运行
extern uint16_t task1_flag;
extern float set_angle;
int D1, D2, D3, D4;
extern int16_t tasklimit;




//uint8_t tempBuffer=0,RxBuffer1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */
    /* MCU Configuration--------------------------------------------------------*/
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    /* USER CODE BEGIN Init */
    /* USER CODE END Init */
    /* Configure the system clock */
    SystemClock_Config();
    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_UART5_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_UART4_Init();
    MX_CAN1_Init();
    /* USER CODE BEGIN 2 */
    motor_init();
    WebControl_Init();
#if !WEB_CONTROL_ENABLED
    Logistics_Init();
#endif
#if !WEB_CONTROL_ENABLED
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
#endif
    HAL_UART_Receive_IT(&huart6, &RxData, 1);
    HAL_UART_Receive_IT(&huart5, &g_usart_receivedata, 1);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer, 1);
    HAL_UART_Receive_IT(&huart2, San_Data, 8);
    //UART2_StartReceive();
    HAL_TIM_Base_Start_IT(&htim1);
    Serial_JY61P_Zero_Yaw(); //忽略函数名，实际使用HWT101
    PID_init();
    initRingBuffer();		//串口屏初始化
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
#if !WEB_CONTROL_ENABLED
    HAL_Delay(1500);
    //上电机械臂归位，注意卡手
    YunTai(IN);
    HAL_Delay(300);
    ZhuaZi(GRAB);
    ZhuanPan(OUT1);
#endif

    while (1) {
        if (WebControl_Task()) {
            start = 0;
            HAL_Delay(1);
            continue;
        }
        Logistics_Task();
        if (Logistics_IsRunning()) { HAL_Delay(1); continue; }
        if (VisionFollow_Task()) {
            start = 0;
            HAL_Delay(1);
            continue;
        }
        //一键启动，地盘银色按钮
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == 0) {
            start = 1;
            tasklimit = 55;
        } else {
            start = 0;
        }

        motor_state_machine();  //状态机
        snprintf(str, sizeof(str), "t0.txt=\"%.8s\"\xff\xff\xff", San_Data);  //扫码
        tjc_send_string(str);

        //调试归零
        if (zero == 1) {
            motor_zero();
            Serial_JY61P_Zero_Yaw();
            ZhuanPan(OUT1);
            ZhuaZi(GRAB);
            YunTai(IN);
            zero = 0;
            start = 0;
        }

        //////////////////视觉调试部分/////////////////////

        //命令1 识别红色物料

        if (D1 == 1) {
            ZhuaZi(OPEN);
            YunTai(OUT);
            Openmv_SendByte('a'); //开启补光灯
            HAL_Delay(10);
            Openmv_SendByte(1);
            D1 = 0;
        }

        //命令2 识别绿色物料
        if (D2 == 1) {
            ZhuaZi(OPEN);
            YunTai(OUT);
            Openmv_SendByte('a');  //开启补光灯
            HAL_Delay(10);
            Openmv_SendByte(2);
            D2 = 0;
        }

        //命令3 识别蓝色物料
        if (D3 == 1) {
            ZhuaZi(OPEN);
            YunTai(OUT);
            Openmv_SendByte('a');  //开启补光灯
            HAL_Delay(10);
            Openmv_SendByte(3);
            D3 = 0;
        }

        //命令5	识别绿色色环
        if (D4 == 1) {
            ZhuaZi(OPEN);
            YunTai(OUT);
            Openmv_SendByte('a');  //开启补光灯
            HAL_Delay(10);
            Openmv_SendByte(7);
            D4 = 0;
        }

        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
    }

    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();

    while (1) {
    }

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
