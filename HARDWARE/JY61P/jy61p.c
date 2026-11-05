#include "jy61p.h"

static uint8_t RxBuffer[11];/*接收数据数组*/
static volatile uint8_t RxState = 0;/*接收状态标志位*/
static uint8_t RxIndex = 0;/*接受数组索引*/
float Roll,Pitch,Yaw;/*角度信息，映射后范围：-135°~225°*/
static volatile uint32_t yaw_tick;
static volatile uint8_t yaw_seen;

uint8_t JY61P_HasFreshYaw(uint32_t max_age_ms)
{ return yaw_seen && (uint32_t)(HAL_GetTick()-yaw_tick)<=max_age_ms; }


/**
 * @brief       数据包处理函数
 * @param       串口接收的数据RxData
 * @retval      无
 */
void jy61p_ReceiveData(uint8_t RxData)
{
	uint8_t i,sum=0;
	if (RxState == 0)	//等待包头
	{
		if (RxData == 0x55)	//收到包头
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 1;
			RxIndex = 1;	//进入下一状态
		}
	}
	else if (RxState == 1)
	{
		if (RxData == 0x53)	/*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 2;
			RxIndex = 2;	//进入下一状态
		}
	}
	else if (RxState == 2)	//接收数据
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//接收完成
		{
			for(i=0;i<10;i++)
			{
				sum = sum + RxBuffer[i];	//计算校验和
			}
			if(sum == RxBuffer[10])		//校验成功
			{
				yaw_tick=HAL_GetTick(); yaw_seen=1U;
				/*计算数据，根据数据内容选择对应的计算公式*/
				Roll = ((int16_t) ((int16_t) RxBuffer[3] << 8 | (int16_t) RxBuffer[2])) / 32768.0f * 180.0f;
				Pitch = ((int16_t) ((int16_t) RxBuffer[5] << 8 | (int16_t) RxBuffer[4])) / 32768.0f * 180.0f;
				Yaw = ((int16_t) ((int16_t) RxBuffer[7] << 8 | (int16_t) RxBuffer[6])) / 32768.0f * 180.0f;
				
				// 将Yaw从-180~+180映射到0~225和-135~0
				// 逆时针旋转序列：0° -> 1° -> ... -> 225° -> -135° -> -134° -> ... -> -1° -> 0°
				// 映射规则：
				// 原始 0° -> 0°, 原始 180° -> 180°
				// 原始 -1° -> -1°, 原始 -135° -> -135°
				// 原始 -179° -> 181°, 原始 -136° -> 224°
				if (Yaw >= 0.0f) {
					// 0° 到 180° 保持不变
				}
				else if (Yaw >= -135.0f) {
					// -135° 到 0° 保持不变
				}
				else {
					// -180° 到 -136° 映射到 180° 到 224°
					Yaw = Yaw + 360.0f;
				}
			}
			RxState = 0;
			RxIndex = 0;	//读取完成，回到最初状态，等待包头
		}
	}
}


void Serial_JY61P_Zero_Yaw(void)
{

    extern UART_HandleTypeDef huart5;

    // 第一组指令: 解锁
    uint8_t unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    HAL_UART_Transmit(&huart5, unlock_cmd, sizeof(unlock_cmd), 100);
    HAL_Delay(100);

    // 第二组指令: Z轴归零
    uint8_t zero_yaw_cmd[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
    HAL_UART_Transmit(&huart5, zero_yaw_cmd, sizeof(zero_yaw_cmd), 100);
    HAL_Delay(100);

    // 第三组指令: 保存设置
    uint8_t save_cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    HAL_UART_Transmit(&huart5, save_cmd, sizeof(save_cmd), 100);
}
