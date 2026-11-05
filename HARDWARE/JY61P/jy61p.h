#ifndef __JY61P_H
#define __JY61P_H

#include "main.h" 

void jy61p_ReceiveData(uint8_t RxData);
void Serial_JY61P_Zero_Yaw(void);
uint8_t JY61P_HasFreshYaw(uint32_t max_age_ms);

extern float Roll,Pitch,Yaw;  // Yaw范围：0°~225°和-135°~0°

#endif
