#ifndef __STATE_H__
#define __STATE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


/* USER CODE BEGIN Prototypes */
void motor_state_machine(void);
uint8_t Motor_TaskIsIdle(void); 
void stop_zero_turn();
void stop_zero();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MCTRL_H__ */
