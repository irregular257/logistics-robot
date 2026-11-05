#ifndef __MCTRL_H__
#define __MCTRL_H__

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
void chassis_xy(float v_set,float angle,float target_omega);
void split_float(float num, char *sign, float *value); 
void chassis_xydistance(float distance,float angle,float target_omega,uint8_t accel,float speed_rps);
void chassis_xyspeed_read(void);
void chassis_xydistance_read(void);
void chassis_xydistance_error(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MCTRL_H__ */
