#ifndef __MOTION_H__
#define __MOTION_H__
#include "main.h"

void go(float speed,float angle,float omega);
void back(float speed,float angle,float omega);
void godis(float speed);
void backdis(float speed);
void left(float a);
void right(float a);
void turn_left(void);
void turn_right(void);
void rise(float a);
void down(float a);
void zero_angle(float angle);

#endif
