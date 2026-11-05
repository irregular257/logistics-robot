#include "mctrl.h"
#include "roboarm.h"
#include "motion.h"
#include "math.h"
//单位是米,可精确到小数点后6位
double Pi=3.14159265358979323846 ;

void go(float speed,float angle,float omega) //速度
{
    chassis_xy(speed,angle,omega);
}

void back(float speed,float angle,float omega)
{
    chassis_xy(speed,angle,omega);
}


void godis(float speed)//位置
{
    chassis_xydistance(speed,0.6,0,0x78,2);
}

void backdis(float speed)//位置
{
    chassis_xydistance(speed,0.6+180,0,0x78,2);
}

void zero_angle(float angle)
{
	
 chassis_xydistance(0,0,Pi*(angle/90),0XB4,2);

}



void left(float a)
{
    chassis_xydistance(a,90,0,0x96,4);
}

void right(float a)
{
    chassis_xydistance(a,270,0,0x96,4);
}

void turn_left()
{
    chassis_xydistance(0,0,Pi*0.91,0xB4,4);
}

void turn_right()
{
    chassis_xydistance(0,0,-Pi*0.91,0xB4,4);
}


void rise(float a)
{
	distance(a,1,12,0xf0);//距离速度待测
}

void down(float a)
{
	distance(a,0,12,0xe6);//距离速度待测
}

