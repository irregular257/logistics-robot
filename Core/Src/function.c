#include "duiji.h"
#include "motion.h"
#include "function.h"
#include "main.h"
#include <stdlib.h>
#include "pid.h"



extern uint8_t Openmv_Annular_RxPacket[4];
extern uint8_t Openmv_Column_RxPacket[4];

extern tPid pid_omega1;
extern tPid pid_omega2;


float iop = 0;



float limit1(float input)
{

    if (input > 0.02) {  // 回到原来准确的速度
        input = 0.02;
    } else if (input < -0.02) {
        input = -0.02;
    }
	  // iop = input;
    return input;
}


float limit2(float input)
{

    if (input > 0.08) {  // 回到原来准确的速度
        input = 0.08;
    } else if (input < -0.08) {
        input = -0.08;
    }
	  // iop = input;
    return input;
}


  int16_t ex, ey;

void jiaqu4(int i)  //抓转盘
{
  if(i == 0x31)
  {
//		HAL_Delay(3000);
		YunTai(OUT);
    ZhuaZi(OPEN);
		HAL_Delay(400);
		down(0.078);
		HAL_Delay(400);
		ZhuanPan(HONG);
		HAL_Delay(300);
    ZhuaZi(GRAB);
    HAL_Delay(400);
    rise(0.078);
		HAL_Delay(300);
    YunTai(IN);
    HAL_Delay(850);
    down(0.042);
    HAL_Delay(500);
    ZhuaZi(OPEN);
    HAL_Delay(300);
    rise(0.042);
    HAL_Delay(200);
		YunTai(OUT);

  }
  if(i == 0x32)
  {
		YunTai(OUT);
    ZhuaZi(OPEN);
		HAL_Delay(400);
		down(0.078);
		HAL_Delay(400);
		ZhuanPan(LV);
		HAL_Delay(300);
    ZhuaZi(GRAB);
    HAL_Delay(400);
    rise(0.078);
		HAL_Delay(300);
    YunTai(IN);
    HAL_Delay(850);
    down(0.042);
    HAL_Delay(500);
    ZhuaZi(OPEN);
    HAL_Delay(300);
    rise(0.042);
    HAL_Delay(200);
		YunTai(OUT);
  }
  if(i == 0x33)
  {
		YunTai(OUT);
    ZhuaZi(OPEN);
		HAL_Delay(400);
		down(0.078);
		HAL_Delay(400);
		ZhuanPan(LAN);
		HAL_Delay(400);
    ZhuaZi(GRAB);
    HAL_Delay(300);
    rise(0.078);
		HAL_Delay(300);
    YunTai(IN);
    HAL_Delay(850);
    down(0.042);
    HAL_Delay(500);
    ZhuaZi(OPEN);
    HAL_Delay(300);
    rise(0.042);
    HAL_Delay(200);
		YunTai(OUT);
  }


}


    
 void jiaozhun2(uint16_t Targetx2,uint16_t Targety2)//绿色色环矫正x=80 y=70
{
 int m=1,n=1;
HAL_Delay(50);
	//调整前后（车子的前后）
	while(m)
	{
		int cx=Targetx2;
		
		pid_omega1.target_val = cx;
		
		ex=cx-(Openmv_Annular_RxPacket[0]);
		if(ex<0)
		{
			//ex=-ex;
			go(limit1(-PID_realize(&pid_omega1,Openmv_Annular_RxPacket[0])),180,0);//比例输出
			HAL_Delay(20);  // 平衡速度和精度
		}
		else if(ex>0)
		{
			go(limit1(PID_realize(&pid_omega1,Openmv_Annular_RxPacket[0])),0,0);//比例输出
			HAL_Delay(20);  // 平衡速度和精度
		}
		if(ex==0)
			m--;
		

		
}
    
  m=1;
	n=1;
	
	//调整左右（横移）
	while(n)
	{
		int cy=Targety2;
		
		pid_omega1.target_val = cy;
		
		ey=cy-(Openmv_Annular_RxPacket[1]);

		
		
    if(ey<0)
		{
			go(limit1(-PID_realize(&pid_omega1,Openmv_Annular_RxPacket[1])),90,0);//比例输出
			HAL_Delay(20);  // 平衡速度和精度
		}                                               
		else if(ey>0)
		{
			go(limit1(PID_realize(&pid_omega1,Openmv_Annular_RxPacket[1])),270,0);//比例输出
			HAL_Delay(20);  // 平衡速度和精度
		}
		if(ey==0)
		{
			n--;
		}
			
		
	}

	while(m)
	{
		int cx=Targetx2;
		
		pid_omega1.target_val = cx;
		
		ex=cx-(Openmv_Annular_RxPacket[0]);
		if(ex<0)
		{
			//ex=-ex;
			go(limit1(-PID_realize(&pid_omega1,Openmv_Annular_RxPacket[0])),180,0);//比例输出
			HAL_Delay(20);  // 平衡速度和精度
		}
		else if(ex>0)
		{
			go(limit1(PID_realize(&pid_omega1,Openmv_Annular_RxPacket[0])),0,0);//比例输出
			HAL_Delay(20);  // 平衡速度和精度
		}
		if(ex==0)
			m--;
			
}
	
go(0,0,0);
	
}

		
		

void fangzhi(int i)//放置地面
{
	
	if(i==0x31)
	{		
		
		ZhuaZi(OPEN);
		YunTai(IN);
		HAL_Delay(600);
		ZhuanPan(HONG);
		HAL_Delay(800);
		down(0.05);
		HAL_Delay(500);
		ZhuaZi(GRAB);
		HAL_Delay(200);
		rise(0.05);
		HAL_Delay(500);		
		YunTai(OUT);
		HAL_Delay(300);
		down(0.176);
		HAL_Delay(1000);
	  ZhuaZi(OPEN);
		HAL_Delay(100);
//		rise(0.179);
//		HAL_Delay(2000);
	}
	  
	if(i==0x32)
	{
		
		ZhuaZi(OPEN);
		YunTai(IN);
		HAL_Delay(600);
		ZhuanPan(LV);
		HAL_Delay(800);
		down(0.05);
		HAL_Delay(500);
		ZhuaZi(GRAB);
		HAL_Delay(200);
		rise(0.05);
		HAL_Delay(500);		
		YunTai(OUT);
		HAL_Delay(300);
		down(0.176);
		HAL_Delay(1000);
	  ZhuaZi(OPEN);
		HAL_Delay(100);
//		rise(0.179);
//		HAL_Delay(2000);
	
	
	}
	
	if(i==0x33)
	{   
		
		ZhuaZi(OPEN);
		YunTai(IN);
		HAL_Delay(600);
		ZhuanPan(LAN);
		HAL_Delay(800);
		down(0.05);
		HAL_Delay(500);
		ZhuaZi(GRAB);
		HAL_Delay(200);
		rise(0.05);
		HAL_Delay(500);		
		YunTai(OUT);
		HAL_Delay(300);
		down(0.176);
		HAL_Delay(1000);
	  ZhuaZi(OPEN);
		HAL_Delay(100);
//		rise(0.179);
//		HAL_Delay(2000);


	}
}


void jiaqu2(int i)//夹取地面上的物料
{
	//红色
	if(i==0x31)
	{												
				YunTai(OUT);
		ZhuaZi(OPEN);
//		HAL_Delay(200);
//		down(0.179);
		HAL_Delay(800);
		ZhuanPan(HONG);
		HAL_Delay(800);
		ZhuaZi(GRAB);
		HAL_Delay(400);
		rise(0.179);
		HAL_Delay(200);
		YunTai(IN);
		HAL_Delay(800);
		down(0.043);
		HAL_Delay(500);
		ZhuaZi(OPEN);
		HAL_Delay(100);
		rise(0.043);
		HAL_Delay(400);


	}			

	//绿色
	if(i==0x32)
	{
		YunTai(OUT);
		ZhuaZi(OPEN);
//		HAL_Delay(200);
//		down(0.179);
		HAL_Delay(800);
		ZhuanPan(LV);
		HAL_Delay(800);
		ZhuaZi(GRAB);
		HAL_Delay(400);
		rise(0.179);
		HAL_Delay(200);
		YunTai(IN);
		HAL_Delay(800);
		down(0.043);
		HAL_Delay(500);
		ZhuaZi(OPEN);
		HAL_Delay(100);
		rise(0.043);
		HAL_Delay(400);
	}
	
	//蓝色
	if(i==0x33)
	{
		YunTai(OUT);
		ZhuaZi(OPEN);
//		HAL_Delay(200);
//		down(0.179);
		HAL_Delay(800);
		ZhuanPan(LAN);
		HAL_Delay(800);
		ZhuaZi(GRAB);
		HAL_Delay(400);
		rise(0.179);
		HAL_Delay(200);
		YunTai(IN);
		HAL_Delay(800);
		down(0.043);
		HAL_Delay(500);
		ZhuaZi(OPEN);
		HAL_Delay(100);
		rise(0.043);
		HAL_Delay(400);
	}
}

 void jiaozhun5(uint16_t Targetx2,uint16_t Targety2)//x=164 y=142
{
 int m=1,n=1;
HAL_Delay(20);
	//调整前后（车子的前后）
	while(m)
	{
		int cx=Targetx2;
		
		pid_omega2.target_val = cx;

		ex=cx-(Openmv_Column_RxPacket[0]);
		if(ex<0)
		{
			//ex=-ex;
			go(limit2(-PID_realize(&pid_omega2,Openmv_Column_RxPacket[0])),180,0);
			HAL_Delay(20); 
		}
		else if(ex>0)
		{
			go(limit2(PID_realize(&pid_omega2,Openmv_Column_RxPacket[0])),0,0);
			HAL_Delay(20);  
		}
		if(abs(ex)<=1)
			m--;
		
				
		
		
	//}
		
}
    
  m=1;
	n=1;
	
	//调整左右（横移）
	while(n)
	{
		int cy=Targety2;
		
		pid_omega2.target_val = cy;
		
		ey=cy-(Openmv_Column_RxPacket[1]);

		
		
    if(ey<0)
		{
			go(limit2(-PID_realize(&pid_omega2,Openmv_Column_RxPacket[1])),90,0);
			HAL_Delay(20);  
		}                                               
		else if(ey>0)
		{
			go(limit2(PID_realize(&pid_omega2,Openmv_Column_RxPacket[1])),270,0);
			HAL_Delay(20);  
		}
		if(abs(ey)<=1)
		{
			n--;
		}
			
		
	}

	
	while(m)
	{
		int cx=Targetx2;
		
		pid_omega2.target_val = cx;

		ex=cx-(Openmv_Column_RxPacket[0]);
		if(ex<0)
		{
			//ex=-ex;
			go(limit2(-PID_realize(&pid_omega2,Openmv_Column_RxPacket[0])),180,0);
			HAL_Delay(20);  
		}
		else if(ex>0)
		{
			go(limit2(PID_realize(&pid_omega2,Openmv_Column_RxPacket[0])),0,0);
			HAL_Delay(20);  
		}
		if(abs(ex)<=1)
			m--;
		
		
		
	//}
		
}
	
     go(0,0,0);
	
}



void fangzhi7(int i)//码垛放置
{

	if(i==0x31)
	{		
		ZhuaZi(OPEN);
		YunTai(IN);
		HAL_Delay(600);
		ZhuanPan(HONG);
		HAL_Delay(800);
		down(0.05);
		HAL_Delay(700);
		ZhuaZi(GRAB);
		HAL_Delay(200);
		rise(0.05);
		HAL_Delay(400);		
		YunTai(OUT);
		HAL_Delay(400);
		down(0.10);
		HAL_Delay(800);
	  ZhuaZi(OPEN);
		HAL_Delay(100);
		rise(0.10);
		HAL_Delay(400);
		

	}
	
	if(i==0x32)
	{
		ZhuaZi(OPEN);
		YunTai(IN);
		HAL_Delay(600);
		ZhuanPan(LV);
		HAL_Delay(800);
		down(0.05);
		HAL_Delay(700);
		ZhuaZi(GRAB);
		HAL_Delay(200);
		rise(0.05);
		HAL_Delay(400);		
		YunTai(OUT);
		HAL_Delay(400);
		down(0.10);
		HAL_Delay(800);
	  ZhuaZi(OPEN);
		HAL_Delay(100);
		rise(0.10);
		HAL_Delay(400);

	}
	
	if(i==0x33)
	{   
		ZhuaZi(OPEN);
		YunTai(IN);
		HAL_Delay(600);
		ZhuanPan(LAN);
		HAL_Delay(800);
		down(0.05);
		HAL_Delay(700);
		ZhuaZi(GRAB);
		HAL_Delay(200);
		rise(0.05);
		HAL_Delay(400);		
		YunTai(OUT);
		HAL_Delay(400);
		down(0.10);
		HAL_Delay(800);
	  ZhuaZi(OPEN);
		HAL_Delay(100);
		rise(0.10);
		HAL_Delay(400);

	}



}
