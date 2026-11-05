#include "motion.h"
#include "function.h"
#include "duiji.h"
#include "stm32f4xx_it.h"
#include "control.h"

uint8_t TaskNum = 0;
uint8_t j = 1,i = 1;
uint8_t task[1]={99};

uint8_t anny[2];
extern uint8_t Openmv_Column_RxPacket[4];

void control(void)
{
    uint8_t a,b,c,d,e,f;
    a = Code_RxPacket[0];
    b = Code_RxPacket[1];
    c = Code_RxPacket[2];
    d = Code_RxPacket[4];
    e = Code_RxPacket[5];
    f = Code_RxPacket[6];
    //添加oled显示用于调试
    //OLED_Refesh();
    
    switch(TaskNum)
    {
        case 0:
            left(0);
            HAL_Delay(0);
            go(0);
            HAL_Delay(0);
            //HMI_send();//扫码
           // YunTai(OUT);
           //  ZhuaZi(OPEN);
						TaskNum = 1;
            case 1:
            go(0);
            HAL_Delay(0);
            right(0);
            HAL_Delay(0);
            down(0);
            HAL_Delay(0);
            //HMI_send();
            TaskNum = 5;//暂时略过抓取任务，直接开始任务5
            case 2:
            /**************第一个抓取物料******************/
                   if(a == 0x31)
                   {
                    int k = 1;
                    while (k)
                    {
                       Openmv_SendByte(1);
                        HAL_Delay(0);
                        if(a == 0x31 && task[0] == 1)
                        {
                            int j = 1;
                            while (j)
                            {
                                jiaqu4(0x31); 
                                ZhuanPan(OUT);
                                 ZhuaZi(OPEN);
                                YunTai(OUT);
                                HAL_Delay(0);
                                task[0] = 0;
                                j--;

                            }
                         }
                     }
                }
                  if(a == 0x32)
                  {
                    int k = 1;
                    while (k)
                    {
                       Openmv_SendByte(1);
                        HAL_Delay(0);
                        if(a == 0x32 && task[0] == 2)
                        {
                            int j = 1;
                            while (j)
                            {
                                jiaqu4(0x32);
                                ZhuanPan(OUT);
                                 ZhuaZi(OPEN);
                                YunTai(OUT);
                                HAL_Delay(0);
                                task[0] = 0; 
                                j--;

                            }
                         }
                     }
                  }
                  if(a == 0x33)
                  {
                    int k = 1;
                    while (k)
                    {
                       Openmv_SendByte(1);
                        HAL_Delay(0);
                        if(a == 0x33 && task[0] == 3)
                        {
                            int j = 1;
                            while (j)
                            {
                                jiaqu4(0x33);
                                ZhuanPan(OUT);
                                 ZhuaZi(OPEN);
                                YunTai(OUT);
                                HAL_Delay(0);
                                task[0] = 0;  
                                j--;

                            }
                         }
                     }
                  }
                  TaskNum = 3;
                  /*******************第二个抓的物料*******************/
                  case 3:
                  down(0);
                  HAL_Delay(0);
                  if(b == 0x31)
                   {
                    int k = 1;
                    while (k)
                    {
                       Openmv_SendByte(1);
                        HAL_Delay(0);
                        if(b == 0x31 && task[0] == 1)
                        {
                            int j = 1;
                            while (j)
                            {
                                jiaqu4(0x31); 
                                ZhuanPan(OUT);
                                 ZhuaZi(OPEN);
                                YunTai(OUT);
                                HAL_Delay(0);
                                task[0] = 0;
                                j--;

                            }
                         }
                     }
                }
                  if(b == 0x32)
                  {
                    int k = 1;
                    while (k)
                    {
                       Openmv_SendByte(1);
                        HAL_Delay(0);
                        if(b == 0x32 && task[0] == 2)
                        {
                            int j = 1;
                            while (j)
                            {
                                jiaqu4(0x32);
                                ZhuanPan(OUT);
                                 ZhuaZi(OPEN);
                                YunTai(OUT);
                                HAL_Delay(0);
                                task[0] = 0; 
                                j--;

                            }
                         }
                     }
                  }
                  if(b == 0x33)
                  {
                    int k = 1;
                    while (k)
                    {
                       Openmv_SendByte(1);
                        HAL_Delay(0);
                        if(b == 0x33 && task[0] == 3)
                        {
                            int j = 1;
                            while (j)
                            {
                                jiaqu4(0x33);
                                ZhuanPan(OUT);
                                 ZhuaZi(OPEN);
                                YunTai(OUT);
                                HAL_Delay(0);
                                task[0] = 0;  
                                j--;

                            }
                         }
                     }
                  }
                  TaskNum = 4;
                  /*******************第三个抓的物料*******************/
                  case 4:
                  down(0);
                  HAL_Delay(0);
                  if(c == 0x31)
                   {
                    int k = 1;
                    while (k)
                    {
                       Openmv_SendByte(1);
                        HAL_Delay(0);
                        if(c == 0x31 && task[0] == 1)
                        {
                            int j = 1;
                           while (j)
                           {
                                jiaqu4(0x31);
                                task[0] = 0;
                                j--;
                           }
                        }
                    }
                   }
                   if(c == 0x32)
                   {
                    int k = 1;
                    while (k)
                    {
                       Openmv_SendByte(1);
                        HAL_Delay(0);
                        if(c == 0x32 && task[0] == 2)
                        {
                            int j = 1;
                           while (j)
                           {
                                jiaqu4(0x32);
                                task[0] = 0;
                                j--;
                           }
                        }
                    }
                   }
                   if(c == 0x33)
                   {
                    int k = 1;
                    while (k)
                    {
                       Openmv_SendByte(1);
                        HAL_Delay(0);
                        if(c == 0x33 && task[0] == 3)
                        {
                            int j = 1;
                           while (j)
                           {
                                jiaqu4(0x33);
                                task[0] = 0;
                                j--;
                           }
                        }
                    }
                   }
                   YunTai(IN);
                    ZhuaZi(OPEN);
                   ZhuanPan(IN);
                   TaskNum = 5;
                   case 5:
                   /*******************第一次离开转盘*******************/
                   left(0);
                   HAL_Delay(0);
                   back(0);
                   HAL_Delay(0);
                   turn_right(0);
                   HAL_Delay(0);
                   go(0);
                   HAL_Delay(0);
                   turn_right(0);
                   HAL_Delay(0);
                   right(0);
                   HAL_Delay(0);
                   /*******************第一次放靶子*******************/
                   ZhuaZi(OPEN);
                   YunTai(OUT);
                   HAL_Delay(0);
                   down(0);
                   HAL_Delay(0);
                   Openmv_SendByte(5);
                   HAL_Delay(0);
                   jiaozhun2(0,0);//中心坐标
                   HAL_Delay(0);
									 TaskNum = 12;//略过抓取
									 case 6:
                   /********************第一个放靶************************/	
                   if(a == 0x33)
                   {
                    int k = 1;
                    while (k)
                    {
                        if(a == 0x33)
                        {
                            rise(0);
                            HAL_Delay(0);
                            go(0);
                            HAL_Delay(0);
                            fangzhi(0x33);
                             ZhuaZi(OPEN);
                            k--;                
                        }

                    }
                    
                   }
                   if(a == 0x32)
                   {
                    int k = 1;
                    while (k)
                    {
                        if(a == 0x32)
                        {
                            rise(0);
                            HAL_Delay(0);
                            fangzhi(0x32);
                             ZhuaZi(OPEN);
                            k--;                
                        }

                    }
                    
                   }
                   if(a == 0x31)
                   {
                    int k = 1;
                    while (k)
                    {
                        if(a == 0x31)
                        {
                            rise(0);
                            HAL_Delay(0);
                            fangzhi(0x31);
                             ZhuaZi(OPEN);
                            k--;                
                        }

                    }
                    
                   }
                HAL_Delay(0);
                rise(0);
            TaskNum=7;
            /********************第二个放靶************************/
            case 7:
            if(a == 0x31 && b == 0x32)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x31 && b == 0x32)
                    {
                        go(0);
                        HAL_Delay(0);
                        fangzhi(0x32);
                         ZhuaZi(OPEN);
                        k--;                
                    }
                }

            }
            if(a == 0x31 && b == 0x33)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x31 && b == 0x33)
                    {
                        go(0);
                        HAL_Delay(0);
                        fangzhi(0x33);
                         ZhuaZi(OPEN);
                        k--;               
                    }
                }
            }
            if(a == 0x32 && b == 0x31)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x32 && b == 0x31)
                    {
                        back(0);
                        HAL_Delay(0);
                        fangzhi(0x31);
                         ZhuaZi(OPEN);
                        k--;              
                    }
                }
            }
            if(a == 0x32 && b == 0x33)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x32 && b == 0x33)
                    {
                        go(0);
                        HAL_Delay(0);
                        fangzhi(0x33);
                         ZhuaZi(OPEN);
                        k--;             
                    }
                }
            }
            if(a == 0x33 && b == 0x31)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x33 && b == 0x31)
                    {
                        back(0);
                        HAL_Delay(0);
                        fangzhi(0x31);
                         ZhuaZi(OPEN);
                        k--;             
                    }
                }
            }
            if(a == 0x33 && b == 0x32)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x33 && b == 0x32)
                    {
                        back(0);
                        HAL_Delay(0);
                        fangzhi(0x32);
                         ZhuaZi(OPEN);
                        k--;            
                    }
                }
            }
            HAL_Delay(0);
            rise(0);
            TaskNum=8;
            /********************第三个放靶************************/
            case 8:
            if(b == 0x31 && c == 0x32)
            {
                int k = 1;
                while(k)
                {
                    if(b == 0x31 && c ==0x32)
                    {
                        go(0);
                        HAL_Delay(0);
                        fangzhi(0x32);
                         ZhuaZi(OPEN);
                        k--;                
    
                }
            }
        }
            if(b == 0x31 && c == 0x33)
            {
                int k = 1;
                while(k)
                {
                    if(b == 0x31 && c == 0x33)
                    {
                        go(0);
                        HAL_Delay(0);
                        fangzhi(0x33);
                         ZhuaZi(OPEN);
                        k--;              
                    }
                }
            }
            if(b == 0x32 && c == 0x31)
            {
                int k = 1;
                while(k)
                {
                    if(b == 0x32 && c == 0x31)
                    {
                        back(0);
                        HAL_Delay(0);
                        fangzhi(0x31);
                         ZhuaZi(OPEN);
                        k--;             
                    }
                }
            }
            if(b == 0x32 && c == 0x33)
            {
                int k = 1;
                while(k)
                {
                    if(b == 0x32 && c == 0x33)
                    {
                        go(0);
                        HAL_Delay(0);
                        fangzhi(0x33);
                         ZhuaZi(OPEN);
                        k--;            
                    }
                }
            }
            if(b == 0x33 && c == 0x31)
            {
                int k = 1;
                while(k)
                {
                    if(b == 0x33 && c == 0x31)
                    {
                        back(0);
                        HAL_Delay(0);
                        fangzhi(0x31);
                         ZhuaZi(OPEN);
                        k--;            
                    }
                }
            }
            if(b == 0x33 && c == 0x32)
            {
                int k = 1;
                while(k)
                {
                    if(b == 0x33 && c == 0x32)
                    {
                        back(0);
                        HAL_Delay(0);
                        fangzhi(0x32);
                         ZhuaZi(OPEN);
                        k--;            
                    }
                }
            }

            TaskNum=9;
            case 9:
                
            /********************拿：抓放下去的物料************************/
            
            //123 132 213 231 312 321六种情况
            if(a == 0x31 && b == 0x32 && c == 0x33)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x31 && b == 0x32 && c == 0x33)
                    {
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        back(0);	
                        YunTai(OUT);
                        HAL_Delay(0);
                        jiaqu2(0x31);
                        go(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x32);
                        go(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x33);
                        k--;             
                    }
                }
            }
            if(a == 0x31 && b == 0x33 && c == 0x32)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x31 && b == 0x33 && c == 0x32)
                    {
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        back(0);	
                        YunTai(OUT);
                        HAL_Delay(0);
                        jiaqu2(0x31);
                        go(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x33);
                        back(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x32);
                        k--;
                    }
                }
            }
            if(a == 0x32 && b == 0x31 && c == 0x33)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x32 && b == 0x31 && c == 0x33)
                    {
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        back(0);	
                        YunTai(OUT);
                        HAL_Delay(0);
                        jiaqu2(0x32);
                        back(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x31);
                        go(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x33);
                        k--;
                    }
                }
            }
            if(a == 0x32 && b == 0x33 && c == 0x31)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x32 && b == 0x33 && c == 0x31)
                    {
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        go(0);	
                        YunTai(OUT);
                        HAL_Delay(0);
                        jiaqu2(0x32);
                        go(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x33);
                        back(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x31);
                        k--;
                    }
                }
            }
            if(a == 0x33 && b == 0x31 && c == 0x32)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x33 && b == 0x31 && c == 0x32)
                    {
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        go(0);	
                        YunTai(OUT);
                        HAL_Delay(0);
                        jiaqu2(0x33);
                        back(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x31);
                        go(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x32);
                        k--;
                    }
                }
            }
            if(a == 0x33 && b == 0x32 && c == 0x31)
            {
                int k = 1;
                while(k)
                {
                    if(a == 0x33 && b == 0x32 && c == 0x31)
                    {
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        go(0);	
                        YunTai(OUT);
                        HAL_Delay(0);
                        jiaqu2(0x33);
                        back(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x32);
                        back(0);
                        YunTai(OUT);
                         ZhuaZi(OPEN);
                        HAL_Delay(0);
                        down(0);
                        HAL_Delay(0);
                        jiaqu2(0x31);
                        k--;
                    }
                }
            }
        TaskNum=12;
        case 12:
         	/********************前进到精加工区************************/
				      rise(0);
		          HAL_Delay(0);
		          ZhuaZi(GRAB);
				      //三个都试一遍
            if(c == 0x31) //根据位置调整后退距离
            {
                left(0);
                HAL_Delay(0);    
                back(0);
                HAL_Delay(0);
                turn_right(0);
                HAL_Delay(0);
                back(0);
                HAL_Delay(0);
            }
            else if(c == 0x32)
            {
                left(0);
                HAL_Delay(0);    
                back(0);
                HAL_Delay(0);
                turn_right(0);
                HAL_Delay(0);
                back(0);
                HAL_Delay(0);
            }
            else if(c == 0x33)
            {
                left(0);
                HAL_Delay(0);    
                back(0);
                HAL_Delay(0);
                turn_right(0);
                HAL_Delay(0);
                back(0);
                HAL_Delay(0);
            }
            /********************放置物料，和之前的流程一样************************/
            right(0);
            ZhuaZi(OPEN);
            YunTai(OUT);
            HAL_Delay(0);
            down(0);
            HAL_Delay(0);
            Openmv_SendByte(5);
            HAL_Delay(0);
            jiaozhun2(0,0);
            HAL_Delay(0);
						TaskNum = 19;//略过
						case 13:
            /********************第一个放靶************************/	
            if(a == 0x33)
            {
             int k = 1;
             while (k)
             {
                 if(a == 0x33)
                 {
                     rise(0);
                     HAL_Delay(0);
                     go(0);
                     HAL_Delay(0);
                     fangzhi(0x33);
                      ZhuaZi(OPEN);
                     k--;                
                 }

             }
             
            }
            if(a == 0x32)
            {
             int k = 1;
             while (k)
             {
                 if(a == 0x32)
                 {
                     rise(0);
                     HAL_Delay(0);
                     fangzhi(0x32);
                      ZhuaZi(OPEN);
                     k--;                
                 }

             }
             
            }
            if(a == 0x31)
            {
             int k = 1;
             while (k)
             {
                 if(a == 0x31)
                 {
                     rise(0);
                     HAL_Delay(0);
                     fangzhi(0x31);
                      ZhuaZi(OPEN);
                     k--;                
                 }

             }
             
            }
         HAL_Delay(0);
         rise(0);
     TaskNum=14;
     /********************第二个放靶************************/
     case 14:
     if(a == 0x31 && b == 0x32)
     {
         int k = 1;
         while(k)
         {
             if(a == 0x31 && b == 0x32)
             {
                 go(0);
                 HAL_Delay(0);
                 fangzhi(0x32);
                  ZhuaZi(OPEN);
                 k--;                
             }
         }

     }
     if(a == 0x31 && b == 0x33)
     {
         int k = 1;
         while(k)
         {
             if(a == 0x31 && b == 0x33)
             {
                 go(0);
                 HAL_Delay(0);
                 fangzhi(0x33);
                  ZhuaZi(OPEN);
                 k--;               
             }
         }
     }
     if(a == 0x32 && b == 0x31)
     {
         int k = 1;
         while(k)
         {
             if(a == 0x32 && b == 0x31)
             {
                 back(0);
                 HAL_Delay(0);
                 fangzhi(0x31);
                  ZhuaZi(OPEN);
                 k--;              
             }
         }
     }
     if(a == 0x32 && b == 0x33)
     {
         int k = 1;
         while(k)
         {
             if(a == 0x32 && b == 0x33)
             {
                 go(0);
                 HAL_Delay(0);
                 fangzhi(0x33);
                  ZhuaZi(OPEN);
                 k--;             
             }
         }
     }
     if(a == 0x33 && b == 0x31)
     {
         int k = 1;
         while(k)
         {
             if(a == 0x33 && b == 0x31)
             {
                 back(0);
                 HAL_Delay(0);
                 fangzhi(0x31);
                  ZhuaZi(OPEN);
                 k--;             
             }
         }
     }
     if(a == 0x33 && b == 0x32)
     {
         int k = 1;
         while(k)
         {
             if(a == 0x33 && b == 0x32)
             {
                 back(0);
                 HAL_Delay(0);
                 fangzhi(0x32);
                  ZhuaZi(OPEN);
                 k--;            
             }
         }
     }
     HAL_Delay(0);
     rise(0);
     TaskNum=15;
     /********************第三个放靶************************/
     case 15:
     if(b == 0x31 && c == 0x32)
     {
         int k = 1;
         while(k)
         {
             if(b == 0x31 && c ==0x32)
             {
                 go(0);
                 HAL_Delay(0);
                 fangzhi(0x32);
                  ZhuaZi(OPEN);
                 k--;                

         }
     }
 }
     if(b == 0x31 && c == 0x33)
     {
         int k = 1;
         while(k)
         {
             if(b == 0x31 && c == 0x33)
             {
                 go(0);
                 HAL_Delay(0);
                 fangzhi(0x33);
                  ZhuaZi(OPEN);
                 k--;              
             }
         }
     }
     if(b == 0x32 && c == 0x31)
     {
         int k = 1;
         while(k)
         {
             if(b == 0x32 && c == 0x31)
             {
                 back(0);
                 HAL_Delay(0);
                 fangzhi(0x31);
                  ZhuaZi(OPEN);
                 k--;             
             }
         }
     }
     if(b == 0x32 && c == 0x33)
     {
         int k = 1;
         while(k)
         {
             if(b == 0x32 && c == 0x33)
             {
                 go(0);
                 HAL_Delay(0);
                 fangzhi(0x33);
                  ZhuaZi(OPEN);
                 k--;            
             }
         }
     }
     if(b == 0x33 && c == 0x31)
     {
         int k = 1;
         while(k)
         {
             if(b == 0x33 && c == 0x31)
             {
                 back(0);
                 HAL_Delay(0);
                 fangzhi(0x31);
                  ZhuaZi(OPEN);
                 k--;            
             }
         }
     }
     if(b == 0x33 && c == 0x32)
     {
         int k = 1;
         while(k)
         {
             if(b == 0x33 && c == 0x32)
             {
                 back(0);
                 HAL_Delay(0);
                 fangzhi(0x32);
                  ZhuaZi(OPEN);
                 k--;            
             }
         }
     }
     HAL_Delay(0);
     rise(0);
     TaskNum=19;
     case 19:
     /**********************前进至物料区**********************/	
		 rise(0);
		 HAL_Delay(0);
		 ZhuaZi(GRAB);
		 //三个都试一遍
     if(c == 0x31)//根据位置调整实际距离
     {
        left(0);
        HAL_Delay(0);
        back(0);
        HAL_Delay(0);
        turn_right(0);
        HAL_Delay(0);
        back(0);
        HAL_Delay(0);
        right(0);
        HAL_Delay(0);
     }
     else if(c == 0x32)
     {
        left(0);
        HAL_Delay(0);
        back(0);
        HAL_Delay(0);
        turn_right(0);
        HAL_Delay(0);
        back(0);
        HAL_Delay(0);
        right(0);
        HAL_Delay(0);
     }
     else if(c == 0x33)
     {
        left(0);
        HAL_Delay(0);
        back(0);
        HAL_Delay(0);
        turn_right(0);
        HAL_Delay(0);
        back(0);
        HAL_Delay(0);
        right(0);
        HAL_Delay(0);
     }
     TaskNum=20;
     case 20:
     /**********************抓取第二轮物料**********************/
     ZhuaZi(OPEN);
	   YunTai(OUT);
     Openmv_SendByte(9);//多色块检测，用于后面颜色较准
     HAL_Delay(0);
     while(j)
     {
        if(Openmv_Column_RxPacket[2]==9)
        {
            anny[0]=0x04;
            anny[1]=Openmv_Column_RxPacket[3];
            Openmv_SendArray(anny,2);
        }
     
     while(i)
     {
        if(Openmv_Column_RxPacket[2]==4)//接收到校准指令
        {
            jiaozhun5(0,0);//定位物料
            HAL_Delay(0);
            down(0);
            HAL_Delay(0);
            j--;
            i--;
        }

     }
    }
    TaskNum=24;//略过抓取	
    case 21:
/*******************第一个抓的物料*******************/
if(d == 0x31)
{
 int k = 1;
 while (k)
 {
    Openmv_SendByte(1);
     HAL_Delay(0);
     if(d == 0x31 && task[0] == 1)
     {
         int j = 1;
         while (j)
         {
             jiaqu4(0x31); 
             ZhuanPan(OUT);
              ZhuaZi(OPEN);
             YunTai(OUT);
             HAL_Delay(0);
             task[0] = 0;
             j--;

         }
      }
  }
}
if(d == 0x32)
{
 int k = 1;
 while (k)
 {
    Openmv_SendByte(1);
     HAL_Delay(0);
     if(d == 0x32 && task[0] == 2)
     {
         int j = 1;
         while (j)
         {
             jiaqu4(0x32);
             ZhuanPan(OUT);
              ZhuaZi(OPEN);
             YunTai(OUT);
             HAL_Delay(0);
             task[0] = 0; 
             j--;

         }
      }
  }
}
if(d == 0x33)
{
 int k = 1;
 while (k)
 {
    Openmv_SendByte(1);
     HAL_Delay(0);
     if(d == 0x33 && task[0] == 3)
     {
         int j = 1;
         while (j)
         {
             jiaqu4(0x33);
             ZhuanPan(OUT);
              ZhuaZi(OPEN);
             YunTai(OUT);
             HAL_Delay(0);
             task[0] = 0;  
             j--;

         }
      }
  }
}
TaskNum = 22;
/*******************第二个抓的物料*******************/
case 22:
down(0);
HAL_Delay(0);
if(e == 0x31)
{
 int k = 1;
 while (k)
 {
    Openmv_SendByte(1);
     HAL_Delay(0);
     if(e == 0x31 && task[0] == 1)
     {
         int j = 1;
         while (j)
         {
             jiaqu4(0x31); 
             ZhuanPan(OUT);
              ZhuaZi(OPEN);
             YunTai(OUT);
             HAL_Delay(0);
             task[0] = 0;
             j--;

         }
      }
  }
}
if(e == 0x32)
{
 int k = 1;
 while (k)
 {
    Openmv_SendByte(1);
     HAL_Delay(0);
     if(e == 0x32 && task[0] == 2)
     {
         int j = 1;
         while (j)
         {
             jiaqu4(0x32);
             ZhuanPan(OUT);
              ZhuaZi(OPEN);
             YunTai(OUT);
             HAL_Delay(0);
             task[0] = 0; 
             j--;

         }
      }
  }
}
if(e == 0x33)
{
 int k = 1;
 while (k)
 {
    Openmv_SendByte(1);
     HAL_Delay(0);
     if(e == 0x33 && task[0] == 3)
     {
         int j = 1;
         while (j)
         {
             jiaqu4(0x33);
             ZhuanPan(OUT);
              ZhuaZi(OPEN);
             YunTai(OUT);
             HAL_Delay(0);
             task[0] = 0;  
             j--;

         }
      }
  }
}
TaskNum = 23;
/*******************第三个抓的物料*******************/
case 23:
down(0);
HAL_Delay(0);
if(f == 0x31)
{
 int k = 1;
 while (k)
 {
    Openmv_SendByte(1);
     HAL_Delay(0);
     if(f == 0x31 && task[0] == 1)
     {
         int j = 1;
        while (j)
        {
             jiaqu4(0x31);
             task[0] = 0;
             j--;
        }
     }
 }
}
if(f == 0x32)
{
 int k = 1;
 while (k)
 {
    Openmv_SendByte(1);
     HAL_Delay(0);
     if(f == 0x32 && task[0] == 2)
     {
         int j = 1;
        while (j)
        {
             jiaqu4(0x32);
             task[0] = 0;
             j--;
        }
     }
 }
}
if(f == 0x33)
{
 int k = 1;
 while (k)
 {
    Openmv_SendByte(1);
     HAL_Delay(0);
     if(f == 0x33 && task[0] == 3)
     {
         int j = 1;
        while (j)
        {
             jiaqu4(0x33);
             task[0] = 0;
             j--;
        }
     }
 }
}
YunTai(IN);
 ZhuaZi(OPEN);
ZhuanPan(IN);
TaskNum = 24;
case 24:
/*******************第二次离开转盘*******************/
ZhuaZi(GRAB);
YunTai(IN);
left(0);
HAL_Delay(0);
back(0);
HAL_Delay(0);
turn_right(0);
HAL_Delay(0);
go(0);
HAL_Delay(0);
turn_right(0);
HAL_Delay(0);
right(0);
HAL_Delay(0);
/*******************第一次放靶子*******************/
ZhuaZi(OPEN);
YunTai(OUT);
HAL_Delay(0);
down(0);
HAL_Delay(0);
Openmv_SendByte(5);
HAL_Delay(0);
jiaozhun2(0,0);
HAL_Delay(0);
TaskNum = 29;//略过抓取
case 25:
/********************第一个放靶************************/	
if(d == 0x33)
{
 int k = 1;
 while (k)
 {
     if(d == 0x33)
     {
         rise(0);
         HAL_Delay(0);
         go(0);
         HAL_Delay(0);
         fangzhi(0x33);
          ZhuaZi(OPEN);
         k--;                
     }

 }
 
}
if(d == 0x32)
{
 int k = 1;
 while (k)
 {
     if(d == 0x32)
     {
         rise(0);
         HAL_Delay(0);
         fangzhi(0x32);
          ZhuaZi(OPEN);
         k--;                
     }

 }
 
}
if(d == 0x31)
{
 int k = 1;
 while (k)
 {
     if(d == 0x31)
     {
         rise(0);
         HAL_Delay(0);
         fangzhi(0x31);
          ZhuaZi(OPEN);
         k--;                
     }

 }
 
}
HAL_Delay(0);
rise(0);
TaskNum=26;
/********************第二个放靶************************/
case 26:
if(d == 0x31 && e == 0x32)
{
int k = 1;
while(k)
{
 if(d == 0x31 && e == 0x32)
 {
     go(0);
     HAL_Delay(0);
     fangzhi(0x32);
      ZhuaZi(OPEN);
     k--;                
 }
}

}
if(d == 0x31 && e == 0x33)
{
int k = 1;
while(k)
{
 if(d == 0x31 && e == 0x33)
 {
     go(0);
     HAL_Delay(0);
     fangzhi(0x33);
      ZhuaZi(OPEN);
     k--;               
 }
}
}
if(d == 0x32 && e == 0x31)
{
int k = 1;
while(k)
{
 if(d == 0x32 && e == 0x31)
 {
     back(0);
     HAL_Delay(0);
     fangzhi(0x31);
      ZhuaZi(OPEN);
     k--;              
 }
}
}
if(d == 0x32 && e == 0x33)
{
int k = 1;
while(k)
{
 if(d == 0x32 && e == 0x33)
 {
     go(0);
     HAL_Delay(0);
     fangzhi(0x33);
      ZhuaZi(OPEN);
     k--;             
 }
}
}
if(d == 0x33 && e == 0x31)
{
int k = 1;
while(k)
{
 if(d == 0x33 && e == 0x31)
 {
     back(0);
     HAL_Delay(0);
     fangzhi(0x31);
      ZhuaZi(OPEN);
     k--;             
 }
}
}
if(d == 0x33 && e == 0x32)
{
int k = 1;
while(k)
{
 if(d == 0x33 && e == 0x32)
 {
     back(0);
     HAL_Delay(0);
     fangzhi(0x32);
      ZhuaZi(OPEN);
     k--;            
 }
}
}
HAL_Delay(0);
rise(0);
TaskNum=27;
/********************第三个放靶************************/
case 27:
if(e == 0x31 && f == 0x32)
{
int k = 1;
while(k)
{
 if(e == 0x31 && f ==0x32)
 {
     go(0);
     HAL_Delay(0);
     fangzhi(0x32);
      ZhuaZi(OPEN);
     k--;                

}
}
}
if(e == 0x31 && f == 0x33)
{
int k = 1;
while(k)
{
 if(e == 0x31 && f == 0x33)
 {
     go(0);
     HAL_Delay(0);
     fangzhi(0x33);
      ZhuaZi(OPEN);
     k--;              
 }
}
}
if(e == 0x32 && f == 0x31)
{
int k = 1;
while(k)
{
 if(e == 0x32 && f == 0x31)
 {
     back(0);
     HAL_Delay(0);
     fangzhi(0x31);
      ZhuaZi(OPEN);
     k--;             
 }
}
}
if(e == 0x32 && f == 0x33)
{
int k = 1;
while(k)
{
 if(e == 0x32 && f == 0x33)
 {
     go(0);
     HAL_Delay(0);
     fangzhi(0x33);
      ZhuaZi(OPEN);
     k--;            
 }
}
}
if(e == 0x33 && f == 0x31)
{
int k = 1;
while(k)
{
 if(e == 0x33 && f == 0x31)
 {
     back(0);
     HAL_Delay(0);
     fangzhi(0x31);
      ZhuaZi(OPEN);
     k--;            
 }
}
}
if(e == 0x33 && f == 0x32)
{
int k = 1;
while(k)
{
 if(e == 0x33 && f == 0x32)
 {
     back(0);
     HAL_Delay(0);
     fangzhi(0x32);
      ZhuaZi(OPEN);
     k--;            
 }
}
}

TaskNum=28;
case 28:

/********************拿：抓放下去的物料************************/

//123 132 213 231 312 321六种情况
if(d == 0x31 && e == 0x32 && f == 0x33)
{
int k = 1;
while(k)
{
 if(d == 0x31 && e == 0x32 && f == 0x33)
 {
      ZhuaZi(OPEN);
     HAL_Delay(0);
     back(0);	
     YunTai(OUT);
     HAL_Delay(0);
     jiaqu2(0x31);
     go(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x32);
     go(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x33);
     k--;             
 }
}
}
if(d == 0x31 && e == 0x33 && f == 0x32)
{
int k = 1;
while(k)
{
 if(d == 0x31 && e == 0x33 && f == 0x32)
 {
      ZhuaZi(OPEN);
     HAL_Delay(0);
     back(0);	
     YunTai(OUT);
     HAL_Delay(0);
     jiaqu2(0x31);
     go(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x33);
     back(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x32);
     k--;
 }
}
}
if(d == 0x32 && e == 0x31 && f == 0x33)
{
int k = 1;
while(k)
{
 if(d == 0x32 && e == 0x31 && f == 0x33)
 {
      ZhuaZi(OPEN);
     HAL_Delay(0);
     back(0);	
     YunTai(OUT);
     HAL_Delay(0);
     jiaqu2(0x32);
     back(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x31);
     go(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x33);
     k--;
 }
}
}
if(d == 0x32 && e == 0x33 && f == 0x31)
{
int k = 1;
while(k)
{
 if(d == 0x32 && e == 0x33 && f == 0x31)
 {
      ZhuaZi(OPEN);
     HAL_Delay(0);
     go(0);	
     YunTai(OUT);
     HAL_Delay(0);
     jiaqu2(0x32);
     go(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x33);
     back(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x31);
     k--;
 }
}
}
if(d == 0x33 && e == 0x31 && f == 0x32)
{
int k = 1;
while(k)
{
 if(d == 0x33 && e == 0x31 && f == 0x32)
 {
      ZhuaZi(OPEN);
     HAL_Delay(0);
     go(0);	
     YunTai(OUT);
     HAL_Delay(0);
     jiaqu2(0x33);
     back(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x31);
     go(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x32);
     k--;
 }
}
}
if(d == 0x33 && e == 0x32 && f == 0x31)
{
int k = 1;
while(k)
{
 if(d == 0x33 && e == 0x32 && f == 0x31)
 {
      ZhuaZi(OPEN);
     HAL_Delay(0);
     go(0);	
     YunTai(OUT);
     HAL_Delay(0);
     jiaqu2(0x33);
     back(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x32);
     back(0);
     YunTai(OUT);
      ZhuaZi(OPEN);
     HAL_Delay(0);
     down(0);
     HAL_Delay(0);
     jiaqu2(0x31);
     k--;
 }
}
}
TaskNum=29;
case 29:
/********************前进到精加工区************************/
rise(0);
HAL_Delay(0);
ZhuaZi(GRAB);
//三个都试一遍
if(c == 0x31) //根据位置调整后退距离
{
left(0);
HAL_Delay(0);    
back(0);
HAL_Delay(0);
turn_right(0);
HAL_Delay(0);
back(0);
HAL_Delay(0);
}
else if(c == 0x32)
{
left(0);
HAL_Delay(0);    
back(0);
HAL_Delay(0);
turn_right(0);
HAL_Delay(0);
back(0);
HAL_Delay(0);
}
else if(c == 0x33)
{
left(0);
HAL_Delay(0);    
back(0);
HAL_Delay(0);
turn_right(0);
HAL_Delay(0);
back(0);
HAL_Delay(0);
}
/*******************放置物料，和之前的流程一样************************/	
/********************第一个放靶************************/
    ZhuaZi(OPEN);
		YunTai(OUT);
		HAL_Delay(0);
		down(0);
		HAL_Delay(0);
		Openmv_SendByte(7);//发送指令进行中间的定位
		HAL_Delay(0);
		jiaozhun5(0,0);//先定位中间的
		HAL_Delay(0);
TaskNum = 33;//略过抓取
case 30:
        /********************第一个放靶************************/
        if(d == 0x33)
        {
         int k = 1;
         while (k)
         {
             if(d == 0x33)
             {
                 rise(0);
                 HAL_Delay(0);
                 go(0);
                 HAL_Delay(0);
                 fangzhi7(0x33);
                  ZhuaZi(OPEN);
                 k--;                
             }

         }
         
        }
        if(d == 0x32)
        {
         int k = 1;
         while (k)
         {
             if(d == 0x32)
             {
                 rise(0);
                 HAL_Delay(0);
                 fangzhi7(0x32);
                  ZhuaZi(OPEN);
                 k--;                
             }

         }
         
        }
        if(d == 0x31)
        {
         int k = 1;
         while (k)
         {
             if(d == 0x31)
             {
                 rise(0);
                 HAL_Delay(0);
                 fangzhi7(0x31);
                  ZhuaZi(OPEN);
                 k--;                
             }

         }
         
        }
     HAL_Delay(0);
     rise(0);
 TaskNum=31;
 /********************第二个放靶************************/
 case 31:
 if(d == 0x31 && e == 0x32)
 {
     int k = 1;
     while(k)
     {
         if(d == 0x31 && e == 0x32)
         {
             go(0);
             HAL_Delay(0);
             fangzhi7(0x32);
              ZhuaZi(OPEN);
             k--;                
         }
     }

 }
 if(d == 0x31 && e == 0x33)
 {
     int k = 1;
     while(k)
     {
         if(d == 0x31 && e == 0x33)
         {
             go(0);
             HAL_Delay(0);
             fangzhi7(0x33);
              ZhuaZi(OPEN);
             k--;               
         }
     }
 }
 if(d == 0x32 && e == 0x31)
 {
     int k = 1;
     while(k)
     {
         if(d == 0x32 && e == 0x31)
         {
             back(0);
             HAL_Delay(0);
             fangzhi7(0x31);
              ZhuaZi(OPEN);
             k--;              
         }
     }
 }
 if(d == 0x32 && e == 0x33)
 {
     int k = 1;
     while(k)
     {
         if(d == 0x32 && e == 0x33)
         {
             go(0);
             HAL_Delay(0);
             fangzhi7(0x33);
              ZhuaZi(OPEN);
             k--;             
         }
     }
 }
 if(d == 0x33 && e == 0x31)
 {
     int k = 1;
     while(k)
     {
         if(d == 0x33 && e == 0x31)
         {
             back(0);
             HAL_Delay(0);
             fangzhi7(0x31);
              ZhuaZi(OPEN);
             k--;             
         }
     }
 }
 if(d == 0x33 && e == 0x32)
 {
     int k = 1;
     while(k)
     {
         if(d == 0x33 && e == 0x32)
         {
             back(0);
             HAL_Delay(0);
             fangzhi7(0x32);
              ZhuaZi(OPEN);
             k--;            
         }
     }
 }
 HAL_Delay(0);
 rise(0);
 TaskNum=32;
 /********************第三个放靶************************/
 case 32:
 if(e == 0x31 && f == 0x32)
 {
     int k = 1;
     while(k)
     {
         if(e == 0x31 && f ==0x32)
         {
             go(0);
             HAL_Delay(0);
             fangzhi7(0x32);
              ZhuaZi(OPEN);
             k--;                

     }
 }
}
 if(e == 0x31 && f == 0x33)
 {
     int k = 1;
     while(k)
     {
         if(e == 0x31 && f == 0x33)
         {
             go(0);
             HAL_Delay(0);
             fangzhi7(0x33);
              ZhuaZi(OPEN);
             k--;              
         }
     }
 }
 if(e == 0x32 && f == 0x31)
 {
     int k = 1;
     while(k)
     {
         if(e == 0x32 && f == 0x31)
         {
             back(0);
             HAL_Delay(0);
             fangzhi7(0x31);
              ZhuaZi(OPEN);
             k--;             
         }
     }
 }
 if(e == 0x32 && f == 0x33)
 {
     int k = 1;
     while(k)
     {
         if(e == 0x32 && f == 0x33)
         {
             go(0);
             HAL_Delay(0);
             fangzhi7(0x33);
              ZhuaZi(OPEN);
             k--;            
         }
     }
 }
 if(e == 0x33 && f == 0x31)
 {
     int k = 1;
     while(k)
     {
         if(e == 0x33 && f == 0x31)
         {
             back(0);
             HAL_Delay(0);
             fangzhi7(0x31);
              ZhuaZi(OPEN);
             k--;            
         }
     }
 }
 if(e == 0x33 && f == 0x32)
 {
     int k = 1;
     while(k)
     {
         if(e == 0x33 && f == 0x32)
         {
             back(0);
             HAL_Delay(0);
             fangzhi7(0x32);
              ZhuaZi(OPEN);
             k--;            
         }
     }
 }
 HAL_Delay(0);
 rise(0);
 HAL_Delay(0);
 TaskNum=33;
 case 33:
 /**********************回到起点**********************/	
 rise(0);
 HAL_Delay(0);
 ZhuaZi(GRAB);
 //三个都试一遍
 if(f==0x31)
 {
     YunTai(IN);
     ZhuanPan(IN);
     left(0);
     HAL_Delay(0);
     back(0);//需要变动
     HAL_Delay(0);
     turn_left();
     
     HAL_Delay(0);
     back(0);//需要变动
     HAL_Delay(0);
     right(0);
 }
 
 else if(f==0x32)
 {
     YunTai(IN);
     ZhuanPan(IN);
     left(0);
     HAL_Delay(0);
     back(0);//需要变动
     HAL_Delay(0);
     turn_left();
     
     HAL_Delay(0);
     back(0);//需要变动
     HAL_Delay(0);
     right(0);
 }
 else if(f==0x33)
 {
     YunTai(IN);
     ZhuanPan(IN);
     left(0);
     HAL_Delay(0);
     back(0);//需要变动
     HAL_Delay(0);
     turn_left();
     HAL_Delay(0);
     back(0);//需要变动
     HAL_Delay(0);
     right(0);
 }
    break;
  }



}


