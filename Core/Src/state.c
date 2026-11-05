#include "state.h"
#include "mctrl.h"
#include "motor.h"
#include "main.h"
#include "stm32f4xx_it.h"
#include "mctrl.h"
#include "motion.h"
#include "pid.h"
#include "jy61p.h"
#include "tjc_usart_hmi.h"
#include "duiji.h"
#include <math.h>
#include "function.h"
#include "string.h"  // ???? memset

typedef enum {
    STATE_IDLE,
    STATE_MOVING,
    STATE_DONE,
    STATE_ROBOARM,
    STATE_SPEED,
    STATE_DISTANCE

} MotorState;


extern uint8_t flag;
extern uint8_t task1[1];
MotorState motor_state = STATE_IDLE;
MotorState move_state = STATE_SPEED;
uint16_t task1_flag = 1;
float set_x = 0; //设定x轴距离
float set_y = 0; //设定y轴距离
float set_angle = 0; //设定陀螺仪校准角度
int control_flag = 0;
extern double vx;
extern tPid pid_omega, pid_omega1;
int start = 0, zero = 0;
extern uint8_t a, b, c, d, e, f, g;
int dir = 1; //运行方向，1前进0后退
uint8_t j = 1, i = 1;
int wait_count = 0;
extern uint8_t Openmv_Column_RxPacket[4];

int16_t tasklimit = 0;
void task1_dis(uint16_t task_id);
uint8_t detect_cmd = 0;


float limit(float input)
{
    if (input > 1) {
        input = 1;
    } else if (input < -1) {
        input = -1;
    }

    return input;
}

/**
 * @brief   底盘方向校准
 * @details None
 * @param   None
 * @retval  None
 * @note    None
 */


void stop_zero()
{
    float err = set_angle - Yaw;

    if (fabs(Yaw - set_angle) > 0.2) {
        zero_angle(err);
        HAL_Delay(500);//等待校准
    }
}


/**
 * @brief   主状态机
 * @details 由于张大头步进闭环缺陷，命令间必须加延时，导致无法在中断中调用电机控制接口，故暂且放在状态机中
 *					采用给速度，读距离的方式，实现底盘校准并走到的距离，由于速度模式给了加速度，设定距离并不准，需
 *          估算调试
 * @param   None
 * @retval  None
 * @note    None
 */

uint8_t Motor_TaskIsIdle(void)
{
    return motor_state == STATE_IDLE && (task1_flag == 1 || task1_flag == 200);
}

void motor_state_machine()
{
    if (start == 1) {
        switch (motor_state) {
        case STATE_IDLE:
            task1_dis(task1_flag);
            break;

        case STATE_MOVING:
            if (flag == 1) {
                motor_state = STATE_DONE;
            } else {
                switch (move_state) {
                case STATE_SPEED:
                    for (unsigned char i = 0; i < 4; i++) {
                        motor_readdistance(i + 1);
                    }

                    data_cal();
                    chassis_xydistance_read();

                    if ((set_x - vx) < 0.0005) {
                        flag = 1;
                    } else {
                        if (control_flag == 1) {
                            if (dir == 1) {
                                go(2.5, 0.68, limit(PID_realize(&pid_omega, Yaw)));
                            } else if (dir == 0) {
                                back(2.5, 180 + 0.68, limit(PID_realize(&pid_omega, Yaw)));
                            }

                            control_flag = 0;
                        }
                    }

                    break;

                case STATE_ROBOARM:
                    break;

                //                case STATE_DISTANCE:
                //                    for (unsigned char i = 0; i < 4; i++) {
                //                        motor_readstate(i + 1);
                //                    }

                //                    data_cal();
                //                    chassis_xydistance_read();

                //                    if (((bj1.state & 0x03) == 0x03) && ((bj2.state & 0x03) == 0x03) && ((bj3.state & 0x03) == 0x03)
                //                            && ((bj4.state & 0x03) == 0x03)) {
                //                        flag = 1;
                //                    } else {
                //                        flag = 0;
                //                    }

                //                    break;

                default:
                    break;
                }
            }

            break;

        case STATE_DONE:
            flag = 0;
            go(0, 0, 0);
            task1_flag++;

            /*
            *任务限制
            *可用于单步任务调试
            */
            if (task1_flag >= tasklimit) {
                task1_flag = 200;
            }

            motor_state = STATE_IDLE;
            break;

        default:
            motor_state = STATE_IDLE;
            break;
        }
    }
}

/****************************任务开始******************************/

void task1_dis(uint16_t task_id)
{
    switch (task_id) {
    case 200:
        break;

    case 199:
        move_state = STATE_SPEED;
        set_angle = 0;
        motor_state = STATE_MOVING;
        break;

    case 1:
        move_state = STATE_SPEED;
        left(0.2);
        HAL_Delay(1000);
        stop_zero();
        flag = 1;
        set_x = 0;
        set_angle = 0;
        motor_state = STATE_MOVING;
        break;

    case 2:
        move_state = STATE_SPEED;
        motor_zero();
        set_x = 0.30;
        dir = 1;
        set_angle = 0;
        motor_state = STATE_MOVING;
        break;

    case 3:
        move_state = STATE_SPEED;
        HAL_Delay(600);
        stop_zero();
        motor_zero();
        YunTai(OUT);
        ZhuaZi(OPEN);
        HAL_Delay(300);
        stop_zero();
        motor_zero();
        set_x = 0.605;
        dir = 1;
        set_angle = 0;
        motor_state = STATE_MOVING;
        break;

    case 4:
        HAL_Delay(600);
        stop_zero();
        motor_zero();
        move_state = STATE_ROBOARM;
        right(0.079);
        HAL_Delay(800);
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 5:
        move_state = STATE_ROBOARM;

        /*
         *滞后抓取逻辑
        *如果机械臂刚到物料区域就看见第一个要抓取的颜色则不抓，
        *等第二次看见再进行抓取，若第一次没看见则一旦看见就执
         *行抓取。
        */
        if (a == 0x31) {
            detect_cmd = 21;
        } else if (a == 0x32) {
            detect_cmd = 22;
        } else if (a == 0x33) {
            detect_cmd = 23;
        }

        if (wait_count == 0) {
            memset(Openmv_Column_RxPacket, 0, sizeof(Openmv_Column_RxPacket));
            Openmv_SendByte(detect_cmd);
            HAL_Delay(10);
            Openmv_SendByte('a');
            HAL_Delay(1500);
            wait_count = 1;
            break;
        }

        if (wait_count == 1) {
            static uint8_t detect_66_count = 0;
            uint8_t response = Openmv_Column_RxPacket[0];

            if (response == 55 || response == detect_cmd) {
                memset(Openmv_Column_RxPacket, 0, sizeof(Openmv_Column_RxPacket));
                detect_66_count = 0;
                wait_count = 2;
                break;
            } else if (response == 66) {
                detect_66_count++;

                if (detect_66_count >= 3) {
                    memset(Openmv_Column_RxPacket, 0, sizeof(Openmv_Column_RxPacket));
                    detect_66_count = 0;
                    wait_count = 5;
                    break;
                }
            }

            break;
        }

        if (wait_count == 2) {
            if (Openmv_Column_RxPacket[0] == 66) {
                memset(Openmv_Column_RxPacket, 0, sizeof(Openmv_Column_RxPacket));
                wait_count = 3;
                break;
            }

            break;
        }

        if (wait_count == 3) {
            if (Openmv_Column_RxPacket[0] == detect_cmd) {
                wait_count = 4;
                break;
            } else {
                break;
            }
        }

        if (wait_count == 5) {
            if (Openmv_Column_RxPacket[0] == detect_cmd) {
                wait_count = 4;
                break;
            } else {
                break;
            }
        }

        if (wait_count == 4) {
            if (a == 0x31) {
                int k = 1;
                Openmv_SendByte(1);
                HAL_Delay(10);
                Openmv_SendByte('a');
                HAL_Delay(10);

                while (k) {
                    if (a == 0x31 && task1[0] == 1) {
                        int j = 1;

                        while (j) {
                            jiaqu4(0x31);
                            ZhuaZi(OPEN);
                            YunTai(OUT);
                            HAL_Delay(700);
                            task1[0] = 0;
                            j--;
                            k--;
                            flag = 1;
                        }
                    }
                }
            }

            if (a == 0x32) {
                int k = 1;
                Openmv_SendByte(2);
                HAL_Delay(10);
                Openmv_SendByte('a');

                while (k) {
                    if (a == 0x32 && task1[0] == 1) {
                        int j = 1;

                        while (j) {
                            jiaqu4(0x32);
                            // ZhuanPan(OUT1);
                            ZhuaZi(OPEN);
                            YunTai(OUT);
                            HAL_Delay(700);
                            task1[0] = 0;
                            j--;
                            k--;
                            flag = 1;
                        }
                    }
                }
            }

            if (a == 0x33) {
                int k = 1;
                Openmv_SendByte(3);
                HAL_Delay(10);
                Openmv_SendByte('a');

                while (k) {
                    if (a == 0x33 && task1[0] == 1) {
                        int j = 1;

                        while (j) {
                            jiaqu4(0x33);
                            ZhuanPan(OUT1);
                            ZhuaZi(OPEN);
                            HAL_Delay(500);
                            YunTai(OUT);
                            HAL_Delay(700);
                            task1[0] = 0;
                            j--;
                            k--;
                            flag = 1;
                        }
                    }
                }
            }

            wait_count = 0;
            motor_state = STATE_MOVING;
        }

        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 6:
        move_state = STATE_ROBOARM;

        if (b == 0x31) {
            int k = 1;
            Openmv_SendByte(1);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (b == 0x31 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x31);
                        ZhuaZi(OPEN);
                        YunTai(OUT);
                        HAL_Delay(700);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        if (b == 0x32) {
            int k = 1;
            Openmv_SendByte(2);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (b == 0x32 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x32);
                        ZhuaZi(OPEN);
                        YunTai(OUT);
                        HAL_Delay(700);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        if (b == 0x33) {
            int k = 1;
            Openmv_SendByte(3);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (b == 0x33 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x33);
                        ZhuaZi(OPEN);
                        YunTai(OUT);
                        HAL_Delay(700);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        motor_state = STATE_MOVING;
        break;

    case 7:
        move_state = STATE_ROBOARM;

        if (c == 0x31) {
            int k = 1;
            Openmv_SendByte(1);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (c == 0x31 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x31);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        if (c == 0x32) {
            int k = 1;
            Openmv_SendByte(2);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (c == 0x32 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x32);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        if (c == 0x33) {
            int k = 1;
            Openmv_SendByte(3);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (c == 0x33 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x33);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        YunTai(IN);
        ZhuaZi(OPEN);
        Openmv_SendByte('b');
        ZhuanPan(OUT1);
        motor_state = STATE_MOVING;
        break;

    case 8:
        move_state = STATE_ROBOARM;
        YunTai(IN);
        HAL_Delay(500);
        stop_zero();
        left(0.096);
        HAL_Delay(800);
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 9:
        move_state = STATE_SPEED;
        set_x = 0.14;
        dir = 0;
        set_angle = 0;
        motor_state = STATE_MOVING;
        break;

    case 10:
        move_state = STATE_ROBOARM;
        HAL_Delay(700);
        stop_zero();
        motor_zero();
        turn_left();
        HAL_Delay(1400);
        set_angle = 90;
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 11:
        move_state = STATE_SPEED;
        set_x = 1.356;
        dir = 1;
        set_angle = 90;
        motor_state = STATE_MOVING;
        break;

    case 12:
        move_state = STATE_ROBOARM;
        HAL_Delay(800);
        stop_zero();
        motor_zero();
        turn_left();
        flag = 1;
        HAL_Delay(1400);
        set_angle = 180;
        stop_zero();
        motor_zero();
        motor_state = STATE_MOVING;
        break;

    case 13:
        move_state = STATE_SPEED;
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 14:
        move_state = STATE_ROBOARM;
        right(0.035);
        ZhuaZi(OPEN);
        YunTai(OUT);
        HAL_Delay(800);
        stop_zero();
        motor_zero();
        Openmv_SendByte('a');
        HAL_Delay(500);
        Openmv_SendByte(5);
        HAL_Delay(10);
        jiaozhun2(80, 66);
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 15:
        move_state = STATE_ROBOARM;

        if (a == 0x33) {
            int k = 1;

            while (k) {
                if (a == 0x33) {
                    godis(0.138);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x32) {
            int k = 1;

            while (k) {
                if (a == 0x32) {
                    godis(0);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x31) {
            int k = 1;

            while (k) {
                if (a == 0x31) {
                    backdis(0.138);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        rise(0.176);
        HAL_Delay(600);
        motor_state = STATE_MOVING;
        break;

    case 16:
        move_state = STATE_ROBOARM;

        if (a == 0x31 && b == 0x32) {
            int k = 1;

            while (k) {
                if (a == 0x31 && b == 0x32) {
                    //  HAL_Delay(2000);
                    godis(0.138);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x31 && b == 0x33) {
            int k = 1;

            while (k) {
                if (a == 0x31 && b == 0x33) {
                    godis(0.276);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x32 && b == 0x31) {
            int k = 1;

            while (k) {
                if (a == 0x32 && b == 0x31) {
                    backdis(0.138);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x32 && b == 0x33) {
            int k = 1;

            while (k) {
                if (a == 0x32 && b == 0x33) {
                    godis(0.138);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x33 && b == 0x31) {
            int k = 1;

            while (k) {
                if (a == 0x33 && b == 0x31) {
                    backdis(0.276);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x33 && b == 0x32) {
            int k = 1;

            while (k) {
                if (a == 0x33 && b == 0x32) {
                    backdis(0.138);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        rise(0.176);
        HAL_Delay(600);
        motor_state = STATE_MOVING;
        break;

    case 17:
        move_state = STATE_ROBOARM;

        if (b == 0x31 && c == 0x32) {
            int k = 1;

            while (k) {
                if (b == 0x31 && c == 0x32) {
                    godis(0.138);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (b == 0x31 && c == 0x33) {
            int k = 1;

            while (k) {
                if (b == 0x31 && c == 0x33) {
                    godis(0.275);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (b == 0x32 && c == 0x31) {
            int k = 1;

            while (k) {
                if (b == 0x32 && c == 0x31) {
                    backdis(0.138);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (b == 0x32 && c == 0x33) {
            int k = 1;

            while (k) {
                if (b == 0x32 && c == 0x33) {
                    godis(0.138);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (b == 0x33 && c == 0x31) {
            int k = 1;

            while (k) {
                if (b == 0x33 && c == 0x31) {
                    backdis(0.275);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (b == 0x33 && c == 0x32) {
            int k = 1;

            while (k) {
                if (b == 0x33 && c == 0x32) {
                    backdis(0.138);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        Openmv_SendByte('b');
        motor_state = STATE_MOVING;
        break;

    case 18:
        move_state = STATE_DISTANCE;

        if (a == 0x31 && b == 0x32 && c == 0x33) {
            int k = 1;

            while (k) {
                if (a == 0x31 && b == 0x32 && c == 0x33) {
                    backdis(0.276);
                    jiaqu2(0x31);
                    godis(0.138);
                    down(0.179);
                    jiaqu2(0x32);
                    godis(0.138);
                    down(0.179);
                    jiaqu2(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x31 && b == 0x33 && c == 0x32) {
            int k = 1;

            while (k) {
                if (a == 0x31 && b == 0x33 && c == 0x32) {
                    backdis(0.138);
                    jiaqu2(0x31);
                    godis(0.276);
                    down(0.179);
                    jiaqu2(0x33);
                    backdis(0.138);
                    down(0.179);
                    jiaqu2(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x32 && b == 0x31 && c == 0x33) {
            int k = 1;

            while (k) {
                if (a == 0x32 && b == 0x31 && c == 0x33) {
                    backdis(0.138);
                    jiaqu2(0x32);
                    backdis(0.138);
                    down(0.179);
                    jiaqu2(0x31);
                    godis(0.276);
                    down(0.179);
                    jiaqu2(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x32 && b == 0x33 && c == 0x31) {
            int k = 1;

            while (k) {
                if (a == 0x32 && b == 0x33 && c == 0x31) {
                    godis(0.138);
                    jiaqu2(0x32);
                    godis(0.138);
                    down(0.179);
                    jiaqu2(0x33);
                    backdis(0.276);
                    down(0.179);
                    jiaqu2(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x33 && b == 0x31 && c == 0x32) {
            int k = 1;

            while (k) {
                if (a == 0x33 && b == 0x31 && c == 0x32) {
                    godis(0.138);
                    jiaqu2(0x33);
                    backdis(0.276);
                    down(0.179);
                    jiaqu2(0x31);
                    godis(0.138);
                    down(0.179);
                    jiaqu2(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x33 && b == 0x32 && c == 0x31) {
            int k = 1;

            while (k) {
                if (a == 0x33 && b == 0x32 && c == 0x31) {
                    godis(0.276);
                    jiaqu2(0x33);
                    backdis(0.138);
                    down(0.179);
                    jiaqu2(0x32);
                    backdis(0.138);
                    down(0.179);
                    jiaqu2(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        motor_state = STATE_MOVING;
        break;

    case 19:
        move_state = STATE_SPEED;

        if (c == 0x31) {
            left(0.08);
            HAL_Delay(600);
            set_angle = 180;
            stop_zero();
            motor_zero();
            set_x = 0.38;
            dir = 0;
        } else if (c == 0x32) {
            left(0.08);
            HAL_Delay(600);
            set_angle = 180;
            stop_zero();
            motor_zero();
            set_x = 0.53;
            dir = 0;
        } else if (c == 0x33) {
            left(0.08);
            HAL_Delay(600);
            set_angle = 180;
            stop_zero();
            motor_zero();
            set_x = 0.67;
            dir = 0;
        }

        motor_state = STATE_MOVING;
        break;

    case 20:
        move_state = STATE_ROBOARM;
        HAL_Delay(650);
        stop_zero();
        motor_zero();
        set_angle = 90;
        turn_right();
        HAL_Delay(1400);
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 21:
        move_state = STATE_SPEED;
        set_x = 0.55;
        dir = 0;
        set_angle = 90;
        motor_state = STATE_MOVING;
        break;

    case 22:
        move_state = STATE_ROBOARM;
        HAL_Delay(700);
        stop_zero();
        motor_zero();
        right(0.09);
        ZhuaZi(OPEN);
        YunTai(OUT);
        HAL_Delay(500);
        stop_zero();
        Openmv_SendByte('a');
        HAL_Delay(500);
        Openmv_SendByte(5);
        HAL_Delay(200);
        jiaozhun2(80, 66);
        stop_zero();
        Openmv_SendByte('b');
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 23:
        move_state = STATE_ROBOARM;

        if (a == 0x33) {
            int k = 1;

            while (k) {
                if (a == 0x33) {
                    godis(0.139);
                    stop_zero();
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x32) {
            int k = 1;

            while (k) {
                if (a == 0x32) {
                    godis(0);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x31) {
            int k = 1;

            while (k) {
                if (a == 0x31) {
                    backdis(0.139);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        rise(0.176);
        HAL_Delay(600);
        motor_state = STATE_MOVING;
        break;

    case 24:
        move_state = STATE_ROBOARM;

        if (a == 0x31 && b == 0x32) {
            int k = 1;

            while (k) {
                if (a == 0x31 && b == 0x32) {
                    godis(0.139);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x31 && b == 0x33) {
            int k = 1;

            while (k) {
                if (a == 0x31 && b == 0x33) {
                    godis(0.278);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x32 && b == 0x31) {
            int k = 1;

            while (k) {
                if (a == 0x32 && b == 0x31) {
                    backdis(0.139);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x32 && b == 0x33) {
            int k = 1;

            while (k) {
                if (a == 0x32 && b == 0x33) {
                    godis(0.139);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x33 && b == 0x31) {
            int k = 1;

            while (k) {
                if (a == 0x33 && b == 0x31) {
                    backdis(0.278);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (a == 0x33 && b == 0x32) {
            int k = 1;

            while (k) {
                if (a == 0x33 && b == 0x32) {
                    backdis(0.139);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        rise(0.176);
        HAL_Delay(600);
        motor_state = STATE_MOVING;
        break;

    case 25:
        move_state = STATE_ROBOARM;

        if (b == 0x31 && c == 0x32) {
            int k = 1;

            while (k) {
                if (b == 0x31 && c == 0x32) {
                    godis(0.139);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (b == 0x31 && c == 0x33) {
            int k = 1;

            while (k) {
                if (b == 0x31 && c == 0x33) {
                    godis(0.278);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (b == 0x32 && c == 0x31) {
            int k = 1;

            while (k) {
                if (b == 0x32 && c == 0x31) {
                    backdis(0.139);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (b == 0x32 && c == 0x33) {
            int k = 1;

            while (k) {
                if (b == 0x32 && c == 0x33) {
                    godis(0.138);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (b == 0x33 && c == 0x31) {
            int k = 1;

            while (k) {
                if (b == 0x33 && c == 0x31) {
                    backdis(0.275);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (b == 0x33 && c == 0x32) {
            int k = 1;

            while (k) {
                if (b == 0x33 && c == 0x32) {
                    backdis(0.139);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        Openmv_SendByte('b');
        motor_state = STATE_MOVING;
        rise(0.176);
        HAL_Delay(600);
        break;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////?????//////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    case 26:
        move_state = STATE_SPEED;

        if (c == 0x31) {
            left(0.09);
            HAL_Delay(600);
            set_angle = 90;
            stop_zero();
            motor_zero();
            set_x = 0.47;
            dir = 0;
        } else if (c == 0x32) {
            left(0.09);
            HAL_Delay(600);
            set_angle = 90;
            stop_zero();
            motor_zero();
            set_x = 0.60;
            dir = 0;
        } else if (c == 0x33) {
            left(0.09);
            HAL_Delay(600);
            set_angle = 90;
            stop_zero();
            motor_zero();
            set_x = 0.73;
            dir = 0;
        }

        motor_state = STATE_MOVING;
        break;

    case 27:
        move_state = STATE_ROBOARM;
        HAL_Delay(500);
        stop_zero();
        motor_zero();
        set_angle = 0;
        turn_right();
        HAL_Delay(1400);
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 28:
        move_state = STATE_SPEED;
        set_x = 0.16;
        dir = 0;
        set_angle = 0;
        motor_state = STATE_MOVING;
        break;

    case 29:
        move_state = STATE_ROBOARM;
        HAL_Delay(1000);
        stop_zero();
        motor_zero();
        ZhuaZi(OPEN);
        YunTai(OUT);
        right(0.055);
        HAL_Delay(1000);
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 30:
        move_state = STATE_ROBOARM;
        HAL_Delay(10);

        if (d == 0x31) {
            detect_cmd = 21;
        } else if (d == 0x32) {
            detect_cmd = 22;
        } else if (d == 0x33) {
            detect_cmd = 23;
        } else {
            break;
        }

        if (wait_count == 0) {
            memset(Openmv_Column_RxPacket, 0, sizeof(Openmv_Column_RxPacket));
            Openmv_SendByte(detect_cmd);
            HAL_Delay(10);
            Openmv_SendByte('a');
            HAL_Delay(1500);
            wait_count = 1;
            break;
        }

        if (wait_count == 1) {
            static uint8_t detect_66_count = 0;
            uint8_t response = Openmv_Column_RxPacket[0];

            if (response == 55 || response == detect_cmd) {
                memset(Openmv_Column_RxPacket, 0, sizeof(Openmv_Column_RxPacket));
                detect_66_count = 0;
                wait_count = 2;
                break;
            } else if (response == 66) {
                detect_66_count++;

                if (detect_66_count >= 3) {
                    memset(Openmv_Column_RxPacket, 0, sizeof(Openmv_Column_RxPacket));
                    detect_66_count = 0;
                    wait_count = 5;
                    break;
                }
            }

            break;
        }

        if (wait_count == 2) {
            if (Openmv_Column_RxPacket[0] == 66) {
                memset(Openmv_Column_RxPacket, 0, sizeof(Openmv_Column_RxPacket));
                wait_count = 3;
                break;
            }

            break;
        }

        if (wait_count == 3) {
            if (Openmv_Column_RxPacket[0] == detect_cmd) {
                wait_count = 4;
                break;
            } else {
                break;
            }
        }

        if (wait_count == 5) {
            if (Openmv_Column_RxPacket[0] == detect_cmd) {
                wait_count = 4;
                break;
            } else {
                break;
            }
        }

        if (wait_count == 4) {
            if (d == 0x31) {
                int k = 1;
                Openmv_SendByte(1);
                HAL_Delay(10);
                Openmv_SendByte('a');
                HAL_Delay(10);

                while (k) {
                    if (d == 0x31 && task1[0] == 1) {
                        int j = 1;

                        while (j) {
                            jiaqu4(0x31);
                            // ZhuanPan(OUT1);
                            ZhuaZi(OPEN);
                            YunTai(OUT);
                            HAL_Delay(700);
                            task1[0] = 0;
                            j--;
                            k--;
                            flag = 1;
                        }
                    }
                }
            }

            if (d == 0x32) {
                int k = 1;
                Openmv_SendByte(2);
                HAL_Delay(10);
                Openmv_SendByte('a');

                while (k) {
                    if (d == 0x32 && task1[0] == 1) {
                        int j = 1;

                        while (j) {
                            jiaqu4(0x32);
                            // ZhuanPan(OUT1);
                            ZhuaZi(OPEN);
                            YunTai(OUT);
                            HAL_Delay(700);
                            task1[0] = 0;
                            j--;
                            k--;
                            flag = 1;
                        }
                    }
                }
            }

            if (d == 0x33) {
                int k = 1;
                Openmv_SendByte(3);
                HAL_Delay(10);
                Openmv_SendByte('a');

                while (k) {
                    if (d == 0x33 && task1[0] == 1) {
                        int j = 1;

                        while (j) {
                            jiaqu4(0x33);
                            ZhuaZi(OPEN);
                            YunTai(OUT);
                            HAL_Delay(700);
                            task1[0] = 0;
                            j--;
                            k--;
                            flag = 1;
                        }
                    }
                }
            }

            wait_count = 0;
            motor_state = STATE_MOVING;
        }

        break;

    case 31:
        move_state = STATE_ROBOARM;

        if (e == 0x31) {
            int k = 1;
            Openmv_SendByte(1);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (e == 0x31 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x31);
                        ZhuaZi(OPEN);
                        YunTai(OUT);
                        HAL_Delay(700);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        if (e == 0x32) {
            int k = 1;
            Openmv_SendByte(2);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (e == 0x32 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x32);
                        ZhuaZi(OPEN);
                        YunTai(OUT);
                        HAL_Delay(700);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        if (e == 0x33) {
            int k = 1;
            Openmv_SendByte(3);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (e == 0x33 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x33);
                        ZhuaZi(OPEN);
                        YunTai(OUT);
                        HAL_Delay(700);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        motor_state = STATE_MOVING;
        break;

    case 32:
        move_state = STATE_ROBOARM;

        if (f == 0x31) {
            int k = 1;
            Openmv_SendByte(1);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (f == 0x31 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x31);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        if (f == 0x32) {
            int k = 1;
            Openmv_SendByte(2);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (f == 0x32 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x32);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        if (f == 0x33) {
            int k = 1;
            Openmv_SendByte(3);
            HAL_Delay(10);
            Openmv_SendByte('a');

            while (k) {
                if (f == 0x33 && task1[0] == 1) {
                    int j = 1;

                    while (j) {
                        jiaqu4(0x33);
                        task1[0] = 0;
                        j--;
                        k--;
                        flag = 1;
                    }
                }
            }
        }

        YunTai(IN);
        ZhuaZi(OPEN);
        Openmv_SendByte('b');
        ZhuanPan(OUT1);
        motor_state = STATE_MOVING;
        break;

    case 33:
        move_state = STATE_ROBOARM;
        //	HAL_Delay(2000);
        YunTai(IN);
        HAL_Delay(500);
        stop_zero();
        left(0.08);
        HAL_Delay(800);
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 34:
        move_state = STATE_SPEED;
        motor_zero();
        set_x = 0.145;
        dir = 0;
        set_angle = 0;
        motor_state = STATE_MOVING;
        break;

    case 35:
        move_state = STATE_ROBOARM;
        HAL_Delay(700);
        stop_zero();
        motor_zero();
        turn_left();
        HAL_Delay(1400);
        set_angle = 90;
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 36:
        move_state = STATE_SPEED;
        set_x = 1.356;
        dir = 1;
        set_angle = 90;
        motor_state = STATE_MOVING;
        break;

    case 37:
        move_state = STATE_ROBOARM;
        HAL_Delay(800);
        stop_zero();
        motor_zero();
        turn_left();
        flag = 1;
        HAL_Delay(1400);
        set_angle = 180;
        stop_zero();
        motor_zero();
        //  HAL_Delay(800);
        motor_state = STATE_MOVING;
        break;

    case 38:
        move_state = STATE_SPEED;
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 39:
        move_state = STATE_ROBOARM;
        right(0.04);
        ZhuaZi(OPEN);
        YunTai(OUT);
        HAL_Delay(800);
        stop_zero();
        motor_zero();
        HAL_Delay(10);
        Openmv_SendByte('a');
        HAL_Delay(10);
        Openmv_SendByte(5);
        HAL_Delay(200);
        jiaozhun2(80, 66);
        stop_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 40:
        move_state = STATE_ROBOARM;

        if (d == 0x33) {
            int k = 1;

            while (k) {
                if (d == 0x33) {
                    godis(0.139);
                    stop_zero();
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x32) {
            int k = 1;

            while (k) {
                if (d == 0x32) {
                    godis(0);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x31) {
            int k = 1;

            while (k) {
                if (d == 0x31) {
                    backdis(0.139);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        rise(0.176);
        HAL_Delay(600);
        motor_state = STATE_MOVING;
        break;

    case 41:
        move_state = STATE_ROBOARM;

        if (d == 0x31 && e == 0x32) {
            int k = 1;

            while (k) {
                if (d == 0x31 && e == 0x32) {
                    godis(0.139);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x31 && e == 0x33) {
            int k = 1;

            while (k) {
                if (d == 0x31 && e == 0x33) {
                    godis(0.278);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x32 && e == 0x31) {
            int k = 1;

            while (k) {
                if (d == 0x32 && e == 0x31) {
                    backdis(0.139);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x32 && e == 0x33) {
            int k = 1;

            while (k) {
                if (d == 0x32 && e == 0x33) {
                    godis(0.139);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x33 && e == 0x31) {
            int k = 1;

            while (k) {
                if (d == 0x33 && e == 0x31) {
                    backdis(0.278);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x33 && e == 0x32) {
            int k = 1;

            while (k) {
                if (d == 0x33 && e == 0x32) {
                    backdis(0.139);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        rise(0.176);
        HAL_Delay(600);
        motor_state = STATE_MOVING;
        break;

    case 42:
        move_state = STATE_ROBOARM;

        if (e == 0x31 && f == 0x32) {
            int k = 1;

            while (k) {
                if (e == 0x31 && f == 0x32) {
                    godis(0.139);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (e == 0x31 && f == 0x33) {
            int k = 1;

            while (k) {
                if (e == 0x31 && f == 0x33) {
                    godis(0.278);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (e == 0x32 && f == 0x31) {
            int k = 1;

            while (k) {
                if (e == 0x32 && f == 0x31) {
                    backdis(0.139);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (e == 0x32 && f == 0x33) {
            int k = 1;

            while (k) {
                if (e == 0x32 && f == 0x33) {
                    godis(0.138);
                    fangzhi(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (e == 0x33 && f == 0x31) {
            int k = 1;

            while (k) {
                if (e == 0x33 && f == 0x31) {
                    backdis(0.275);
                    fangzhi(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (e == 0x33 && f == 0x32) {
            int k = 1;

            while (k) {
                if (e == 0x33 && f == 0x32) {
                    backdis(0.139);
                    fangzhi(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        Openmv_SendByte('b');
        motor_state = STATE_MOVING;
        break;

    case 43:
        move_state = STATE_DISTANCE;

        if (d == 0x31 && e == 0x32 && f == 0x33) {
            int k = 1;

            while (k) {
                if (d == 0x31 && e == 0x32 && f == 0x33) {
                    backdis(0.276);
                    jiaqu2(0x31);
                    godis(0.138);
                    down(0.179);
                    jiaqu2(0x32);
                    godis(0.138);
                    down(0.179);
                    jiaqu2(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x31 && e == 0x33 && f == 0x32) {
            int k = 1;

            while (k) {
                if (d == 0x31 && e == 0x33 && f == 0x32) {
                    backdis(0.138);
                    jiaqu2(0x31);
                    godis(0.276);
                    down(0.179);
                    jiaqu2(0x33);
                    backdis(0.138);
                    down(0.179);
                    jiaqu2(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x32 && e == 0x31 && f == 0x33) {
            int k = 1;

            while (k) {
                if (d == 0x32 && e == 0x31 && f == 0x33) {
                    backdis(0.138);
                    jiaqu2(0x32);
                    backdis(0.138);
                    down(0.179);
                    jiaqu2(0x31);
                    godis(0.276);
                    down(0.179);
                    jiaqu2(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x32 && e == 0x33 && f == 0x31) {
            int k = 1;

            while (k) {
                if (d == 0x32 && e == 0x33 && f == 0x31) {
                    godis(0.138);
                    jiaqu2(0x32);
                    godis(0.138);
                    down(0.179);
                    jiaqu2(0x33);
                    backdis(0.275);
                    down(0.179);
                    jiaqu2(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x33 && e == 0x31 && f == 0x32) {
            int k = 1;

            while (k) {
                if (d == 0x33 && e == 0x31 && f == 0x32) {
                    godis(0.138);
                    jiaqu2(0x33);
                    backdis(0.276);
                    down(0.179);
                    jiaqu2(0x31);
                    godis(0.138);
                    down(0.179);
                    jiaqu2(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x33 && e == 0x32 && f == 0x31) {
            int k = 1;

            while (k) {
                if (d == 0x33 && e == 0x32 && f == 0x31) {
                    godis(0.276);
                    jiaqu2(0x33);
                    backdis(0.138);
                    down(0.179);
                    jiaqu2(0x32);
                    backdis(0.138);
                    down(0.179);
                    jiaqu2(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        motor_state = STATE_MOVING;
        break;

    case 44:
        move_state = STATE_SPEED;

        if (f == 0x31) {
            left(0.08);
            HAL_Delay(600);
            set_angle = 180;
            stop_zero();
            motor_zero();
            set_x = 0.375;
            dir = 0;
        } else if (f == 0x32) {
            left(0.08);
            HAL_Delay(600);
            set_angle = 180;
            stop_zero();
            motor_zero();
            set_x = 0.51;
            dir = 0;
        } else if (f == 0x33) {
            left(0.08);
            HAL_Delay(600);
            set_angle = 180;
            stop_zero();
            motor_zero();
            set_x = 0.67;
            dir = 0;
        }

        motor_state = STATE_MOVING;
        break;

    case 45:
        move_state = STATE_ROBOARM;
        HAL_Delay(650);
        stop_zero();
        motor_zero();
        set_angle = 90;
        turn_right();
        HAL_Delay(1400);
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 46:
        move_state = STATE_SPEED;
        set_x = 0.55;
        dir = 0;
        set_angle = 90;
        motor_state = STATE_MOVING;
        break;

    case 47:
        move_state = STATE_ROBOARM;
        HAL_Delay(700);
        stop_zero();
        motor_zero();
        right(0.09);
        ZhuaZi(OPEN);
        YunTai(OUT);
        HAL_Delay(500);
        set_angle = 90;
        stop_zero();
        Openmv_SendByte('a');
        HAL_Delay(200);
        Openmv_SendByte(7);
        HAL_Delay(10);
        jiaozhun5(165, 133);
        stop_zero();
        Openmv_SendByte('b');
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 48:
        move_state = STATE_ROBOARM;

        if (d == 0x33) {
            int k = 1;

            while (k) {
                if (d == 0x33) {
                    godis(0.139);
                    fangzhi7(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x32) {
            int k = 1;

            while (k) {
                if (d == 0x32) {
                    godis(0);
                    fangzhi7(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x31) {
            int k = 1;

            while (k) {
                if (d == 0x31) {
                    backdis(0.139);
                    fangzhi7(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        motor_state = STATE_MOVING;
        break;

    case 49://?????????
        move_state = STATE_ROBOARM;

        if (d == 0x31 && e == 0x32) {
            int k = 1;

            while (k) {
                if (d == 0x31 && e == 0x32) {
                    godis(0.139);
                    fangzhi7(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x31 && e == 0x33) {
            int k = 1;

            while (k) {
                if (d == 0x31 && e == 0x33) {
                    godis(0.278);
                    fangzhi7(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x32 && e == 0x31) {
            int k = 1;

            while (k) {
                if (d == 0x32 && e == 0x31) {
                    backdis(0.139);
                    fangzhi7(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x32 && e == 0x33) {
            int k = 1;

            while (k) {
                if (d == 0x32 && e == 0x33) {
                    godis(0.139);
                    fangzhi7(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x33 && e == 0x31) {
            int k = 1;

            while (k) {
                if (d == 0x33 && e == 0x31) {
                    backdis(0.278);
                    fangzhi7(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (d == 0x33 && e == 0x32) {
            int k = 1;

            while (k) {
                if (d == 0x33 && e == 0x32) {
                    backdis(0.139);
                    fangzhi7(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        motor_state = STATE_MOVING;
        break;

    case 50:
        move_state = STATE_ROBOARM;

        if (e == 0x31 && f == 0x32) {
            int k = 1;

            while (k) {
                if (e == 0x31 && f == 0x32) {
                    godis(0.139);
                    fangzhi7(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        if (e == 0x31 && f == 0x33) {
            int k = 1;

            while (k) {
                if (e == 0x31 && f == 0x33) {
                    godis(0.278);;
                    fangzhi7(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (e == 0x32 && f == 0x31) {
            int k = 1;

            while (k) {
                if (e == 0x32 && f == 0x31) {
                    backdis(0.139);
                    fangzhi7(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (e == 0x32 && f == 0x33) {
            int k = 1;

            while (k) {
                if (e == 0x32 && f == 0x33) {
                    godis(0.138);
                    fangzhi7(0x33);
                    k--;
                    flag = 1;
                }
            }
        }

        if (e == 0x33 && f == 0x31) {
            int k = 1;

            while (k) {
                if (e == 0x33 && f == 0x31) {
                    backdis(0.278);
                    fangzhi7(0x31);
                    k--;
                    flag = 1;
                }
            }
        }

        if (e == 0x33 && f == 0x32) {
            int k = 1;

            while (k) {
                if (e == 0x33 && f == 0x32) {
                    backdis(0.139);
                    fangzhi7(0x32);
                    k--;
                    flag = 1;
                }
            }
        }

        Openmv_SendByte('b');
        motor_state = STATE_MOVING;
        YunTai(IN);
        ZhuaZi(GRAB);
        break;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////?????//////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    case 51:
        move_state = STATE_SPEED;

        if (f == 0x31) {
            left(0.09);
            HAL_Delay(600);
            set_angle = 90;
            stop_zero();
            motor_zero();
            set_x = 0.45;
            dir = 0;
        } else if (f == 0x32) {
            left(0.09);
            HAL_Delay(600);
            set_angle = 90;
            stop_zero();
            motor_zero();
            set_x = 0.61;
            dir = 0;
            //set_angle=90;
        } else if (f == 0x33) {
            left(0.09);
            HAL_Delay(600);
            set_angle = 90;
            stop_zero();
            motor_zero();
            set_x = 0.75;
            dir = 0;
        }

        motor_state = STATE_MOVING;
        break;

    case 52:
        move_state = STATE_ROBOARM;
        HAL_Delay(600);
        stop_zero();
        motor_zero();
        set_angle = 0;
        turn_right();
        HAL_Delay(1400);
        stop_zero();
        motor_zero();
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    case 53:
        move_state = STATE_SPEED;
        set_x = 1.48;
        dir = 0;
        set_angle = 0;
        motor_state = STATE_MOVING;
        break;

    case 54:
        move_state = STATE_SPEED;
        HAL_Delay(800);
        stop_zero();
        motor_zero();
        right(0.175);
        HAL_Delay(800);
        flag = 1;
        motor_state = STATE_MOVING;
        break;

    default:
        break;
    }
}