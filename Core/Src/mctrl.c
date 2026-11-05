#include "mctrl.h"
#include "motor.h"
#include "math.h"
#include "main.h"
float a11 = 0.105; //车长21的一半
float b11 = 0.1075; //车宽21.5的一半
float r = 0.2293; //轮子的周长
float speed_cal[4] = {0, 0, 0, 0};
double vx = 0;
double vy = 0;
double w = 0;
double set_vx = 0;
double set_vy = 0;
double set_w = 0;
double w_write = 0;
//float ppi = 0;
/*
float v1_speed=0;
float v2_speed=0;
float v3_speed=0;
float v4_speed=0;
float v1_distance=0;
float v2_distance=0;
float v3_distance=0;
float v4_distance=0;
*/
typedef struct {
    float speed;
    float distance;
    float setdistance;
} chassis;
chassis v1 = {0, 0, 0};
chassis v2 = {0, 0, 0};
chassis v3 = {0, 0, 0};
chassis v4 = {0, 0, 0};
/*
typedef union
{
	float cmd;
	uint16_t data;
}Dataa;
*/

typedef union {
    float f;
    uint32_t u;
} FloatUnion;
void split_float(float num, char *sign, float *value)
{
    FloatUnion fu;
    fu.f = num;
    *sign = (fu.u >> 31) & 0x01;
    *value = fabsf(num);
}
void chassis_xy(float v_set, float angle, float target_omega)
{
    //v_set单位为每秒多少转
    char dirsign = 0; //方向标志位
    float value = 0; //提取后的数据
    uint16_t value1 = 0; //转换后的数据
    const double pi = acos(-1.0);
    float target_x = v_set * 60 * cos(angle * pi / 180);
    float target_y = v_set * 60 * sin(angle * pi / 180); //cos写入的值为弧度
    /*


    speed_cal[0]=target_y-target_x+target_omega*(a+b);//使得v=1时，cal为60
    speed_cal[1]=target_y+target_x-target_omega*(a+b);//解决float负数的问题
    speed_cal[2]=target_y-target_x-target_omega*(a+b);
    speed_cal[3]=target_y+target_x+target_omega*(a+b);
    */
    //求得每个轮子的转速
    speed_cal[0] = target_x - target_y - target_omega * 60 * (a11 + b11); //使得v=1时，cal为60
    speed_cal[1] = -(target_x + target_y + target_omega * 60 * (a11 + b11)); //解决float负数的问题
    speed_cal[2] = target_x + target_y - target_omega * 60 * (a11 + b11);
    speed_cal[3] = -(target_x - target_y + target_omega * 60 * (a11 + b11));

    for (unsigned char i = 0; i < 4; i++) {
        split_float(speed_cal[i], &dirsign, &value);
        value1 = (uint16_t)value;
        //num.cmd=speed_cal[0];
        //motor_speed(i+1,0x01,value1,0x00,0x01);
        motor_speed(i + 1, dirsign, value1, 0xB4, 0x01);
        //	motor_readspeed(i+1);//放到sync后面,提升读取实时性
    }

    motor_sync();
}
void chassis_xydistance(float distance, float angle, float target_omega, uint8_t accel, float speed_rps)
{
    //distance为需要走的距离
    //target_omega原本为角速度，此处需要改成转过的角度，将其带入公式，可得出单个轮子转过的圈数，得出圈数后，已知3200脉冲为1转，乘以3200即可
    //accel为加速度参数
    //speed_rps为电机转速，单位：转/秒
    //motor_distance(uint8_t id,uint8_t turn,uint16_t speed,uint8_t accel,uint32_t pulse,uint8_t mode,uint8_t sync);
    char dirsign = 0; //方向标志位
    float value = 0; //提取后的数据
    uint16_t speed_rpm = 0; //转换后的转速（转/分）
    const double pi = acos(-1.0);
    // 将转/秒转换为转/分
    speed_rpm = (uint16_t)(speed_rps * 60.0);
    float target_x = distance * cos(angle * pi / 180) * 3200 / 0.2293; //将距离转化为脉冲数
    float target_y = distance * sin(angle * pi / 180) * 3200 / 0.2293; //cos写入的值为弧度//删去1.414
    float n = sqrt(a11 * a11 + b11 * b11) * target_omega * sqrt(2) / (4 * pi * 0.0365);
    float omega_pulse = n * 3200;
    speed_cal[0] = target_x - target_y - omega_pulse;
    speed_cal[1] = -(target_x + target_y + omega_pulse); //解决float负数的问题
    speed_cal[2] = target_x + target_y - omega_pulse; //
    speed_cal[3] = -(target_x - target_y +
                     omega_pulse); //此部分的自转还没有解决，target_omega需要设置对应的车身旋转距离，涉及车长

    for (unsigned char i = 0; i < 4; i++) {
        split_float(speed_cal[i], &dirsign, &value);
        motor_distance(i + 1, dirsign, speed_rpm, accel, value, 0x00, 0x01);
        //motor_state(i+1);
        //motor_readdistance(i+1);
        //motor_read_setdistance(i+1);
    }

    motor_sync();
    //3200脉冲为1转，一转走0.2293m
    //过程为，设定距离，距离/0.2293，让该值乘以3200，将得出的值转为uint32_t的数据类型
    //同时，完成以上内容时，首先需要先区分距离的正负，将距离分成正负标志位与数据位
    //沿着主轴方向，转一圈距离等于周长，横向移动一个轮子周长的距离时，轮子需要转动1.414圈
}

void chassis_xyspeed_read()
{
    // 先转换为带符号的速度值
    float v1_signed, v2_signed, v3_signed, v4_signed;
    v1_signed = (bj1.dirsign == 1) ? -(float)bj1.speed : (float)bj1.speed;
    v2_signed = (bj2.dirsign == 0) ? -(float)bj2.speed : (float)bj2.speed;
    v3_signed = (bj3.dirsign == 1) ? -(float)bj3.speed : (float)bj3.speed;
    v4_signed = (bj4.dirsign == 0) ? -(float)bj4.speed : (float)bj4.speed;
    // 保存到结构体（用于调试）
    v1.speed = v1_signed;
    v2.speed = v2_signed;
    v3.speed = v3_signed;
    v4.speed = v4_signed;
    // 计算车体速度
    vx = fabs((v4_signed + v3_signed) / 120.0);
    vy = fabs((v3_signed - v1_signed) / 120.0);
    w = fabs((v2_signed - v3_signed) / (0.425 * 60.0));
}
void chassis_xydistance_read()
{
    // 先转换为带符号的距离值
    double v1_dist_signed, v2_dist_signed, v3_dist_signed, v4_dist_signed;
    double v1_setdist_signed, v2_setdist_signed, v3_setdist_signed, v4_setdist_signed;
    // 实际距离转换
    v1_dist_signed = (bj1.dirsign == 1) ? -(double)bj1.distance : (double)bj1.distance;
    v2_dist_signed = (bj2.dirsign == 0) ? -(double)bj2.distance : (double)bj2.distance;
    v3_dist_signed = (bj3.dirsign == 1) ? -(double)bj3.distance : (double)bj3.distance;
    v4_dist_signed = (bj4.dirsign == 0) ? -(double)bj4.distance : (double)bj4.distance;
    // 设定距离转换
    v1_setdist_signed = (bj1.dirsign == 1) ? -(double)bj1.setdistance : (double)bj1.setdistance;
    v2_setdist_signed = (bj2.dirsign == 0) ? -(double)bj2.setdistance : (double)bj2.setdistance;
    v3_setdist_signed = (bj3.dirsign == 1) ? -(double)bj3.setdistance : (double)bj3.setdistance;
    v4_setdist_signed = (bj4.dirsign == 0) ? -(double)bj4.setdistance : (double)bj4.setdistance;
    // 保存到结构体（用于调试）
    v1.distance = v1_dist_signed;
    v2.distance = v2_dist_signed;
    v3.distance = v3_dist_signed;
    v4.distance = v4_dist_signed;
    v1.setdistance = v1_setdist_signed;
    v2.setdistance = v2_setdist_signed;
    v3.setdistance = v3_setdist_signed;
    v4.setdistance = v4_setdist_signed;
    // 计算车体实际位移
    vx = fabs((v4_dist_signed + v3_dist_signed) / (65536.0 * 2.0) * 0.2293);
    vy = fabs((v3_dist_signed - v1_dist_signed) / (65536.0 * 2.0) * 0.2293);
    w = fabs((v2_dist_signed - v3_dist_signed) / (0.425 * 65536.0) * 0.2293);
    // 计算车体设定位移
    set_vx = fabs((v4_setdist_signed + v3_setdist_signed) / (65536.0 * 2.0) * 0.2293);
    set_vy = fabs((v3_setdist_signed - v1_setdist_signed) / (65536.0 * 2.0) * 0.2293);
    set_w = fabs((v2_setdist_signed - v3_setdist_signed) / (0.425 * 65536.0) * 0.2293);
}
void chassis_xydistance_error()
{
    w_write = set_w - w;
    chassis_xydistance(0, 0, w_write, 0x0A, 1.0); // 默认加速度0x0A，速度1转/秒
}

