#include "pid.h"
#include "jy61p.h"

tPid pid_omega;
tPid pid_omega1;
tPid pid_omega2;

extern float set_angle;

void PID_init()
{
pid_omega.actual_val=0;
pid_omega.target_val=set_angle;
pid_omega.err=0.0;
pid_omega.err_last=0.0;
pid_omega.err_sum=0.0;
pid_omega.Kp=0.2;
pid_omega.Ki=0.0;
pid_omega.Kd=0.0;
	
	
pid_omega1.actual_val=0;
pid_omega1.target_val=0;
pid_omega1.err=0.0;
pid_omega1.err_last=0.0;
pid_omega1.err_sum=0.0;
pid_omega1.Kp=0.03; 
pid_omega1.Ki=0.0;
pid_omega1.Kd=0.01; 
	
pid_omega2.actual_val=0;
pid_omega2.target_val=0;
pid_omega2.err=0.0;
pid_omega2.err_last=0.0;
pid_omega2.err_sum=0.0;
pid_omega2.Kp=0.08;  
pid_omega2.Ki=0.0;
pid_omega2.Kd=0.08;  

}

float PID_realize(tPid * pid,float actual_val)
{
pid->actual_val = actual_val;
pid->err = pid->target_val - pid->actual_val;
pid->err_sum += pid->err;

pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);

pid->err_last = pid->err;
return pid->actual_val;
}