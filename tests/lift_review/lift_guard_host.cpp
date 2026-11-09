/* Manual lift supervision exercised with the production web dispatcher and
 * the production length-based motor feedback parser, without real actuators. */
#define __MAIN_H
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <vector>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
typedef struct { int instance; } UART_HandleTypeDef;
typedef struct { int instance; } TIM_HandleTypeDef;
UART_HandleTypeDef huart1={1}, huart4={4}, huart6={6};
TIM_HandleTypeDef htim1={1};
#define TIM_CHANNEL_1 1U
#define TIM_CHANNEL_2 2U
#define TIM_CHANNEL_3 3U
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)0)
#define __DMB() ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(x) ((void)0)
int start, zero;
volatile uint8_t vision_follow_enable;
float Yaw, Roll, Pitch;
static uint32_t clock_ms;
static double motor_position[5], motor_rpm[5];
static bool feedback_available, jammed, motor_uart_ok;
static bool lift_pending, lift_moving, wrong_direction;
static double lift_goal, lift_requested_rpm;
static double lift_rate_scale=1.0;
static uint32_t last_lift_pulses;
static unsigned nonzero_lift_commands, stop_lift_commands;
static unsigned tray_commands;
static uint8_t *receiver;
uint32_t HAL_GetTick(void) { return clock_ms; }
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *) { return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t) { return HAL_OK; }
HAL_StatusTypeDef Duoji_StartPWM(uint32_t) { return HAL_OK; }
void ZhuaZi(uint16_t) {}
void YunTai(uint16_t) {}
void ZhuanPan(float) {}
#include "../../Core/Src/logistics_task.c"
#include "../../Core/Src/web_control.c"

void RobotControl_ReadStatus(RobotStatus *status) {
    status->state=logistics_state; status->drop_index=logistics_drop_index;
    status->error=logistics_error; status->running=Logistics_IsRunning();
    status->plan_ready=Logistics_PlanReady(); status->x_m=logistics_x_m; status->y_m=logistics_y_m;
}
uint8_t RobotControl_IsAutomatic(void) { return Logistics_IsRunning(); }
uint8_t RobotControl_SetPlan(const uint8_t *items,uint8_t length) {
    return Logistics_SetPlan(items,length) ? WEB_RESULT_OK : WEB_RESULT_BAD_ARGUMENT;
}
uint8_t RobotControl_Start(const uint8_t *items,uint8_t length) {
    if (!Logistics_PlanMatches(items,length)) return WEB_RESULT_INTERLOCK;
    return Logistics_Start() ? WEB_RESULT_OK : WEB_RESULT_INTERLOCK;
}
uint8_t RobotControl_Stop(void) { Logistics_Cancel(); return WEB_RESULT_OK; }
uint8_t RobotControl_GetLiftFeedback(int64_t *count,uint32_t *tick,uint32_t *serial) {
    return Logistics_GetLiftFeedback(count,tick,serial);
}
void MotorBus_Begin(void) {}
void MotorBus_End(void) {}
uint8_t MotorBus_Transmit(const uint8_t *data,uint16_t size,uint32_t gap_ms) {
    uint8_t ok=HAL_UART_Transmit(&huart1,(uint8_t *)data,size,10U)==HAL_OK;
    HAL_Delay(gap_ms);
    return ok;
}
/* Simulate two independently scheduled tasks. */
static void service_tasks(void) { Logistics_Task(); WebControl_Task(); }

HAL_StatusTypeDef JiaZhua_SetAngle(uint16_t) { return HAL_OK; }
HAL_StatusTypeDef Tray_SetAngle(uint16_t) { ++tray_commands; return HAL_OK; }
HAL_StatusTypeDef Duoji_SetPresets(uint16_t,uint16_t,uint16_t,uint16_t) { return HAL_OK; }
uint16_t Duoji_GetClawPreset(uint8_t open) { return open ? OPEN : GRAB; }
uint16_t Duoji_GetGimbalPreset(uint8_t extended) { return extended ? OUT : IN; }
uint8_t JY61P_HasFreshYaw(uint32_t) { return 1U; }
uint8_t VisionObserve_Start(uint8_t) { return 1U; }
uint8_t VisionObserve_StartTarget(uint8_t,uint8_t) { return 1U; }
uint8_t VisionObserve_Stop(void) { return 1U; }
uint8_t VisionObserve_Read(VisionObservation *,uint32_t) { return 0U; }
static std::vector<WebFrame> replies;
static void advance(uint32_t ms) {
    if (!jammed) {
        for (unsigned i=0;i<4;++i)
            motor_position[i]+=motor_rpm[i]*LOG_ENCODER_COUNTS_REV*ms/60000.0;
        if (lift_moving) {
            double step=lift_requested_rpm*lift_rate_scale*LOG_ENCODER_COUNTS_REV*ms/60000.0;
            if (!wrong_direction && fabs(lift_goal-motor_position[4])<=step) {
                motor_position[4]=lift_goal; lift_moving=false; motor_rpm[4]=0;
            } else {
                double sign=lift_goal>motor_position[4] ? 1.0 : -1.0;
                motor_position[4]+=step*sign*(wrong_direction ? -1.0 : 1.0);
            }
        }
    }
    clock_ms+=ms;
}
void HAL_Delay(uint32_t ms) { advance(ms); }
static void feedback(unsigned id) {
    int64_t count=(int64_t)motor_position[id-1];
    uint32_t magnitude=(uint32_t)(count<0 ? -count : count);
    uint8_t bytes[8]={(uint8_t)id,0x36,(uint8_t)(count<0),
        (uint8_t)(magnitude>>24),(uint8_t)(magnitude>>16),
        (uint8_t)(magnitude>>8),(uint8_t)magnitude,0x6B};
    for (unsigned i=0;i<8;++i) Logistics_MotorRxByteISR(bytes[i]);
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart,uint8_t *bytes,
                                  uint16_t n,uint32_t timeout) {
    assert(timeout<=10U);
    if (uart==&huart4) {
        assert(n>=13U);
        WebFrame frame={};
        frame.session=Web_Get32(bytes+4); frame.sequence=Web_Get16(bytes+8);
        frame.command=bytes[10]; frame.length=bytes[3];
        memcpy(frame.payload,bytes+11,frame.length); replies.push_back(frame);
        return HAL_OK;
    }
    if (uart!=&huart1) return HAL_OK;
    if (!motor_uart_ok) return 1;
    if (n==3U && bytes[1]==0x36U) {
        advance(1U); /* Model a reply newer than the caller's entry timestamp. */
        if (feedback_available) feedback(bytes[0]);
    } else if (n==5U && bytes[1]==0xFEU) {
        assert(bytes[2]==0x98U && bytes[3]==0U && bytes[4]==0x6BU);
        motor_rpm[bytes[0]-1]=0;
        if (bytes[0]==5U) {
            ++stop_lift_commands; lift_pending=lift_moving=false;
        }
    } else if (n==8U && bytes[1]==0xF6U) {
        unsigned rpm=((unsigned)bytes[3]<<8)|bytes[4];
        motor_rpm[bytes[0]-1]=(bytes[2] ? -1.0 : 1.0)*rpm;
        /* Manual lift must use finite relative position instead of velocity. */
        assert(bytes[0]!=5U || rpm==0U);
    } else if (n==13U && bytes[1]==0xFDU) {
        assert(bytes[0]==5U && bytes[10]==0U && bytes[11]==1U && bytes[12]==0x6BU);
        unsigned rpm=((unsigned)bytes[3]<<8)|bytes[4];
        assert(rpm==WEB_LIFT_RPM && rpm==15U);
        last_lift_pulses=((uint32_t)bytes[6]<<24)|((uint32_t)bytes[7]<<16)|
                         ((uint32_t)bytes[8]<<8)|bytes[9];
        uint32_t expected=(uint32_t)(WEB_LIFT_STEP_M*LOG_LIFT_PULSES_METRE+0.5f);
        assert(last_lift_pulses==expected && last_lift_pulses>0U);
        lift_goal=motor_position[4]+(bytes[2] ? -1.0 : 1.0)*
                  last_lift_pulses*LOG_ENCODER_COUNTS_REV/3200.0;
        lift_requested_rpm=rpm; lift_pending=true; ++nonzero_lift_commands;
    } else if (n==4U) {
        assert(bytes[0]==0 && bytes[1]==0xFFU && bytes[2]==0x66U);
        if (lift_pending) {
            lift_moving=true; lift_pending=false;
            motor_rpm[4]=(lift_goal>motor_position[4] ? 1.0 : -1.0)*lift_requested_rpm;
        }
    } else assert(false);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,uint8_t *byte,uint16_t n) {
    assert(uart==&huart4 && n==1U); receiver=byte; return HAL_OK;
}
static uint8_t result(void) {
    for (size_t i=replies.size();i>0;--i)
        if (replies[i-1].command==WEB_CMD_ACK) return replies[i-1].payload[1];
    return 255U;
}
static void request(uint8_t command,uint16_t sequence,const int8_t *payload=NULL,uint8_t length=0U) {
    WebFrame frame={}; frame.session=1U; frame.sequence=sequence;
    frame.command=command; frame.length=length;
    if (length) memcpy(frame.payload,payload,length);
    uint8_t wire[WEB_MAX_FRAME]; uint8_t count=Web_Encode(&frame,wire);
    for (unsigned i=0;i<count;++i) {
        *receiver=wire[i]; WebControl_RxCompleteISR(); advance(1U);
    }
    service_tasks();
}
static void reset(bool initial_feedback=true) {
    clock_ms=1U; start=zero=0; vision_follow_enable=0;
    feedback_available=initial_feedback; jammed=wrong_direction=false; motor_uart_ok=true;
    lift_pending=lift_moving=false; lift_goal=lift_requested_rpm=0; last_lift_pulses=0;
    lift_rate_scale=1.0;
    memset(motor_position,0,sizeof(motor_position)); memset(motor_rpm,0,sizeof(motor_rpm));
    Logistics_Init(); WebControl_Init(); nonzero_lift_commands=stop_lift_commands=0;
    tray_commands=0U;
    replies.clear();
    request(WEB_CMD_HELLO,1U);
    if (initial_feedback) {
        /* Poll all five axes, exercising the real receive parser. */
        for (unsigned i=0;i<18;++i) { advance(10U); service_tasks(); }
    }
    int8_t manual=WEB_MODE_MANUAL; request(WEB_CMD_MODE,2U,&manual,1U);
    assert(mode==WEB_MODE_MANUAL);
}
static void jog_lift(int8_t direction,uint16_t sequence) {
    const int8_t command[4]={0,0,0,direction};
    request(WEB_CMD_JOG,sequence,command,4U);
    if (motion_dirty) { advance(WEB_MOTOR_PERIOD_MS); service_tasks(); }
}
static void hold(unsigned duration_ms,int8_t direction=1) {
    uint32_t began=clock_ms; uint16_t sequence=4U;
    while ((uint32_t)(clock_ms-began)<duration_ms && mode==WEB_MODE_MANUAL) {
        advance(50U); jog_lift(direction,sequence++);
    }
}
static void tests(void) {
    reset(false); jog_lift(1,3U);
    assert(nonzero_lift_commands==0U && motor_rpm[4]==0 && !jog_active);
    assert(result()==WEB_RESULT_INTERLOCK);

    reset(); jog_lift(1,3U);
    assert(result()==WEB_RESULT_OK && jog_active && motor_rpm[4]>0);
    assert(nonzero_lift_commands==1U);
    hold(120U); assert(nonzero_lift_commands==1U); /* Lease renewal is not reissue. */
    jog_lift(0,20U); assert(!jog_active && motor_rpm[4]==0 && stop_lift_commands>0);

    /* The new tray path cannot move during real, feedback-driven lift motion. */
    reset(); jog_lift(1,3U);
    uint8_t tray_angle[2]; Web_Put16(tray_angle,135U);
    request(WEB_CMD_TRAY_ANGLE,4U,(const int8_t *)tray_angle,2U);
    assert(result()==WEB_RESULT_INTERLOCK && tray_commands==0U && manual_lift_active);
    jog_lift(0,5U);
    request(WEB_CMD_TRAY_ANGLE,6U,(const int8_t *)tray_angle,2U);
    assert(result()==WEB_RESULT_OK && tray_commands==1U && tray_target==135U);

    reset(); jog_lift(-1,3U);
    assert(result()==WEB_RESULT_OK && motor_rpm[4]<0);
    request(WEB_CMD_STOP,4U); assert(mode==WEB_MODE_IDLE && motor_rpm[4]==0);

    reset(); jog_lift(1,3U); uint32_t lease=jog_tick;
    clock_ms=lease+WEB_JOG_LEASE_MS; service_tasks();
    assert(mode==WEB_MODE_IDLE && !jog_active && motor_rpm[4]==0 && fault==WEB_FAULT_JOG_TIMEOUT);

    reset(); jog_lift(1,3U); lease=link_tick;
    clock_ms=lease+WEB_LINK_LEASE_MS; service_tasks();
    assert(mode==WEB_MODE_IDLE && !session_valid && motor_rpm[4]==0 && fault==WEB_FAULT_LINK_TIMEOUT);

    reset(); jammed=true; jog_lift(1,3U); hold(3000U);
    assert(mode==WEB_MODE_IDLE && !jog_active && motor_rpm[4]==0 && fault!=WEB_FAULT_NONE);
    unsigned commands=nonzero_lift_commands;
    jog_lift(1,100U); assert(nonzero_lift_commands==commands && mode==WEB_MODE_IDLE);

    reset(); jog_lift(1,3U); feedback_available=false; hold(3000U);
    assert(mode==WEB_MODE_IDLE && !jog_active && motor_rpm[4]==0 && fault!=WEB_FAULT_NONE);

    reset(); jog_lift(1,3U); hold(3000U);
    assert(mode==WEB_MODE_MANUAL && !jog_active && motor_rpm[4]==0 && fault==WEB_FAULT_NONE);
    assert(nonzero_lift_commands==1U); /* Fresh JOG frames cannot reset the stroke cap. */
    assert(fabs(motor_position[4]*LOG_LIFT_METRES_REV/LOG_ENCODER_COUNTS_REV-WEB_LIFT_STEP_M)
           <=WEB_LIFT_STEP_TOLERANCE_M);
    jog_lift(0,100U); assert(mode==WEB_MODE_MANUAL);
    jog_lift(1,101U); assert(nonzero_lift_commands==2U && motor_rpm[4]>0);

    reset(); jog_lift(1,3U); jog_lift(-1,4U);
    assert(mode==WEB_MODE_IDLE && !jog_active && motor_rpm[4]==0 && result()==WEB_RESULT_INTERLOCK);

    reset(); wrong_direction=true; jog_lift(1,3U); hold(2000U);
    assert(mode==WEB_MODE_IDLE && !jog_active && motor_rpm[4]==0 && fault!=WEB_FAULT_NONE);

    /* Slow valid progress keeps no-progress supervision alive but must still
     * reach the independent, non-renewable one-stroke time limit. */
    reset(); lift_rate_scale=0.2; jog_lift(1,3U); hold(3000U);
    assert(mode==WEB_MODE_IDLE && !jog_active && motor_rpm[4]==0 && fault==10U);

    /* A stale snapshot cannot be sent after a valid JOG waits for motor I/O. */
    reset(); motor_tick=clock_ms;
    const int8_t up[4]={0,0,0,1}; request(WEB_CMD_JOG,3U,up,4U);
    assert(motion_dirty && nonzero_lift_commands==0U);
    advance(WEB_MOTOR_PERIOD_MS);
    feedback_available=false;
    encoders[4].tick=clock_ms-WEB_LIFT_FEEDBACK_TIMEOUT_MS-1U;
    service_tasks();
    assert(nonzero_lift_commands==0U && !jog_active && result()==WEB_RESULT_INTERLOCK);

    reset(); motor_uart_ok=false; jog_lift(1,3U);
    assert(mode==WEB_MODE_IDLE && !jog_active && fault==WEB_FAULT_MOTOR_UART);

    /* Query-copy API uses signed counts and does not split on payload 0x6B. */
    reset(); motor_position[4]=-0x6B; feedback(5U);
    int64_t count=0; uint32_t tick=0, serial=0;
    assert(Logistics_GetLiftFeedback(&count,&tick,&serial));
    assert(count==-0x6B && serial>0U && tick==clock_ms);

    printf("PASS: finite 5mm lift parser/dispatch, no/stale feedback interlock, low-speed directions, release, JOG/link leases, jam/direction/lost-feedback faults, press latch, UART failure\n");
}
int main(void) { tests(); return 0; }
