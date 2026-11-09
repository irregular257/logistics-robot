/* Execute the production state machine with a simulated Emm motor bus. */
#define __MAIN_H
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
typedef int HAL_StatusTypeDef;
#define HAL_OK 0
typedef struct { int port; } UART_HandleTypeDef;
typedef struct { int port; } TIM_HandleTypeDef;
UART_HandleTypeDef huart1={1},huart6={6};
TIM_HandleTypeDef htim1={1};
#define TIM_CHANNEL_1 1U
#define TIM_CHANNEL_2 2U
#define TIM_CHANNEL_3 3U
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)0)
static uint32_t clock_ms;
float Yaw,Roll,Pitch;
static bool yaw_ok=true, motor_ok=true, feedback_ok=true, vision_ok=true;
static double position[5], rpm[5], lift_goal, true_x,true_y;
static bool lift_active, lift_jammed;
static unsigned drops, stores, fetches, commands, max_motor_timeout, started_observing;
static bool tray[3], holding, arm_out;
static unsigned selected_slot;
static float drop_x[3],drop_y[3];
static bool opened, observing;
static uint16_t claw_close_preset, claw_open_preset, gimbal_in_preset, gimbal_out_preset;
static uint16_t last_claw_angle, last_gimbal_angle;
static unsigned gimbal_folds;
static const uint8_t expected_plan[6]={4U,1U,7U,4U,3U,3U};
static uint8_t requested_targets[6], observed_color, observed_shape;
static uint8_t wrong_color, wrong_shape;
uint32_t HAL_GetTick(void) { return clock_ms; }
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
void MotorBus_Begin(void) {}
void MotorBus_End(void) {}
uint8_t MotorBus_Transmit(const uint8_t *data, uint16_t size, uint32_t gap_ms) {
    uint8_t ok = HAL_UART_Transmit(&huart1, (uint8_t *)data, size, 10U) == HAL_OK;
    HAL_Delay(gap_ms);
    return ok;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t) { return HAL_OK; }
HAL_StatusTypeDef Duoji_StartPWM(uint32_t) { return HAL_OK; }
void ZhuaZi(uint16_t);
void YunTai(uint16_t);
void ZhuanPan(float);
#include "../Core/Src/logistics_task.c"

uint16_t Duoji_GetClawPreset(uint8_t open) {
    return open ? claw_open_preset : claw_close_preset;
}
uint16_t Duoji_GetGimbalPreset(uint8_t extended) {
    return extended ? gimbal_out_preset : gimbal_in_preset;
}
uint8_t JY61P_HasFreshYaw(uint32_t) { return yaw_ok; }
uint8_t VisionObserve_StartTarget(uint8_t color,uint8_t shape) {
    assert(started_observing<3U);
    assert(color==expected_plan[2U*started_observing]);
    assert(shape==expected_plan[2U*started_observing+1U]);
    requested_targets[2U*started_observing]=color;
    requested_targets[2U*started_observing+1U]=shape;
    observed_color=color; observed_shape=shape;
    observing=true; ++started_observing; return 1;
}
uint8_t VisionObserve_Stop(void) { observing=false; return 1; }
uint8_t VisionObserve_Read(VisionObservation *sample,uint32_t) {
    if (!observing || !vision_ok) return 0;
    sample->x=(uint16_t)LOG_ALIGN_TARGET_X; sample->y=(uint16_t)LOG_ALIGN_TARGET_Y;
    sample->color=wrong_color ? wrong_color : observed_color;
    sample->shape=wrong_shape ? wrong_shape : observed_shape;
    sample->tick=clock_ms;
    return 1;
}
void YunTai(uint16_t angle) {
    assert(angle==gimbal_in_preset || angle==gimbal_out_preset);
    last_gimbal_angle=angle;
    arm_out=angle==gimbal_out_preset;
    if (!arm_out) ++gimbal_folds;
}
void ZhuanPan(float angle) {
    if (angle==OUT1) return;
    const float slots[]={HONG,LV,LAN};
    for (unsigned i=0;i<3;++i) if (angle==slots[i]) { selected_slot=i; return; }
    assert(false);
}
void ZhuaZi(uint16_t angle) {
    assert(angle==claw_open_preset || angle==claw_close_preset);
    last_claw_angle=angle;
    opened=angle==claw_open_preset;
    if (!opened && logistics_state==LOG_PICK_LOWER) {
        assert(arm_out && !holding && stores<3);
        assert(!wrong_color && !wrong_shape);
        assert(observed_color==expected_plan[2U*stores]);
        assert(observed_shape==expected_plan[2U*stores+1U]);
        assert(fabs(true_x-LOG_PICK_X_M)<0.025 && fabs(true_y-LOG_PICK_Y_M)<0.025);
        holding=true;
    }
    if (opened && logistics_state==LOG_STORE_LOWER) {
        assert(!arm_out && holding && !tray[selected_slot]);
        assert(selected_slot==stores);
        tray[selected_slot]=true; holding=false; ++stores;
    }
    if (!opened && logistics_state==LOG_FETCH_LOWER) {
        assert(stores==3 && !arm_out && !holding && tray[selected_slot]);
        assert(selected_slot==drops);
        tray[selected_slot]=false; holding=true; ++fetches;
    }
    if (opened && logistics_state==LOG_PLACE_LOWER) {
        assert(stores==3 && arm_out && holding && fetches==drops+1);
        assert(drops<3); drop_x[drops]=(float)true_x; drop_y[drops]=(float)true_y; ++drops;
        holding=false;
    }
}
static void advance(uint32_t ms) {
    double forward=(rpm[0]-rpm[1]+rpm[2]-rpm[3])*0.25;
    double left=(-rpm[0]-rpm[1]+rpm[2]+rpm[3])*0.25;
    double metres_ms=LOG_WHEEL_CIRCUMFERENCE_M/60000.0;
    true_x-=left*metres_ms*ms; true_y+=forward*metres_ms*ms;
    for (unsigned i=0;i<4;++i) position[i]+=rpm[i]*LOG_ENCODER_COUNTS_REV*ms/60000.0;
    if (lift_active && !lift_jammed) {
        double step=LOG_LIFT_RPM*LOG_ENCODER_COUNTS_REV*ms/60000.0;
        if (fabs(lift_goal-position[4])<=step) { position[4]=lift_goal; lift_active=false; }
        else position[4]+=lift_goal>position[4] ? step : -step;
    }
    clock_ms+=ms;
}
void HAL_Delay(uint32_t ms) { advance(ms); }
static void reply_position(unsigned id) {
    int64_t count=(int64_t)position[id-1];
    uint32_t mag=(uint32_t)(count<0 ? -count : count);
    uint8_t data[8]={(uint8_t)id,0x36,(uint8_t)(count<0),
                    (uint8_t)(mag>>24),(uint8_t)(mag>>16),(uint8_t)(mag>>8),(uint8_t)mag,0x6B};
    for(unsigned i=0;i<8;++i) Logistics_MotorRxByteISR(data[i]);
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *port,uint8_t *wire,uint16_t n,uint32_t timeout) {
    assert(timeout<=10U); if(timeout>max_motor_timeout)max_motor_timeout=timeout;
    if(port!=&huart1)return HAL_OK;
    if(!motor_ok)return 1;
    ++commands;
    if(n==3 && wire[1]==0x36) {
        advance(1); /* Reply timestamp newer than the task's entry timestamp. */
        if(feedback_ok)reply_position(wire[0]);
    }
    else if(n==5 && wire[1]==0xFE) { rpm[wire[0]-1]=0; if(wire[0]==5)lift_active=false; }
    else if(n==8 && wire[1]==0xF6) {
        if (((wire[3]<<8)|wire[4]) && logistics_state==LOG_OUTBOUND)
            assert(stores==3 && started_observing==3 && !holding);
        if (logistics_state==LOG_RETURN) assert(drops==3);
        rpm[wire[0]-1]=((wire[3]<<8)|wire[4])*(wire[2]?-1.0:1.0);
    }
    else if(n==13 && wire[1]==0xFD) {
        assert(wire[0]==5 && wire[10]==0 && wire[11]==0);
        uint32_t pulses=((uint32_t)wire[6]<<24)|((uint32_t)wire[7]<<16)|((uint32_t)wire[8]<<8)|wire[9];
        lift_goal=position[4]+(wire[2]?-1.0:1.0)*pulses*LOG_ENCODER_COUNTS_REV/3200.0;
        lift_active=true;
    } else assert(n==4 && wire[1]==0xFF);
    return HAL_OK;
}
static void reset(bool configure_plan=true) {
    clock_ms=1; Yaw=0; yaw_ok=motor_ok=feedback_ok=vision_ok=true;
    memset(position,0,sizeof(position)); memset(rpm,0,sizeof(rpm));
    true_x=LOG_PICK_X_M; true_y=LOG_PICK_Y_M;
    drops=stores=fetches=commands=started_observing=0; lift_active=observing=holding=lift_jammed=false;
    memset(tray,0,sizeof(tray)); selected_slot=0;
    claw_close_preset=GRAB; claw_open_preset=OPEN;
    gimbal_in_preset=IN; gimbal_out_preset=OUT;
    last_claw_angle=last_gimbal_angle=0xffffU; gimbal_folds=0U;
    memset(requested_targets,0,sizeof(requested_targets));
    wrong_color=wrong_shape=observed_color=observed_shape=0U;
    Logistics_Init();
    assert(!Logistics_PlanReady());
    if (configure_plan) assert(Logistics_SetPlan(expected_plan,sizeof(expected_plan)));
    /* Include 0x6B as encoder payload to exercise length-based parsing. */
    position[0]=107;
    for(unsigned i=0;i<50;++i) { advance(10); Logistics_Task(); }
}
static void run_for(uint32_t ms) {
    uint32_t deadline=clock_ms+ms;
    while(clock_ms<deadline) { advance(10); Logistics_Task(); }
}
int main(void) {
    /* Never move an actuator before all three explicit targets are configured. */
    reset(false);
    unsigned commands_before_start=commands;
    assert(!Logistics_Start() && logistics_error==LOG_ERROR_PLAN);
    assert(commands==commands_before_start && !Logistics_IsRunning());
    assert(last_claw_angle==0xffffU && last_gimbal_angle==0xffffU && !observing);
    assert(!Logistics_PlanMatches(expected_plan,sizeof(expected_plan)));
    assert(!Logistics_SetPlan(NULL,6U));
    assert(!Logistics_SetPlan(expected_plan,5U));
    assert(!Logistics_SetPlan(expected_plan,7U));
    const uint8_t invalid_colors[]={0U,5U,6U,8U,255U};
    const uint8_t invalid_shapes[]={0U,2U,7U,255U};
    uint8_t candidate[6];
    for (unsigned item=0; item<3U; ++item) {
        for (unsigned i=0; i<sizeof(invalid_colors); ++i) {
            memcpy(candidate,expected_plan,sizeof(candidate));
            candidate[2U*item]=invalid_colors[i];
            assert(!Logistics_SetPlan(candidate,sizeof(candidate)));
        }
        for (unsigned i=0; i<sizeof(invalid_shapes); ++i) {
            memcpy(candidate,expected_plan,sizeof(candidate));
            candidate[2U*item+1U]=invalid_shapes[i];
            assert(!Logistics_SetPlan(candidate,sizeof(candidate)));
        }
    }
    assert(!Logistics_PlanReady());
    const uint8_t missing_combinations[][2]={{7U,3U},{1U,1U},{3U,4U},{4U,4U}};
    for (unsigned item=0; item<3U; ++item) {
        for (unsigned i=0; i<sizeof(missing_combinations)/sizeof(missing_combinations[0]); ++i) {
            memcpy(candidate,expected_plan,sizeof(candidate));
            candidate[2U*item]=missing_combinations[i][0];
            candidate[2U*item+1U]=missing_combinations[i][1];
            assert(!Logistics_SetPlan(candidate,sizeof(candidate)));
        }
    }
    assert(!Logistics_PlanReady());
    const uint8_t alternate_plan[6]={1U,5U,1U,6U,2U,4U};
    assert(Logistics_SetPlan(alternate_plan,sizeof(alternate_plan)));
    assert(Logistics_PlanReady() && Logistics_PlanMatches(alternate_plan,sizeof(alternate_plan)));
    assert(!Logistics_PlanMatches(NULL,6U));
    assert(!Logistics_PlanMatches(alternate_plan,5U));
    assert(!Logistics_SetPlan(candidate,sizeof(candidate)));
    assert(Logistics_PlanMatches(alternate_plan,sizeof(alternate_plan)));
    assert(Logistics_SetPlan(expected_plan,sizeof(expected_plan)));
    assert(Logistics_PlanMatches(expected_plan,sizeof(expected_plan)));
    assert(!Logistics_PlanMatches(alternate_plan,sizeof(alternate_plan)));

    reset(); assert(!Logistics_IsRunning()); assert(Logistics_Start());
    assert(!Logistics_Start());
    assert(!Logistics_SetPlan(alternate_plan,sizeof(alternate_plan)));
    assert(!Logistics_SetPlan(expected_plan,sizeof(expected_plan)));
    assert(Logistics_PlanMatches(expected_plan,sizeof(expected_plan)));
    run_for(200000);
    assert(logistics_state==LOG_DONE && logistics_error==0 && drops==3);
    assert(started_observing==3);
    assert(memcmp(requested_targets,expected_plan,sizeof(expected_plan))==0);
    assert(stores==3 && fetches==3 && !holding && !tray[0] && !tray[1] && !tray[2]);
    assert(fabs(drop_x[0]-0.30)<0.025 && fabs(drop_y[0]-1.70)<0.025);
    assert(fabs(drop_x[1]-1.70)<0.025 && fabs(drop_y[1]-1.70)<0.025);
    assert(fabs(drop_x[2]-1.70)<0.025 && fabs(drop_y[2]-0.30)<0.025);
    assert(fabs(true_x-0.30)<0.025 && fabs(true_y-0.30)<0.025);
    assert(!lift_active && fabs(position[4])<1200 && !observing);
    /* A centered box with the wrong color or shape must not trigger lowering.
     * Once correct samples arrive, the same plan can proceed normally. */
    reset(); assert(Logistics_Start()); wrong_shape=3U;
    run_for(4000U);
    assert(logistics_state==LOG_ALIGN && !lift_active && !holding && stores==0U);
    wrong_shape=0U; wrong_color=2U;
    run_for(4000U);
    assert(logistics_state==LOG_ALIGN && !lift_active && !holding && stores==0U);
    wrong_color=0U; run_for(200000U);
    assert(logistics_state==LOG_DONE && drops==3U);
    assert(memcmp(requested_targets,expected_plan,sizeof(expected_plan))==0);
    /* User calibration must reach the hardware commands, including folding.
     * All four values differ from macros, so a hard-coded preset will fail. */
    reset();
    claw_close_preset=33U; claw_open_preset=108U;
    gimbal_in_preset=42U; gimbal_out_preset=188U;
    assert(Logistics_Start());
    assert(last_claw_angle==108U && last_gimbal_angle==188U && arm_out);
    while (logistics_state!=LOG_FOLD && clock_ms<10000U) { advance(10); Logistics_Task(); }
    assert(logistics_state==LOG_FOLD && last_gimbal_angle==42U && !arm_out);
    assert(last_claw_angle==33U && holding && gimbal_folds==1U);
    run_for(200000U);
    assert(logistics_state==LOG_DONE && logistics_error==0U && drops==3U);
    assert(stores==3U && fetches==3U && !holding && !tray[0] && !tray[1] && !tray[2]);
    assert(gimbal_folds>=6U && last_gimbal_angle==42U && last_claw_angle==108U);
    reset(); assert(Logistics_Start()); run_for(4000); Logistics_Cancel();
    assert(logistics_state==LOG_ABORTED && !lift_active && !observing);
    assert(Logistics_PlanMatches(expected_plan,sizeof(expected_plan)));
    for(unsigned i=0;i<4;++i)assert(rpm[i]==0);
    Logistics_ClearPlan();
    assert(!Logistics_PlanReady() && !Logistics_PlanMatches(expected_plan,sizeof(expected_plan)));
    assert(!Logistics_Start() && logistics_error==LOG_ERROR_PLAN);
    reset(); assert(Logistics_Start()); run_for(4000U); Logistics_ClearPlan();
    assert(logistics_state==LOG_ABORTED && !Logistics_PlanReady() && !lift_active && !observing);
    for(unsigned i=0;i<4;++i)assert(rpm[i]==0);
    /* A new tray state must remain cancellable, including an active lift. */
    reset(); assert(Logistics_Start());
    while (logistics_state!=LOG_STORE_LOWER && clock_ms<10000) { advance(10); Logistics_Task(); }
    assert(logistics_state==LOG_STORE_LOWER && lift_active);
    Logistics_Cancel(); assert(logistics_state==LOG_ABORTED && !lift_active && !observing);
    run_for(1000); assert(stores==0 && drops==0);
    reset(); assert(Logistics_Start());
    while (logistics_state!=LOG_STORE_LOWER && clock_ms<10000) { advance(10); Logistics_Task(); }
    assert(lift_active); lift_jammed=true;
    run_for(LOG_LIFT_NO_PROGRESS_MS+400);
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_LIFT_NO_PROGRESS && !lift_active);
    for(unsigned i=0;i<4;++i)assert(rpm[i]==0);
    reset(); assert(Logistics_Start()); feedback_ok=false; run_for(1000);
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_FEEDBACK);
    reset(); yaw_ok=false; assert(!Logistics_Start()); assert(logistics_error==LOG_ERROR_IMU);
    reset(); assert(Logistics_Start()); yaw_ok=false; Logistics_Task();
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_IMU);
    reset(); assert(Logistics_Start()); motor_ok=false; run_for(100);
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_MOTOR_UART);
    reset(); assert(Logistics_Start()); vision_ok=false; run_for(23000);
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_VISION && drops==0);
    reset(); assert(Logistics_Start()); Yaw=25; Logistics_Task();
    assert(logistics_state==LOG_FAULT && logistics_error==LOG_ERROR_HEADING);
    printf("PASS: explicit validated 3-target plans, wrong-color/shape rejection, load/deliver in order, return, calibrated servos, cancellation/session clearing, feedback/yaw/UART/vision faults; max I/O timeout %u ms\n",max_motor_timeout);
    return 0;
}
