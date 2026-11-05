#include "logistics_task.h"
#include "vision_follow.h"
#include "jy61p.h"
#include "usart.h"
#include "tim.h"
#include "duiji.h"
#include <math.h>
#include <string.h>

volatile uint8_t logistics_state, logistics_drop_index, logistics_error;
volatile float logistics_x_m, logistics_y_m;
typedef struct { int64_t count; uint32_t tick, serial; } Encoder;
typedef struct { float x,y; } Point;
static volatile Encoder encoders[5];
static Encoder snapshot[5];
static int64_t previous[4], lift_origin;
static uint32_t used_serial[5], enter_tick, control_tick, poll_tick;
static uint8_t rx[8], rx_count, poll_id, stable, leg, stopped;
static uint32_t rx_tick, stable_vision_tick;
static uint8_t loaded_count;
static uint32_t lift_progress_tick;
static int64_t lift_progress_count;
static float heading, lift_target;
static uint8_t lift_down;
static Point target;
static uint8_t plan[LOG_PLAN_BYTES], plan_ready;
static const float tray_angles[3] = LOG_TRAY_SLOT_ANGLES;

static uint8_t valid_plan(const uint8_t *items, uint8_t length)
{
    uint8_t i;
    if (!items || length != LOG_PLAN_BYTES) return 0U;
    for (i=0U; i<LOG_PLAN_ITEM_COUNT; ++i) {
        uint8_t color=items[2U*i], shape=items[2U*i+1U];
        if (color!=1U && color!=2U && color!=3U && color!=4U && color!=7U) return 0U;
        if (shape!=1U && shape!=3U && shape!=4U && shape!=5U && shape!=6U) return 0U;
        /* Accept only combinations represented in the deployed model. */
        if (shape==1U && color!=4U) return 0U;
        if (shape==3U && color==7U) return 0U;
        if (shape==4U && color!=1U && color!=2U && color!=7U) return 0U;
    }
    return 1U;
}
uint8_t Logistics_SetPlan(const uint8_t *items, uint8_t length)
{
    if (Logistics_IsRunning() || !valid_plan(items,length)) return 0U;
    memcpy(plan,items,sizeof(plan)); plan_ready=1U;
    return 1U;
}
uint8_t Logistics_PlanMatches(const uint8_t *items, uint8_t length)
{
    return plan_ready && valid_plan(items,length) && memcmp(plan,items,sizeof(plan))==0;
}
uint8_t Logistics_PlanReady(void) { return plan_ready; }
void Logistics_ClearPlan(void)
{
    Logistics_Cancel();
    memset(plan,0,sizeof(plan)); plan_ready=0U;
}

static float bound(float value, float limit)
{ return value > limit ? limit : value < -limit ? -limit : value; }
static float wrap(float a)
{ while (a > 180.0f) a -= 360.0f; while (a < -180.0f) a += 360.0f; return a; }
static uint8_t send_motor(uint8_t *data, uint16_t size)
{
    uint8_t ok = HAL_UART_Transmit(&huart1,data,size,10U) == HAL_OK;
    HAL_Delay(size >= 8U ? 10U : 1U);
    return ok;
}
static uint8_t stop_axes(uint8_t first, uint8_t last)
{
    uint8_t id,ok=1U;
    uint8_t data[5]={0,0xFE,0x98,0,0x6B};
    for (id=first;id<=last;++id) { data[0]=id; if (!send_motor(data,5)) ok=0U; }
    return ok;
}
static uint8_t drive(float forward,float left,float omega)
{
    float wheels[4],rpm;
    uint8_t id;
    uint8_t data[8]={0,0xF6,0,0,0,0,1,0x6B};
    uint8_t sync[4]={0,0xFF,0x66,0x6B};
    if (forward==0.0f && left==0.0f && omega==0.0f) {
        if (stopped) return 1U;
        stopped=1U;
        return stop_axes(1,4);
    }
    stopped=0U;
    forward*=60.0f; left*=60.0f; omega*=60.0f*0.2125f;
    wheels[0]=forward-left-omega;
    wheels[1]=-(forward+left+omega);
    wheels[2]=forward+left-omega;
    wheels[3]=-(forward-left+omega);
    for (id=0;id<4;++id) {
        uint16_t speed;
        rpm=fabsf(wheels[id]); speed=(uint16_t)(rpm+0.5f);
        data[0]=id+1; data[2]=wheels[id]<0.0f ? 1U : 0U;
        data[3]=(uint8_t)(speed>>8); data[4]=(uint8_t)speed;
        if (!send_motor(data,8)) return 0U;
    }
    return send_motor(sync,4);
}
static void fail(uint8_t reason)
{
    logistics_error=reason; logistics_state=LOG_FAULT;
    (void)stop_axes(1,5); stopped=1U;
    (void)VisionObserve_Stop();
}
static void enter(uint8_t state)
{ logistics_state=state; enter_tick=HAL_GetTick(); stable=0U; }

void Logistics_MotorRxByteISR(uint8_t byte)
{
    uint32_t now=HAL_GetTick();
    if ((uint32_t)(now-rx_tick)>40U) rx_count=0U;
    rx_tick=now;
    rx[rx_count++]=byte;
    while (rx_count) {
        if (rx[0]>=1U && rx[0]<=5U && (rx_count<2U || rx[1]==0x36U)) {
            if (rx_count<8U) return;
            if (rx[2]<=1U && rx[7]==0x6BU) {
                uint32_t magnitude=((uint32_t)rx[3]<<24)|((uint32_t)rx[4]<<16)|
                                   ((uint32_t)rx[5]<<8)|rx[6];
                volatile Encoder *e=&encoders[rx[0]-1U];
                e->count=rx[2] ? -(int64_t)magnitude : (int64_t)magnitude;
                e->tick=now; ++e->serial;
                rx_count=0U; return;
            }
        }
        --rx_count; memmove(rx,rx+1,rx_count);
    }
}
static void read_snapshot(void)
{
    uint8_t i;
    uint32_t saved=__get_PRIMASK();
    __disable_irq();
    for (i=0;i<5;++i) {
        snapshot[i].count=encoders[i].count;
        snapshot[i].tick=encoders[i].tick;
        snapshot[i].serial=encoders[i].serial;
    }
    __set_PRIMASK(saved);
}
uint8_t Logistics_GetLiftFeedback(int64_t *count, uint32_t *tick, uint32_t *serial)
{
    uint32_t saved;
    if (!count || !tick || !serial) return 0U;
    saved=__get_PRIMASK();
    __disable_irq();
    *count=encoders[4].count;
    *tick=encoders[4].tick;
    *serial=encoders[4].serial;
    __set_PRIMASK(saved);
    return *serial != 0U;
}
static uint8_t feedback_fresh(uint32_t now)
{
    uint8_t i;
    for (i=0;i<5;++i)
        if (!snapshot[i].serial || (uint32_t)(now-snapshot[i].tick)>LOG_FEEDBACK_TIMEOUT_MS)
            return 0U;
    return 1U;
}
static uint8_t integrate(void)
{
    uint8_t i;
    float wheel[4],forward,left,theta;
    for (i=0;i<4;++i) if (snapshot[i].serial==used_serial[i]) return 0U;
    for (i=0;i<4;++i) {
        wheel[i]=(float)(snapshot[i].count-previous[i])*
                 LOG_WHEEL_CIRCUMFERENCE_M/LOG_ENCODER_COUNTS_REV;
        if (i==1U || i==3U) wheel[i]=-wheel[i];
        previous[i]=snapshot[i].count; used_serial[i]=snapshot[i].serial;
    }
    forward=(wheel[0]+wheel[1]+wheel[2]+wheel[3])*0.25f;
    left=(-wheel[0]+wheel[1]+wheel[2]-wheel[3])*0.25f;
    theta=wrap(Yaw-heading)*0.01745329252f;
    logistics_x_m+=-forward*sinf(theta)-left*cosf(theta);
    logistics_y_m+= forward*cosf(theta)-left*sinf(theta);
    return 1U;
}
static float yaw_control(void)
{ return bound(LOG_HEADING_GAIN*wrap(heading-Yaw),LOG_MAX_OMEGA); }
static void set_target(float x,float y)
{ target.x=x; target.y=y; stable=0U; enter_tick=HAL_GetTick(); }
/* Load at pickup once, then UL -> UR -> LR -> pickup, using straight legs. */
static uint8_t route_point(uint8_t index, uint8_t back, uint8_t segment, Point *p)
{
    Point points[2]; uint8_t count;
    if (back) {
        count=1; points[0].x=LOG_PICK_X_M; points[0].y=LOG_PICK_Y_M;
    } else if (index==0U) {
        /* Undo the final vision adjustment before the first long leg. */
        points[0].x=LOG_PICK_X_M; points[0].y=LOG_PICK_Y_M;
        count=2; points[1].x=LOG_PICK_X_M; points[1].y=LOG_FAR_M;
    } else if (index==1U) {
        count=1; points[0].x=LOG_FAR_M; points[0].y=LOG_FAR_M;
    } else {
        count=1; points[0].x=LOG_FAR_M; points[0].y=LOG_PICK_Y_M;
    }
    if (segment>=count) return 0U;
    *p=points[segment]; return 1U;
}
static void begin_route(uint8_t back)
{
    leg=1U;
    enter(back ? LOG_RETURN : LOG_OUTBOUND);
    (void)route_point(logistics_drop_index,back,0,&target);
}
static uint8_t move_step(uint32_t now,uint8_t fresh)
{
    float dx=target.x-logistics_x_m,dy=target.y-logistics_y_m;
    float theta=wrap(Yaw-heading)*0.01745329252f;
    float forward,left,speed;
    if ((uint32_t)(now-enter_tick)>LOG_SEGMENT_TIMEOUT_MS) {
        fail(LOG_ERROR_MOVE_TIMEOUT); return 0U;
    }
    if (fabsf(dx)<=LOG_POSITION_TOLERANCE_M && fabsf(dy)<=LOG_POSITION_TOLERANCE_M) {
        if (!drive(0,0,0)) { fail(LOG_ERROR_MOTOR_UART); return 0U; }
        if (fresh && ++stable>=3U) return 1U;
        return 0U;
    }
    stable=0U;
    forward=LOG_POSITION_GAIN*(-dx*sinf(theta)+dy*cosf(theta))/LOG_WHEEL_CIRCUMFERENCE_M;
    left=LOG_POSITION_GAIN*(-dx*cosf(theta)-dy*sinf(theta))/LOG_WHEEL_CIRCUMFERENCE_M;
    speed=sqrtf(forward*forward+left*left);
    if (speed>LOG_MAX_RPS) { forward*=LOG_MAX_RPS/speed; left*=LOG_MAX_RPS/speed; }
    if (!drive(forward,left,yaw_control())) fail(LOG_ERROR_MOTOR_UART);
    return 0U;
}
static void begin_lift(uint8_t state,uint8_t down,float metres)
{
    uint32_t pulse=(uint32_t)(metres*LOG_LIFT_PULSES_METRE+0.5f);
    uint8_t data[13]={5,0xFD,0,0,LOG_LIFT_RPM,0x50,0,0,0,0,0,0,0x6B};
    lift_origin=snapshot[4].count; used_serial[4]=snapshot[4].serial;
    lift_progress_count=lift_origin; lift_progress_tick=HAL_GetTick();
    lift_target=metres; lift_down=down;
    data[2]=down ? LOG_LIFT_DOWN_DIRECTION : (1U-LOG_LIFT_DOWN_DIRECTION);
    data[6]=(uint8_t)(pulse>>24); data[7]=(uint8_t)(pulse>>16);
    data[8]=(uint8_t)(pulse>>8); data[9]=(uint8_t)pulse;
    enter(state);
    if (!send_motor(data,13)) fail(LOG_ERROR_MOTOR_UART);
}
static uint8_t lift_step(uint32_t now)
{
    float distance=(float)(snapshot[4].count-lift_origin)*LOG_LIFT_METRES_REV/LOG_ENCODER_COUNTS_REV;
    uint8_t negative=(lift_down ? LOG_LIFT_DOWN_DIRECTION : 1U-LOG_LIFT_DOWN_DIRECTION);
    if (negative) distance=-distance;
    if ((uint32_t)(now-enter_tick)>LOG_LIFT_TIMEOUT_MS) { fail(LOG_ERROR_LIFT_TIMEOUT); return 0U; }
    if (snapshot[4].serial!=used_serial[4]) {
        int64_t progress=snapshot[4].count-lift_progress_count;
        used_serial[4]=snapshot[4].serial;
        if (progress>=LOG_LIFT_PROGRESS_COUNTS || progress<=-LOG_LIFT_PROGRESS_COUNTS) {
            lift_progress_count=snapshot[4].count; lift_progress_tick=now;
        }
        if (fabsf(distance-lift_target)<=LOG_LIFT_TOLERANCE_M) ++stable;
        else stable=0U;
    }
    if (fabsf(distance-lift_target)>LOG_LIFT_TOLERANCE_M &&
        (uint32_t)(now-lift_progress_tick)>=LOG_LIFT_NO_PROGRESS_MS) {
        fail(LOG_ERROR_LIFT_NO_PROGRESS); return 0U;
    }
    if (stable>=3U) {
        if (!stop_axes(5,5)) { fail(LOG_ERROR_MOTOR_UART); return 0U; }
        return 1U;
    }
    return 0U;
}
static void prepare(void)
{
    enter(LOG_PREPARE);
    YunTai(Duoji_GetGimbalPreset(1U)); ZhuanPan(tray_angles[logistics_drop_index]); ZhuaZi(Duoji_GetClawPreset(1U));
    if (!VisionObserve_StartTarget(plan[2U*logistics_drop_index],
                                  plan[2U*logistics_drop_index+1U])) fail(LOG_ERROR_MOTOR_UART);
}
void Logistics_Init(void)
{
    memset((void *)encoders,0,sizeof(encoders));
    rx_count=0; poll_id=1U; poll_tick=HAL_GetTick();
    logistics_state=LOG_IDLE; logistics_drop_index=logistics_error=0;
    memset(plan,0,sizeof(plan)); plan_ready=0U;
    loaded_count=0U;
    logistics_x_m=LOG_PICK_X_M; logistics_y_m=LOG_PICK_Y_M; stopped=1;
}
uint8_t Logistics_IsRunning(void)
{ return (logistics_state>=LOG_PREPARE && logistics_state<=LOG_RETURN) ||
         (logistics_state>=LOG_STORE_LOWER && logistics_state<=LOG_DELIVERY_FOLD); }
uint8_t Logistics_Start(void)
{
    uint8_t i;
    uint32_t now=HAL_GetTick();
    if (Logistics_IsRunning()) return 0U;
    if (!plan_ready) { logistics_error=LOG_ERROR_PLAN; return 0U; }
    read_snapshot();
    now=HAL_GetTick();
    if (!feedback_fresh(now)) { logistics_error=LOG_ERROR_FEEDBACK; return 0U; }
    if (!JY61P_HasFreshYaw(LOG_IMU_TIMEOUT_MS)) { logistics_error=LOG_ERROR_IMU; return 0U; }
    if (!stop_axes(1,5)) { logistics_error=LOG_ERROR_MOTOR_UART; return 0U; }
    for (i=0;i<4;++i) { previous[i]=snapshot[i].count; used_serial[i]=snapshot[i].serial; }
    logistics_x_m=LOG_PICK_X_M; logistics_y_m=LOG_PICK_Y_M;
    logistics_drop_index=logistics_error=loaded_count=0; heading=Yaw; stopped=1;
    ZhuaZi(Duoji_GetClawPreset(1U)); YunTai(Duoji_GetGimbalPreset(1U)); ZhuanPan(OUT1);
    if (Duoji_StartPWM(TIM_CHANNEL_1)!=HAL_OK ||
        Duoji_StartPWM(TIM_CHANNEL_2)!=HAL_OK ||
        Duoji_StartPWM(TIM_CHANNEL_3)!=HAL_OK) { fail(LOG_ERROR_SERVO); return 0U; }
    control_tick=now;
    prepare(); return Logistics_IsRunning();
}
void Logistics_Cancel(void)
{
    if (!Logistics_IsRunning()) return;
    logistics_state=LOG_ABORTED;
    (void)stop_axes(1,5); stopped=1U;
    (void)VisionObserve_Stop();
    /* Keep the claw holding its last target. Restart requires physical reset. */
}
void Logistics_Task(void)
{
    uint32_t now=HAL_GetTick(); uint8_t fresh;
    if ((uint32_t)(now-poll_tick)>=LOG_POLL_PERIOD_MS) {
        uint8_t data[3]={poll_id,0x36,0x6B};
        poll_tick=now; if (++poll_id>5U) poll_id=1U;
        if (!send_motor(data,3) && Logistics_IsRunning()) { fail(LOG_ERROR_MOTOR_UART); return; }
    }
    if (!Logistics_IsRunning()) return;
    read_snapshot();
    /* A reply can arrive during the poll above. Take time AFTER copying its
     * timestamp, otherwise unsigned age underflows across a SysTick edge. */
    now=HAL_GetTick();
    if (!feedback_fresh(now)) { fail(LOG_ERROR_FEEDBACK); return; }
    if (!JY61P_HasFreshYaw(LOG_IMU_TIMEOUT_MS)) { fail(LOG_ERROR_IMU); return; }
    if (fabsf(wrap(heading-Yaw))>LOG_MAX_HEADING_ERROR_DEG) { fail(LOG_ERROR_HEADING); return; }
    if ((uint32_t)(now-control_tick)<LOG_CONTROL_PERIOD_MS) return;
    control_tick=now;
    fresh=integrate();
    switch (logistics_state) {
    case LOG_PREPARE:
        if ((uint32_t)(now-enter_tick)>=LOG_SERVO_WAIT_MS) { enter(LOG_ALIGN); stable_vision_tick=0U; }
        break;
    case LOG_ALIGN: {
        VisionObservation sample; float ex,ey,forward,left,speed;
        if ((uint32_t)(now-enter_tick)>LOG_ALIGN_TIMEOUT_MS) { fail(LOG_ERROR_VISION); break; }
        if (fabsf(logistics_x_m-LOG_PICK_X_M)>LOG_ALIGN_MAX_OFFSET_M ||
            fabsf(logistics_y_m-LOG_PICK_Y_M)>LOG_ALIGN_MAX_OFFSET_M) { fail(LOG_ERROR_ALIGN_LIMIT); break; }
        if (!VisionObserve_Read(&sample,LOG_VISION_TIMEOUT_MS) ||
            sample.color!=plan[2U*logistics_drop_index] ||
            sample.shape!=plan[2U*logistics_drop_index+1U]) {
            stable=0; if (!drive(0,0,0)) fail(LOG_ERROR_MOTOR_UART); break;
        }
        ex=LOG_ALIGN_TARGET_X-sample.x; ey=LOG_ALIGN_TARGET_Y-sample.y;
        if (fabsf(ex)<=LOG_ALIGN_DEADBAND_PX && fabsf(ey)<=LOG_ALIGN_DEADBAND_PX) {
            if (!drive(0,0,0)) { fail(LOG_ERROR_MOTOR_UART); break; }
            if (sample.tick!=stable_vision_tick) { stable_vision_tick=sample.tick; ++stable; }
            if (stable>=5U) {
                if (!VisionObserve_Stop()) { fail(LOG_ERROR_MOTOR_UART); break; }
                begin_lift(LOG_PICK_LOWER,1,LOG_PICK_LOWER_M);
            }
        } else {
            stable=0;
            forward=0.002f*(LOG_ALIGN_EX_TO_FORWARD*ex+LOG_ALIGN_EY_TO_FORWARD*ey);
            left=0.002f*(LOG_ALIGN_EX_TO_LEFT*ex+LOG_ALIGN_EY_TO_LEFT*ey);
            speed=sqrtf(forward*forward+left*left);
            if (speed>LOG_ALIGN_MAX_RPS) { forward*=LOG_ALIGN_MAX_RPS/speed; left*=LOG_ALIGN_MAX_RPS/speed; }
            if (!drive(forward,left,yaw_control())) fail(LOG_ERROR_MOTOR_UART);
        }
        break;
    }
    case LOG_PICK_LOWER:
        if (lift_step(now)) { ZhuaZi(Duoji_GetClawPreset(0U)); enter(LOG_GRIP); } break;
    case LOG_GRIP:
        if ((uint32_t)(now-enter_tick)>=LOG_GRIP_WAIT_MS) begin_lift(LOG_PICK_RAISE,0,LOG_PICK_LOWER_M);
        break;
    case LOG_PICK_RAISE:
        if (lift_step(now)) { YunTai(Duoji_GetGimbalPreset(0U)); enter(LOG_FOLD); } break;
    case LOG_FOLD:
        if ((uint32_t)(now-enter_tick)>=LOG_SERVO_WAIT_MS)
            begin_lift(LOG_STORE_LOWER,1,LOG_STORE_LOWER_M);
        break;
    case LOG_STORE_LOWER:
        if (lift_step(now)) { ZhuaZi(Duoji_GetClawPreset(1U)); enter(LOG_STORE_RELEASE); } break;
    case LOG_STORE_RELEASE:
        if ((uint32_t)(now-enter_tick)>=LOG_GRIP_WAIT_MS)
            begin_lift(LOG_STORE_RAISE,0,LOG_STORE_LOWER_M);
        break;
    case LOG_STORE_RAISE:
        if (lift_step(now)) {
            ++loaded_count;
            if (loaded_count==3U) { logistics_drop_index=0U; begin_route(0); }
            else { YunTai(Duoji_GetGimbalPreset(1U)); enter(LOG_LOAD_CLEAR); }
        }
        break;
    case LOG_LOAD_CLEAR:
        if ((uint32_t)(now-enter_tick)>=LOG_SERVO_WAIT_MS) {
            ++logistics_drop_index;
            enter(LOG_LOAD_HOME); set_target(LOG_PICK_X_M,LOG_PICK_Y_M);
        }
        break;
    case LOG_LOAD_HOME:
        if (move_step(now,fresh)) prepare();
        break;
    case LOG_OUTBOUND:
    case LOG_RETURN: {
        uint8_t back=logistics_state==LOG_RETURN; Point next;
        if (move_step(now,fresh)) {
            if (route_point(logistics_drop_index,back,leg,&next)) {
                ++leg; set_target(next.x,next.y);
            } else if (!back) {
                /* Empty claw is above the tray at transport height. */
                ZhuaZi(Duoji_GetClawPreset(1U)); YunTai(Duoji_GetGimbalPreset(0U));
                ZhuanPan(tray_angles[logistics_drop_index]); enter(LOG_FETCH_TURN);
            } else { enter(LOG_DONE); }
        }
        break;
    }
    case LOG_FETCH_TURN:
        if ((uint32_t)(now-enter_tick)>=LOG_SERVO_WAIT_MS)
            begin_lift(LOG_FETCH_LOWER,1,LOG_FETCH_LOWER_M);
        break;
    case LOG_FETCH_LOWER:
        if (lift_step(now)) { ZhuaZi(Duoji_GetClawPreset(0U)); enter(LOG_FETCH_GRIP); } break;
    case LOG_FETCH_GRIP:
        if ((uint32_t)(now-enter_tick)>=LOG_GRIP_WAIT_MS)
            begin_lift(LOG_FETCH_RAISE,0,LOG_FETCH_LOWER_M);
        break;
    case LOG_FETCH_RAISE:
        if (lift_step(now)) { YunTai(Duoji_GetGimbalPreset(1U)); enter(LOG_UNFOLD); } break;
    case LOG_UNFOLD:
        if ((uint32_t)(now-enter_tick)>=LOG_SERVO_WAIT_MS) begin_lift(LOG_PLACE_LOWER,1,LOG_PLACE_LOWER_M);
        break;
    case LOG_PLACE_LOWER:
        if (lift_step(now)) { ZhuaZi(Duoji_GetClawPreset(1U)); enter(LOG_RELEASE); } break;
    case LOG_RELEASE:
        if ((uint32_t)(now-enter_tick)>=LOG_GRIP_WAIT_MS) begin_lift(LOG_PLACE_RAISE,0,LOG_PLACE_LOWER_M);
        break;
    case LOG_PLACE_RAISE:
        if (lift_step(now)) {
            YunTai(Duoji_GetGimbalPreset(0U)); enter(LOG_DELIVERY_FOLD);
        }
        break;
    case LOG_DELIVERY_FOLD:
        if ((uint32_t)(now-enter_tick)>=LOG_SERVO_WAIT_MS) {
            if (logistics_drop_index<2U) { ++logistics_drop_index; begin_route(0); }
            else begin_route(1);
        }
        break;
    default: break;
    }
}
