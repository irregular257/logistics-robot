#include <stdio.h>
#include <string.h>
#include <vector>
#include "stubs/hal_stubs.h"
#include "../../Core/Inc/web_protocol.h"
#include "../../Core/Src/web_control.c"

UART_HandleTypeDef huart1={1}, huart4={4};
TIM_HandleTypeDef htim1={1};
uint8_t vision_follow_enable;
int start, zero;
volatile uint8_t logistics_state,logistics_drop_index,logistics_error;
volatile float logistics_x_m,logistics_y_m;
static unsigned auto_starts;
static unsigned plan_set_calls, plan_clear_calls;
static bool review_plan_ready;
static uint8_t review_plan[WEB_TASK_PLAN_SIZE];
static const uint8_t sample_plan[WEB_TASK_PLAN_SIZE]={4U,1U,7U,4U,3U,3U};
void Logistics_Init(void){logistics_state=LOG_IDLE;logistics_error=logistics_drop_index=0;auto_starts=plan_set_calls=plan_clear_calls=0;review_plan_ready=false;memset(review_plan,0,sizeof(review_plan));}
uint8_t Logistics_IsRunning(void){return (logistics_state>=LOG_PREPARE && logistics_state<=LOG_RETURN) || (logistics_state>=LOG_STORE_LOWER && logistics_state<=LOG_DELIVERY_FOLD);}
uint8_t Logistics_Start(void){if(!review_plan_ready)return 0;++auto_starts;logistics_state=LOG_PREPARE;return 1;}
void Logistics_Cancel(void){if(Logistics_IsRunning())logistics_state=LOG_ABORTED;}
uint8_t Logistics_SetPlan(const uint8_t *items,uint8_t length){
  ++plan_set_calls;
  if(Logistics_IsRunning() || !items || length!=sizeof(review_plan))return 0;
  /* The real planner has separate behavioral tests. This table is the model's
   * fixture inventory so command tests do not accept arbitrary target bytes. */
  static const uint8_t supported[][2]={{4,1},{1,3},{2,3},{3,3},{4,3},
    {1,4},{2,4},{7,4},{1,5},{2,5},{3,5},{4,5},{7,5},
    {1,6},{2,6},{3,6},{4,6},{7,6}};
  for(unsigned i=0;i<3U;++i){bool found=false;
    for(unsigned j=0;j<sizeof(supported)/sizeof(supported[0]);++j)
      if(items[2U*i]==supported[j][0] && items[2U*i+1U]==supported[j][1])found=true;
    if(!found)return 0;
  }
  memcpy(review_plan,items,sizeof(review_plan));review_plan_ready=true;return 1;
}
uint8_t Logistics_PlanMatches(const uint8_t *items,uint8_t length){return review_plan_ready && items && length==sizeof(review_plan) && memcmp(review_plan,items,sizeof(review_plan))==0;}
uint8_t Logistics_PlanReady(void){return review_plan_ready?1U:0U;}
void Logistics_ClearPlan(void){++plan_clear_calls;Logistics_Cancel();review_plan_ready=false;memset(review_plan,0,sizeof(review_plan));}
void Logistics_Task(void){}
uint8_t Logistics_GetLiftFeedback(int64_t *,uint32_t *,uint32_t *){return 0;}
static uint32_t clock_ms;
static uint8_t *receive_byte;
static unsigned go_count, stop_count, claw_count, failures;
static unsigned gimbal_count, gimbal_pwm_count;
static unsigned tray_count, tray_pwm_count;
static uint16_t last_tray_angle;
static uint16_t last_gimbal_angle;
static uint16_t last_claw_angle;
static uint16_t review_presets[4]={GRAB,OPEN,IN,OUT};
static bool fail_gimbal_pwm;
static bool fail_tray_pwm;
static bool claw_pwm_running, gimbal_pwm_running, tray_pwm_running;
static int wheel_values[4];
static bool fail_motor;
static std::vector<WebFrame> replies;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d: %s\n",__FILE__,__LINE__,#cond);++failures; } } while(0)
uint32_t HAL_GetTick(void){return clock_ms;}
void HAL_Delay(uint32_t n){clock_ms+=n;}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *port,uint8_t *wire,uint16_t n,uint32_t timeout){
  CHECK(timeout==10);
  if(port==&huart1){
    if(fail_motor)return 1;
    if(n==5){CHECK(wire[1]==0xFE && wire[2]==0x98 && wire[3]==0 && wire[4]==0x6B);++stop_count;}
    else if(n==8){CHECK(wire[1]==0xF6 && (wire[5]==WEB_CHASSIS_ACCEL || (((wire[3]<<8)|wire[4])==0 && wire[5]==0)) && wire[6]==1 && wire[7]==0x6B);if(wire[0]<=4)wheel_values[wire[0]-1]=((wire[2]? -1:1)*((wire[3]<<8)|wire[4]));}
    else if(n==4){CHECK(wire[0]==0 && wire[1]==0xFF && wire[2]==0x66 && wire[3]==0x6B);if(wheel_values[0]||wheel_values[1]||wheel_values[2]||wheel_values[3])++go_count;}
    else CHECK(false);
  }
  else { WebFrame f={}; CHECK(n>=13);f.session=Web_Get32(wire+4);f.sequence=Web_Get16(wire+8);f.command=wire[10];f.length=wire[3];memcpy(f.payload,wire+11,f.length);replies.push_back(f);}
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *byte,uint16_t){receive_byte=byte;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *){return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t channel){if(channel==TIM_CHANNEL_3){++gimbal_pwm_count;if(fail_gimbal_pwm)return 1;}if(channel==TIM_CHANNEL_2){++tray_pwm_count;if(fail_tray_pwm)return 1;}return HAL_OK;}
HAL_StatusTypeDef Duoji_StartPWM(uint32_t channel){
  bool &running=channel==TIM_CHANNEL_3?gimbal_pwm_running:channel==TIM_CHANNEL_2?tray_pwm_running:claw_pwm_running;
  if(running)return HAL_OK;
  HAL_StatusTypeDef result=HAL_TIM_PWM_Start(&htim1,channel);
  if(result==HAL_OK)running=true;
  return result;
}
void ZhuaZi(uint16_t angle){++claw_count;last_claw_angle=angle;}
HAL_StatusTypeDef JiaZhua_SetAngle(uint16_t angle){if(angle>270U)return HAL_ERROR;ZhuaZi(angle);return Duoji_StartPWM(TIM_CHANNEL_1);}
HAL_StatusTypeDef Tray_SetAngle(uint16_t angle){if(angle>270U)return HAL_ERROR;++tray_count;last_tray_angle=angle;return Duoji_StartPWM(TIM_CHANNEL_2);}
HAL_StatusTypeDef Duoji_SetPresets(uint16_t close,uint16_t open,uint16_t retract,uint16_t extend){
  if(close>270U||open>270U||retract>270U||extend>270U)return HAL_ERROR;
  review_presets[0]=close;review_presets[1]=open;review_presets[2]=retract;review_presets[3]=extend;return HAL_OK;
}
uint16_t Duoji_GetClawPreset(uint8_t open){return review_presets[open?1:0];}
uint16_t Duoji_GetGimbalPreset(uint8_t extended){return review_presets[extended?3:2];}
void YunTai(uint16_t angle){++gimbal_count;last_gimbal_angle=angle;}
static void inject(const uint8_t *data,unsigned n){for(unsigned i=0;i<n;++i){*receive_byte=data[i];WebControl_RxCompleteISR();++clock_ms;}}
static void request(uint8_t command,uint32_t session,uint16_t sequence,const int8_t *payload=nullptr,uint8_t n=0,bool process=true){
  WebFrame f={};f.command=command;f.session=session;f.sequence=sequence;f.length=n;
  if(n)memcpy(f.payload,payload,n);uint8_t wire[WEB_MAX_FRAME];uint8_t bytes=Web_Encode(&f,wire);inject(wire,bytes);if(process)WebControl_Task();
}
static void reset(){clock_ms=1;start=zero=0;vision_follow_enable=0;fail_motor=fail_gimbal_pwm=fail_tray_pwm=false;claw_pwm_running=gimbal_pwm_running=tray_pwm_running=false;WebControl_Init();go_count=stop_count=claw_count=gimbal_count=gimbal_pwm_count=tray_count=tray_pwm_count=0;last_gimbal_angle=last_tray_angle=0;replies.clear();}
static void manual(uint32_t session=1,uint16_t sequence=1){request(WEB_CMD_HELLO,session,sequence);int8_t m=WEB_MODE_MANUAL;request(WEB_CMD_MODE,session,(uint16_t)(sequence+1),&m,1);CHECK(mode==WEB_MODE_MANUAL && session_valid);}
static uint8_t lastResult(){for(size_t i=replies.size();i>0;--i)if(replies[i-1].command==WEB_CMD_ACK)return replies[i-1].payload[1];return 255;}
static void gimbalRequest(uint16_t angle,uint16_t sequence,uint32_t session=1){uint8_t p[2];Web_Put16(p,angle);request(WEB_CMD_GIMBAL,session,sequence,(const int8_t *)p,2);}
static void trayRequest(uint16_t angle,uint16_t sequence,uint32_t session=1){uint8_t p[2];Web_Put16(p,angle);request(WEB_CMD_TRAY_ANGLE,session,sequence,(const int8_t *)p,2);}
static void taskRequest(uint16_t sequence,uint32_t session=1){request(WEB_CMD_TASK,session,sequence,(const int8_t *)sample_plan,sizeof(sample_plan));}
static void startRequest(uint16_t sequence,uint32_t session=1){uint8_t p[1U+WEB_TASK_PLAN_SIZE]={1U};memcpy(p+1,sample_plan,sizeof(sample_plan));request(WEB_CMD_START,session,sequence,(const int8_t *)p,sizeof(p));}
static void tests(){
  reset();manual();int8_t jog_values[]={0,20,0,0};request(WEB_CMD_JOG,1,3,jog_values,4);
  CHECK(jog_active && jog_tick==link_tick && go_count>0 && wheel_values[0]>=11 && wheel_values[0]<=12 && wheel_values[1]==-wheel_values[0] && wheel_values[2]==wheel_values[0] && wheel_values[3]==-wheel_values[0]);
  uint32_t lease=jog_tick,link=link_tick;clock_ms+=100;
  request(WEB_CMD_JOG,1,3,jog_values,4);CHECK(jog_tick==lease && link_tick==link && lastResult()==WEB_RESULT_OK);
  request(WEB_CMD_HELLO,1,1);CHECK(mode==WEB_MODE_MANUAL && jog_tick==lease && link_tick==link);
  request(WEB_CMD_HEARTBEAT,1,4);CHECK(jog_tick==lease && link_tick>link);
  clock_ms=lease+WEB_JOG_LEASE_MS;WebControl_Task();CHECK(mode==WEB_MODE_IDLE && !jog_active && fault==WEB_FAULT_JOG_TIMEOUT);
  /* Holding a button keeps one continuous driver target, without repeatedly
   * restarting the ramp or spending 50 ms on identical speed batches. */
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);
  CHECK(go_count==1 && !motion_dirty);
  unsigned held_stop_count=stop_count;
  for(uint16_t seq=4;seq<=18;++seq){clock_ms+=100;request(WEB_CMD_JOG,1,seq,jog_values,4);CHECK(lastResult()==WEB_RESULT_OK && jog_active && mode==WEB_MODE_MANUAL && go_count==1 && stop_count==held_stop_count && !motion_dirty);}
  int8_t backward[]={0,-20,0,0};request(WEB_CMD_JOG,1,19,backward,4);clock_ms+=WEB_MOTOR_PERIOD_MS;WebControl_Task();
  CHECK(go_count==2 && wheel_values[0]<=-11 && wheel_values[0]>=-12 && wheel_values[1]==-wheel_values[0] && wheel_values[2]==wheel_values[0] && wheel_values[3]==-wheel_values[0]);
  int8_t release_motion[]={0,0,0,0};request(WEB_CMD_JOG,1,20,release_motion,4);CHECK(!jog_active && !motion_dirty && stop_count==held_stop_count+5);
  request(WEB_CMD_JOG,1,21,backward,4);clock_ms+=WEB_MOTOR_PERIOD_MS;WebControl_Task();CHECK(go_count==3 && jog_active);
  lease=jog_tick;clock_ms=lease+WEB_JOG_LEASE_MS;WebControl_Task();CHECK(!jog_active && mode==WEB_MODE_IDLE && fault==WEB_FAULT_JOG_TIMEOUT);
  /* A second identical target queued before first I/O must wait for I/O too. */
  reset();manual();motor_tick=clock_ms;request(WEB_CMD_JOG,1,3,jog_values,4);CHECK(pending_jog_valid && motion_dirty && go_count==0);
  request(WEB_CMD_JOG,1,4,jog_values,4);CHECK(pending_jog_valid && motion_dirty && go_count==0);
  clock_ms+=WEB_MOTOR_PERIOD_MS;WebControl_Task();CHECK(!pending_jog_valid && !motion_dirty && go_count==1 && lastResult()==WEB_RESULT_OK);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);request(WEB_CMD_STOP,1,5);
  int8_t m=WEB_MODE_MANUAL;request(WEB_CMD_MODE,1,4,&m,1);CHECK(mode==WEB_MODE_IDLE && lastResult()==WEB_RESULT_STALE);
  request(WEB_CMD_JOG,1,3,jog_values,4);CHECK(mode==WEB_MODE_IDLE && !jog_active);
  request(WEB_CMD_MODE,1,6,&m,1);CHECK(mode==WEB_MODE_MANUAL);
  request(WEB_CMD_STOP,999,1);CHECK(mode==WEB_MODE_IDLE && active_session==1);
  reset();manual();int8_t bad[]={0,51,0,0};request(WEB_CMD_JOG,1,3,bad,4);CHECK(mode==WEB_MODE_IDLE && lastResult()==WEB_RESULT_BAD_ARGUMENT);
  reset();manual();int8_t both[]={0,20,0,1};request(WEB_CMD_JOG,1,3,both,4);CHECK(mode==WEB_MODE_IDLE && lastResult()==WEB_RESULT_INTERLOCK);
  reset();manual();int8_t lift[]={0,0,0,1};request(WEB_CMD_JOG,1,3,lift,4);CHECK(mode==WEB_MODE_IDLE && lastResult()==WEB_RESULT_INTERLOCK);
  reset();manual(1,65534);request(WEB_CMD_JOG,1,0,jog_values,4);CHECK(jog_active && lastResult()==WEB_RESULT_OK);
  request(WEB_CMD_JOG,1,65535,jog_values,4);CHECK(lastResult()==WEB_RESULT_STALE);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);
  request(WEB_CMD_HELLO,2,1,nullptr,0,false);request(WEB_CMD_JOG,1,4,jog_values,4,false);WebControl_Task();
  CHECK(active_session==2 && mode==WEB_MODE_IDLE && !jog_active);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);
  clock_ms=link_tick+WEB_LINK_LEASE_MS;WebControl_Task();CHECK(!session_valid && mode==WEB_MODE_IDLE);
  request(WEB_CMD_JOG,1,4,jog_values,4);CHECK(!jog_active && lastResult()==WEB_RESULT_STALE);
  request(WEB_CMD_HELLO,2,1);CHECK(session_valid && active_session==2 && mode==WEB_MODE_IDLE);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);
  uint8_t noise[256];memset(noise,0xA5,sizeof noise);inject(noise,sizeof noise);CHECK(rx_faults!=0);
  WebControl_Task();CHECK(!session_valid && !jog_active && mode==WEB_MODE_IDLE && fault==WEB_FAULT_RX_OVERFLOW);
  request(WEB_CMD_HELLO,3,1);CHECK(session_valid && active_session==3 && fault==WEB_FAULT_NONE);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);start=1;WebControl_Task();CHECK(!start && mode==WEB_MODE_IDLE && !jog_active);
  /* The reported car motion exchanges the legacy lateral/yaw patterns. Verify the
   * corrected four buttons through the wire protocol, keeping their own RPM
   * scales: 20% lateral is about 12 RPM; 20% yaw remains about 5 RPM. */
  reset();manual();int8_t right[]={20,0,0,0};request(WEB_CMD_JOG,1,3,right,4);
  CHECK(wheel_values[0]>=11 && wheel_values[0]<=12);
  const int lateral_rpm=wheel_values[0];
  for(unsigned i=0;i<4;++i)CHECK(wheel_values[i]==lateral_rpm);
  int8_t left[]={-20,0,0,0};request(WEB_CMD_JOG,1,4,left,4);clock_ms+=WEB_MOTOR_PERIOD_MS;WebControl_Task();
  for(unsigned i=0;i<4;++i)CHECK(wheel_values[i]==-lateral_rpm);
  int8_t turn_left[]={0,0,20,0};request(WEB_CMD_JOG,1,5,turn_left,4);clock_ms+=WEB_MOTOR_PERIOD_MS;WebControl_Task();
  CHECK(wheel_values[0]==-5 && wheel_values[1]==-5 && wheel_values[2]==5 && wheel_values[3]==5);
  int8_t turn_right[]={0,0,-20,0};request(WEB_CMD_JOG,1,6,turn_right,4);clock_ms+=WEB_MOTOR_PERIOD_MS;WebControl_Task();
  CHECK(wheel_values[0]==5 && wheel_values[1]==5 && wheel_values[2]==-5 && wheel_values[3]==-5);
  /* Mixed left/yaw commands can cancel at the rear pair while the front
   * pair still turns. Reversing both requests reverses every nonzero RPM. */
  int8_t mixed_cancel[]={-17,0,40,0};request(WEB_CMD_JOG,1,7,mixed_cancel,4);clock_ms+=WEB_MOTOR_PERIOD_MS;WebControl_Task();
  CHECK(wheel_values[0]>=-21 && wheel_values[0]<=-19 && wheel_values[1]==wheel_values[0] && wheel_values[2]==0 && wheel_values[3]==0);
  const int mixed_front=wheel_values[0];
  int8_t mixed_reverse[]={17,0,-40,0};request(WEB_CMD_JOG,1,8,mixed_reverse,4);clock_ms+=WEB_MOTOR_PERIOD_MS;WebControl_Task();
  CHECK(wheel_values[0]==-mixed_front && wheel_values[1]==-mixed_front && wheel_values[2]==0 && wheel_values[3]==0);
  request(WEB_CMD_STOP,1,9);CHECK(!jog_active && mode==WEB_MODE_IDLE);
  for(unsigned i=0;i<4;++i)CHECK(wheel_values[i]==0);
  reset();manual();int8_t fastest[]={0,50,0,0};request(WEB_CMD_JOG,1,3,fastest,4);CHECK(wheel_values[0]>=29 && wheel_values[0]<=30 && wheel_values[1]==-wheel_values[0] && wheel_values[2]==wheel_values[0] && wheel_values[3]==-wheel_values[0]);
  request(WEB_CMD_STOP,1,4);CHECK(!jog_active && wheel_values[0]==0 && wheel_values[1]==0 && wheel_values[2]==0 && wheel_values[3]==0);
  reset();manual();WebFrame f={};f.command=WEB_CMD_JOG;f.session=1;f.sequence=3;f.length=4;memcpy(f.payload,jog_values,4);
  uint8_t wire[WEB_MAX_FRAME];uint8_t n=Web_Encode(&f,wire);wire[n-1]^=0x80;inject(wire,n);WebControl_Task();CHECK(!jog_active);
  wire[n-1]^=0x80;inject(wire,n);WebControl_Task();CHECK(jog_active);
  reset();manual();fail_motor=true;request(WEB_CMD_JOG,1,3,jog_values,4);clock_ms+=WEB_MOTOR_PERIOD_MS;WebControl_Task();CHECK(mode==WEB_MODE_IDLE && !jog_active && fault==WEB_FAULT_MOTOR_UART && lastResult()==WEB_RESULT_INTERLOCK);
  reset();request(WEB_CMD_HELLO,1,1);fail_motor=true;request(WEB_CMD_MODE,1,2,&m,1);CHECK(mode==WEB_MODE_IDLE && fault==WEB_FAULT_MOTOR_UART && lastResult()==WEB_RESULT_INTERLOCK);

  reset();manual();int8_t stopped[]={0,0,0,0};request(WEB_CMD_JOG,1,3,stopped,4);clock_ms+=400;request(WEB_CMD_HEARTBEAT,1,4);CHECK(mode==WEB_MODE_MANUAL && !jog_active);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);int8_t malformed=9;request(WEB_CMD_STOP,999,0,&malformed,1);CHECK(mode==WEB_MODE_IDLE && !jog_active && lastResult()==WEB_RESULT_BAD_ARGUMENT);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);WebControl_UartErrorISR();WebControl_Task();CHECK(mode==WEB_MODE_IDLE && !jog_active && !session_valid && fault==WEB_FAULT_UART);
  request(WEB_CMD_HELLO,3,1);request(WEB_CMD_JOG,3,2,jog_values,4);CHECK(!jog_active && lastResult()==WEB_RESULT_WRONG_MODE);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4,false);clock_ms+=WEB_JOG_LEASE_MS;request(WEB_CMD_HEARTBEAT,1,4,nullptr,0,false);WebControl_Task();CHECK(!jog_active && mode==WEB_MODE_IDLE);
  reset();manual();int8_t open=1;request(WEB_CMD_CLAW,1,3,&open,1);CHECK(claw_target==1 && claw_count==1);request(WEB_CMD_CLAW,1,3,&open,1);CHECK(claw_count==1);
  /* TASK stores exactly three explicit targets without moving an actuator.
   * START must echo that acknowledged plan, so old or edited browser state
   * cannot silently launch a different route. */
  request(WEB_CMD_TASK,1,4);CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && !Logistics_PlanReady());
  request(WEB_CMD_START,1,5);CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && auto_starts==0);
  int8_t positioned=1;
  request(WEB_CMD_START,1,6,&positioned,1);CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && auto_starts==0);
  startRequest(7);CHECK(lastResult()==WEB_RESULT_INTERLOCK && auto_starts==0 && mode==WEB_MODE_MANUAL);
  request(WEB_CMD_TASK,1,8,(const int8_t *)sample_plan,5U);CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && !Logistics_PlanReady());
  uint8_t invalid_plan[WEB_TASK_PLAN_SIZE];memcpy(invalid_plan,sample_plan,sizeof(invalid_plan));invalid_plan[0]=1U;
  request(WEB_CMD_TASK,1,9,(const int8_t *)invalid_plan,sizeof(invalid_plan));CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && !Logistics_PlanReady());
  unsigned before_claw=claw_count,before_gimbal=gimbal_count,before_tray=tray_count,before_stop=stop_count;
  taskRequest(10);CHECK(lastResult()==WEB_RESULT_OK && Logistics_PlanMatches(sample_plan,sizeof(sample_plan)) && auto_starts==0);
  CHECK(claw_count==before_claw && gimbal_count==before_gimbal && tray_count==before_tray && stop_count==before_stop && go_count==0);
  unsigned set_calls=plan_set_calls,clear_calls=plan_clear_calls;uint32_t task_lease=link_tick;
  taskRequest(10);CHECK(lastResult()==WEB_RESULT_OK && plan_set_calls==set_calls && link_tick==task_lease);
  request(WEB_CMD_HELLO,1,1);CHECK(Logistics_PlanReady() && plan_clear_calls==clear_calls && link_tick==task_lease);
  clock_ms+=WEB_STATUS_PERIOD_MS;request(WEB_CMD_HEARTBEAT,1,11);
  CHECK(replies.back().command==WEB_CMD_STATUS && replies.back().length==WEB_TASK_STATUS_SIZE && replies.back().payload[30]==WEB_TASK_PROTOCOL_VERSION && replies.back().payload[31]==1U);
  uint8_t mismatched_start[1U+WEB_TASK_PLAN_SIZE]={1U};memcpy(mismatched_start+1,sample_plan,sizeof(sample_plan));mismatched_start[2]=3U;
  request(WEB_CMD_START,1,12,(const int8_t *)mismatched_start,sizeof(mismatched_start));CHECK(lastResult()==WEB_RESULT_INTERLOCK && auto_starts==0 && claw_count==before_claw);
  memcpy(mismatched_start+1,sample_plan,sizeof(sample_plan));mismatched_start[0]=0U;
  request(WEB_CMD_START,1,13,(const int8_t *)mismatched_start,sizeof(mismatched_start));CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && auto_starts==0);
  startRequest(14);CHECK(lastResult()==WEB_RESULT_OK && mode==WEB_MODE_AUTO && auto_starts==1);
  startRequest(14);CHECK(auto_starts==1); // Retry cannot restart the route.
  taskRequest(15);CHECK(lastResult()==WEB_RESULT_WRONG_MODE && plan_set_calls==set_calls && Logistics_PlanMatches(sample_plan,sizeof(sample_plan)));
  request(WEB_CMD_TASK,1,16,(const int8_t *)invalid_plan,sizeof(invalid_plan));CHECK(lastResult()==WEB_RESULT_WRONG_MODE && plan_set_calls==set_calls);
  request(WEB_CMD_JOG,1,17,jog_values,4);CHECK(lastResult()==WEB_RESULT_WRONG_MODE && mode==WEB_MODE_AUTO);
  startRequest(18);CHECK(lastResult()==WEB_RESULT_WRONG_MODE && auto_starts==1);
  request(WEB_CMD_PAUSE,1,19);CHECK(lastResult()==WEB_RESULT_OK && mode==WEB_MODE_IDLE && logistics_state==LOG_ABORTED && Logistics_PlanReady());
  startRequest(20);CHECK(mode==WEB_MODE_AUTO && auto_starts==2);
  request(WEB_CMD_HELLO,2,1);CHECK(mode==WEB_MODE_IDLE && logistics_state==LOG_ABORTED && !Logistics_PlanReady() && plan_clear_calls==clear_calls+1U);
  startRequest(2,2);CHECK(lastResult()==WEB_RESULT_INTERLOCK && auto_starts==2);
  clock_ms+=WEB_STATUS_PERIOD_MS;request(WEB_CMD_HEARTBEAT,2,3);
  CHECK(replies.back().length==WEB_TASK_STATUS_SIZE && replies.back().payload[30]==WEB_TASK_PROTOCOL_VERSION && replies.back().payload[31]==0U);
  taskRequest(4,2);startRequest(5,2);CHECK(mode==WEB_MODE_AUTO && auto_starts==3);
  clock_ms=link_tick+WEB_LINK_LEASE_MS;WebControl_Task();CHECK(mode==WEB_MODE_IDLE && logistics_state==LOG_ABORTED);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);taskRequest(4);
  CHECK(lastResult()==WEB_RESULT_INTERLOCK && jog_active && plan_set_calls==0 && !Logistics_PlanReady());
  /* Exercise actual controller dispatch and PWM channel, not a UI-only target. */
  reset();manual();gimbalRequest(54,3);CHECK(lastResult()==WEB_RESULT_OK && gimbal_target==54 && last_gimbal_angle==54 && gimbal_count==1 && gimbal_pwm_count==1);
  uint32_t gimbal_lease=link_tick;gimbalRequest(54,3);CHECK(gimbal_count==1 && link_tick==gimbal_lease);
  gimbalRequest(229,4);CHECK(lastResult()==WEB_RESULT_OK && gimbal_target==229 && last_gimbal_angle==229 && gimbal_count==2 && gimbal_pwm_count==1);
  gimbalRequest(271,5);CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && gimbal_count==2 && gimbal_target==229);
  gimbalRequest(65535,6);CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && gimbal_count==2);
  int8_t malformed_gimbal=54;request(WEB_CMD_GIMBAL,1,7,&malformed_gimbal,1);CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && gimbal_count==2);
  gimbalRequest(120,8,999);CHECK(lastResult()==WEB_RESULT_STALE && gimbal_count==2);
  clock_ms+=WEB_STATUS_PERIOD_MS;request(WEB_CMD_HEARTBEAT,1,9);
  CHECK(!replies.empty() && replies.back().command==WEB_CMD_STATUS && replies.back().length==WEB_TASK_STATUS_SIZE && Web_Get16(replies.back().payload+16)==229);
  request(WEB_CMD_STOP,1,10);CHECK(mode==WEB_MODE_IDLE && gimbal_target==229);
  gimbalRequest(54,11);CHECK(lastResult()==WEB_RESULT_WRONG_MODE && gimbal_count==2);
  reset();manual();clock_ms+=WEB_STATUS_PERIOD_MS;request(WEB_CMD_HEARTBEAT,1,3);
  CHECK(replies.back().command==WEB_CMD_STATUS && replies.back().length==WEB_TASK_STATUS_SIZE && Web_Get16(replies.back().payload+16)==WEB_GIMBAL_UNKNOWN_ANGLE);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);gimbalRequest(120,4);CHECK(lastResult()==WEB_RESULT_INTERLOCK && gimbal_count==0 && jog_active);
  reset();manual();gimbalRequest(120,3);taskRequest(4);startRequest(5);CHECK(mode==WEB_MODE_AUTO && gimbal_target==WEB_GIMBAL_UNKNOWN_ANGLE);
  gimbalRequest(54,6);CHECK(lastResult()==WEB_RESULT_WRONG_MODE && gimbal_count==1);
  reset();manual();fail_gimbal_pwm=true;gimbalRequest(54,3);CHECK(lastResult()==WEB_RESULT_INTERLOCK && mode==WEB_MODE_IDLE && fault==WEB_FAULT_INTERLOCK && gimbal_target==WEB_GIMBAL_UNKNOWN_ANGLE);
  reset();manual();clock_ms+=WEB_STATUS_PERIOD_MS;request(WEB_CMD_HEARTBEAT,1,40000);CHECK(status_sequence>=1);uint16_t first=status_sequence;clock_ms+=WEB_STATUS_PERIOD_MS;request(WEB_CMD_HEARTBEAT,1,40001);CHECK(status_sequence==(uint16_t)(first+1));

  /* Full angle dispatch and calibration must reach STM32, while storing a
   * calibration cannot itself cause a servo movement. */
  reset();manual();gimbalRequest(0,3);gimbalRequest(270,4);
  CHECK(lastResult()==WEB_RESULT_OK && gimbal_count==2 && last_gimbal_angle==270);
  uint8_t angle_bytes[2];Web_Put16(angle_bytes,0);
  request(WEB_CMD_CLAW_ANGLE,1,5,(const int8_t *)angle_bytes,2);
  CHECK(lastResult()==WEB_RESULT_OK && claw_angle_target==0 && last_claw_angle==0);
  Web_Put16(angle_bytes,270);request(WEB_CMD_CLAW_ANGLE,1,6,(const int8_t *)angle_bytes,2);
  CHECK(lastResult()==WEB_RESULT_OK && claw_angle_target==270 && last_claw_angle==270 && claw_count==2);
  Web_Put16(angle_bytes,271);request(WEB_CMD_CLAW_ANGLE,1,7,(const int8_t *)angle_bytes,2);
  CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && claw_count==2);
  uint8_t preset_bytes[8];const uint16_t calibrated[4]={120,210,65,170};
  for(unsigned i=0;i<4;++i)Web_Put16(preset_bytes+2*i,calibrated[i]);
  request(WEB_CMD_SERVO_PRESETS,1,8,(const int8_t *)preset_bytes,8);
  CHECK(lastResult()==WEB_RESULT_OK && claw_count==2 && gimbal_count==2 && Duoji_GetClawPreset(1)==210 && Duoji_GetGimbalPreset(0)==65);
  uint32_t calibration_lease=link_tick;request(WEB_CMD_SERVO_PRESETS,1,8,(const int8_t *)preset_bytes,8);
  CHECK(link_tick==calibration_lease && claw_count==2 && gimbal_count==2);
  int8_t calibrated_open=1;request(WEB_CMD_CLAW,1,9,&calibrated_open,1);
  CHECK(claw_angle_target==210 && last_claw_angle==210 && claw_count==3);
  clock_ms+=WEB_STATUS_PERIOD_MS;request(WEB_CMD_HEARTBEAT,1,10);
  CHECK(replies.back().length==WEB_TASK_STATUS_SIZE && Web_Get16(replies.back().payload+18)==210);
  for(unsigned i=0;i<4;++i)CHECK(Web_Get16(replies.back().payload+20+2*i)==calibrated[i]);
  Web_Put16(preset_bytes+2,271);request(WEB_CMD_SERVO_PRESETS,1,11,(const int8_t *)preset_bytes,8);
  CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && Duoji_GetClawPreset(0)==120 && Duoji_GetClawPreset(1)==210);
  reset();request(WEB_CMD_HELLO,1,1);for(unsigned i=0;i<4;++i)Web_Put16(preset_bytes+2*i,calibrated[i]);
  request(WEB_CMD_SERVO_PRESETS,1,2,(const int8_t *)preset_bytes,8);
  CHECK(lastResult()==WEB_RESULT_OK && mode==WEB_MODE_IDLE && claw_count==0 && gimbal_count==0);
  int8_t calibrate_mode=WEB_MODE_MANUAL;request(WEB_CMD_MODE,1,3,&calibrate_mode,1);
  request(WEB_CMD_JOG,1,4,jog_values,4);request(WEB_CMD_SERVO_PRESETS,1,5,(const int8_t *)preset_bytes,8);
  CHECK(lastResult()==WEB_RESULT_INTERLOCK && jog_active);

  /* Tray commands use their own PWM channel and status target, with the
   * same replay protection and motion lease as manual servo commands. */
  reset();manual();clock_ms+=WEB_STATUS_PERIOD_MS;request(WEB_CMD_HEARTBEAT,1,3);
  CHECK(replies.back().length==WEB_TASK_STATUS_SIZE && Web_Get16(replies.back().payload+28)==WEB_TRAY_UNKNOWN_ANGLE);
  trayRequest(0,4);CHECK(lastResult()==WEB_RESULT_OK && tray_target==0 && last_tray_angle==0 && tray_count==1 && tray_pwm_count==1 && gimbal_count==0 && claw_count==0);
  uint32_t tray_lease=link_tick;trayRequest(0,4);CHECK(lastResult()==WEB_RESULT_OK && tray_count==1 && link_tick==tray_lease);
  trayRequest(270,5);CHECK(lastResult()==WEB_RESULT_OK && tray_target==270 && last_tray_angle==270 && tray_count==2 && tray_pwm_count==1);
  trayRequest(271,6);trayRequest(65535,7);CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && tray_count==2 && tray_target==270);
  request(WEB_CMD_TRAY_ANGLE,1,8,&open,1);CHECK(lastResult()==WEB_RESULT_BAD_ARGUMENT && tray_count==2);
  trayRequest(135,9,999);CHECK(lastResult()==WEB_RESULT_STALE && tray_count==2);
  clock_ms+=WEB_STATUS_PERIOD_MS;request(WEB_CMD_HEARTBEAT,1,10);
  CHECK(replies.back().command==WEB_CMD_STATUS && replies.back().length==WEB_TASK_STATUS_SIZE && Web_Get16(replies.back().payload+28)==270);
  request(WEB_CMD_STOP,1,11);CHECK(mode==WEB_MODE_IDLE && tray_target==270);
  trayRequest(135,12);CHECK(lastResult()==WEB_RESULT_WRONG_MODE && tray_count==2);
  reset();manual();request(WEB_CMD_JOG,1,3,jog_values,4);trayRequest(135,4);
  CHECK(lastResult()==WEB_RESULT_INTERLOCK && tray_count==0 && jog_active);
  reset();manual();motor_tick=clock_ms;request(WEB_CMD_JOG,1,3,jog_values,4);CHECK(pending_jog_valid);
  trayRequest(135,4);CHECK(lastResult()==WEB_RESULT_INTERLOCK && tray_count==0);
  reset();manual();manual_lift_active=1U;trayRequest(135,3);CHECK(tray_count==0 && mode==WEB_MODE_IDLE);
  reset();manual();fault=WEB_FAULT_INTERLOCK;trayRequest(135,3);CHECK(lastResult()==WEB_RESULT_INTERLOCK && tray_count==0);
  reset();manual();trayRequest(135,3);taskRequest(4);startRequest(5);
  CHECK(mode==WEB_MODE_AUTO && tray_target==WEB_TRAY_UNKNOWN_ANGLE);
  trayRequest(20,6);CHECK(lastResult()==WEB_RESULT_WRONG_MODE && tray_count==1);
  reset();manual();fail_tray_pwm=true;trayRequest(135,3);
  CHECK(lastResult()==WEB_RESULT_INTERLOCK && tray_count==1 && tray_target==WEB_TRAY_UNKNOWN_ANGLE && mode==WEB_MODE_IDLE && fault==WEB_FAULT_INTERLOCK);
  reset();manual();uint8_t stale_tray[2];Web_Put16(stale_tray,135);
  request(WEB_CMD_TRAY_ANGLE,1,3,(const int8_t *)stale_tray,2,false);clock_ms+=WEB_JOG_LEASE_MS;WebControl_Task();
  CHECK(lastResult()==WEB_RESULT_STALE && tray_count==0 && tray_target==WEB_TRAY_UNKNOWN_ANGLE);
  tray_target=135;WebControl_Init();CHECK(tray_target==WEB_TRAY_UNKNOWN_ANGLE);

  /* A rejected long frame contains two intact short frames. Resynchronizing
   * onto JOG must retain and execute the trailing STOP before motor output. */
  reset();manual();uint8_t nested[WEB_MAX_FRAME];memset(nested,0xCC,sizeof nested);
  nested[0]=0xA5;nested[1]=0x5A;nested[2]=WEB_VERSION;nested[3]=WEB_MAX_PAYLOAD;
  WebFrame nested_jog={};nested_jog.session=1;nested_jog.sequence=3;nested_jog.command=WEB_CMD_JOG;nested_jog.length=4;memcpy(nested_jog.payload,jog_values,4);
  unsigned used=12+Web_Encode(&nested_jog,nested+12);WebFrame nested_stop={};nested_stop.session=1;nested_stop.sequence=4;nested_stop.command=WEB_CMD_STOP;used+=Web_Encode(&nested_stop,nested+used);CHECK(used==42);
  inject(nested,sizeof nested);WebControl_Task();CHECK(mode==WEB_MODE_IDLE && !jog_active && highest_sequence==4 && go_count==0);

  /* Independent signed wheel bases from the manual car calibration, not a
   * comparison against the same legacy mixer which caused the mismatch.
   * Forward remains unchanged; lateral and yaw retain their own speed scale.
   * This is a manual-web test, not a claim about automatic route direction. */
  const int forward_basis[4]={1,-1,1,-1};
  const int right_basis[4]={1,1,1,1};
  const int turn_left_basis[4]={-1,-1,1,1};
  reset();const int8_t levels[]={-50,-20,-1,0,1,20,50};unsigned combinations=0;
  for(unsigned xi=0;xi<7;++xi)for(unsigned yi=0;yi<7;++yi)for(unsigned ti=0;ti<7;++ti){
    jog[0]=levels[xi];jog[1]=levels[yi];jog[2]=levels[ti];jog[3]=0;
    CHECK(apply_motion());
    int actual[4];
    for(unsigned i=0;i<4;++i){
      const double target=levels[xi]*0.6*right_basis[i]+levels[yi]*0.6*forward_basis[i]+levels[ti]*0.255*turn_left_basis[i];
      const int expected=(int)target;
      const int delta=wheel_values[i]-expected;
      CHECK(delta>=-1 && delta<=1); /* float-to-RPM truncation only */
      CHECK(wheel_values[i]==0 || expected==0 || (wheel_values[i]>0)==(expected>0));
      actual[i]=wheel_values[i];
    }
    jog[0]=(int8_t)-levels[xi];jog[1]=(int8_t)-levels[yi];jog[2]=(int8_t)-levels[ti];
    CHECK(apply_motion());
    for(unsigned i=0;i<4;++i)CHECK(wheel_values[i]==-actual[i]);
    ++combinations;
  }
  CHECK(combinations==343);
  printf("Calibrated manual wheel-basis comparisons: %u vectors and reversals\n",combinations);
}
int main(){tests();printf("STM32 command-sequence review: %u failure(s)\n",failures);return failures?1:0;}
