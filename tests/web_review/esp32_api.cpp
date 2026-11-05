#include <stdio.h>
#include <string.h>
#include "../../ESP32/robot_WebControl/robot_WebControl.ino"
uint32_t test_ms=100;
static unsigned failures;
#define CHECK(cond) do{if(!(cond)){printf("FAIL %s:%d: %s\n",__FILE__,__LINE__,#cond);++failures;}}while(0)
static void clearRequest(){server.args.clear();server.headers.clear();server.code=0;server.response="";}
static void auth(uint32_t sequence=0){clearRequest();char value[17];snprintf(value,sizeof value,"%016llx",(unsigned long long)control.token);server.headers["X-Token"]=value;if(sequence)server.args["seq"]=String(sequence);}
static void feedback(uint8_t command,uint16_t sequence,uint8_t result=WEB_RESULT_OK,uint8_t state_mode=WEB_MODE_MANUAL,uint8_t state_fault=0){
  WebFrame f={};f.session=uartSession;f.sequence=sequence;f.command=WEB_CMD_ACK;f.length=4;f.payload[0]=command;f.payload[1]=result;f.payload[2]=state_mode;f.payload[3]=state_fault;receiveFrame(f);
}
static void status(uint16_t seq=0,uint8_t mode_value=WEB_MODE_IDLE,uint8_t fault_value=0){
  WebFrame f={};f.session=uartSession;f.sequence=seq;f.command=WEB_CMD_STATUS;f.length=8;f.payload[0]=mode_value;f.payload[1]=fault_value;receiveFrame(f);
}
static void extendedStatus(uint16_t seq,uint8_t length=18,uint16_t angle=120,uint8_t mode_value=WEB_MODE_MANUAL,uint16_t tray_target=85){
  WebFrame f={};f.session=uartSession;f.sequence=seq;f.command=WEB_CMD_STATUS;f.length=length;
  f.payload[0]=mode_value;f.payload[8]=3;f.payload[9]=1;f.payload[11]=1;
  Web_Put16(f.payload+12,160);Web_Put16(f.payload+14,170);Web_Put16(f.payload+16,angle);
  if(length>=28){Web_Put16(f.payload+18,45);for(int i=0;i<4;++i)Web_Put16(f.payload+20+2*i,servoPresets[i]);}
  if(length>=30)Web_Put16(f.payload+28,tray_target);
  if(length==32){f.payload[30]=1;f.payload[31]=1;}
  receiveFrame(f);
}
static void reset(bool with_status=true){test_ms=100;control.clear();StmSerial.tx.clear();StmSerial.rx.clear();heartbeatAt=0;lastFault=0;savedPresetsValid=false;presetStorageReady=true;servoPreferences.writes=0;servoPreferences.failWrite=false;uint16_t defaults[]={10,70,54,170};memcpy(servoPresets,defaults,sizeof defaults);beginSession();feedback(WEB_CMD_HELLO,1);if(with_status)status();clearRequest();}
static void claimed(){handleClaim();CHECK(server.code==200 && control.token && !control.armed);}
static void modeRequest(uint32_t sequence=1){auth(sequence);server.args["value"]="1";handleMode();CHECK(server.code==202 && control.armPending && !control.armed);}
static void armed(){claimed();modeRequest();feedback(WEB_CMD_MODE,control.armSequence);CHECK(control.armed);status(1,WEB_MODE_MANUAL);}
static void jogRequest(uint32_t seq,int y){auth(seq);server.args["x"]="0";server.args["y"]=String(y);server.args["turn"]="0";server.args["lift"]="0";handleJog();}
static unsigned countSent(uint8_t cmd){unsigned n=0;for(auto &wire:StmSerial.tx)if(wire[10]==cmd)++n;return n;}
static void gimbalRequest(uint32_t seq,const char *angle){auth(seq);server.args["angle"]=angle;server.routes["/api/gimbal"]();}
static void clawRequest(uint32_t seq,int value){auth(seq);server.args["value"]=String(value);handleClaw();}
static void clawAngleRequest(uint32_t seq,const char *angle){auth(seq);server.args["angle"]=angle;server.routes["/api/claw-angle"]();}
static void trayRequest(uint32_t seq,const char *angle){auth(seq);server.args["angle"]=angle;server.routes["/api/tray-angle"]();}
static void presetRequest(uint32_t seq,const char *retract="240"){auth(seq);server.args["clawClose"]="20";server.args["clawOpen"]="125";server.args["gimbalIn"]=retract;server.args["gimbalOut"]="170";server.routes["/api/servo-presets"]();}
static void anglePresetTests(){
  reset();armed();extendedStatus(2,28,270);
  CHECK(servoPresetsSupported && gimbalSupported && gimbalAngle==270 && clawAngle==45);
  auth();handleStatus();CHECK(strstr(server.response.c_str(),"\"servoPresetsSupported\":true") && strstr(server.response.c_str(),"\"clawOpen\":70"));
  gimbalRequest(2,"0");CHECK(server.code==202 && Web_Get16(StmSerial.tx.back().data()+11)==0);
  gimbalRequest(3,"270");CHECK(server.code==202 && Web_Get16(StmSerial.tx.back().data()+11)==270);
  gimbalRequest(4,"271");CHECK(server.code==400 && countSent(WEB_CMD_GIMBAL)==2);
  clawAngleRequest(5,"125");CHECK(server.code==202 && countSent(WEB_CMD_CLAW_ANGLE)==1 && Web_Get16(StmSerial.tx.back().data()+11)==125 && clawAngle==45);
  clawAngleRequest(6,"-1");CHECK(server.code==400 && countSent(WEB_CMD_CLAW_ANGLE)==1);
  clawAngleRequest(7,"270.5");CHECK(server.code==400);
  stmJog[3]=1;clawAngleRequest(8,"125");CHECK(server.code==409);presetRequest(9);CHECK(server.code==409 && !presetsPending);stmJog[3]=0;
  presetRequest(10);CHECK(server.code==202 && presetsPending && servoPreferences.writes==0 && servoPresets[2]==54 && !presetsSaved());
  CHECK(StmSerial.tx.back().size()==21 && Web_Get16(StmSerial.tx.back().data()+11)==20 && Web_Get16(StmSerial.tx.back().data()+15)==240);
  uint16_t sequence=presetApplySequence;
  feedback(WEB_CMD_SERVO_PRESETS,(uint16_t)(sequence+1));CHECK(presetsPending && servoPreferences.writes==0);
  feedback(WEB_CMD_SERVO_PRESETS,sequence);CHECK(!presetsPending && servoPreferences.writes==1 && savedPresetsValid && savedPresets[2]==240 && servoPresets[2]==54);
  memcpy(servoPresets,savedPresets,sizeof servoPresets);extendedStatus(3,28);CHECK(presetsSaved());
  CHECK(countSent(WEB_CMD_CLAW)==0 && countSent(WEB_CMD_CLAW_ANGLE)==1 && countSent(WEB_CMD_GIMBAL)==2); // Saving itself sends no movement command.
  presetRequest(11,"271");CHECK(server.code==400 && countSent(WEB_CMD_SERVO_PRESETS)==1);
  reset();armed();extendedStatus(2,18);clawAngleRequest(2,"125");CHECK(server.code==409);presetRequest(3);CHECK(server.code==409 && !presetsPending);
  reset();armed();extendedStatus(2,28);presetRequest(2);sequence=presetApplySequence;feedback(WEB_CMD_SERVO_PRESETS,sequence,WEB_RESULT_INTERLOCK);CHECK(!presetsPending && servoPreferences.writes==0 && !savedPresetsValid);
  reset();armed();extendedStatus(2,28);presetRequest(2);sequence=presetApplySequence;test_ms+=1200;servicePresets();CHECK(!presetsPending && servoPreferences.writes==0);feedback(WEB_CMD_SERVO_PRESETS,sequence);CHECK(servoPreferences.writes==0);
  reset();armed();extendedStatus(2,28);servoPreferences.failWrite=true;presetRequest(2);feedback(WEB_CMD_SERVO_PRESETS,presetApplySequence);CHECK(!presetStorageReady && !savedPresetsValid);presetRequest(3);CHECK(server.code==503);
  reset();armed();extendedStatus(2,28);stmMode=WEB_MODE_AUTO;presetRequest(2);CHECK(server.code==409 && !presetsPending);
}
static void presetRestoreTests(){
  reset();uint16_t saved[]={20,125,240,170};memcpy(savedPresets,saved,sizeof saved);savedPresetsValid=true;
  extendedStatus(1,28,54,WEB_MODE_IDLE);servicePresets();CHECK(presetsPending && countSent(WEB_CMD_SERVO_PRESETS)==1 && servoPreferences.writes==0);
  handleClaim();CHECK(server.code==409 && !control.token);
  feedback(WEB_CMD_SERVO_PRESETS,presetApplySequence);CHECK(presetRestoreDone && !presetsPending && servoPreferences.writes==0);
  memcpy(servoPresets,savedPresets,sizeof servoPresets);extendedStatus(2,28,54,WEB_MODE_IDLE);servicePresets();CHECK(countSent(WEB_CMD_SERVO_PRESETS)==1);
  claimed();CHECK(presetRestoreDone);stopAndRevoke();uint16_t defaults[]={10,70,54,170};memcpy(servoPresets,defaults,sizeof servoPresets);servicePresets();CHECK(countSent(WEB_CMD_SERVO_PRESETS)==1);
  reset();memcpy(savedPresets,saved,sizeof saved);savedPresetsValid=true;
  for(int i=0;i<3;++i){extendedStatus((uint16_t)(1+i),28,54,WEB_MODE_IDLE);servicePresets();CHECK(presetsPending);test_ms+=1500;servicePresets();CHECK(!presetsPending);}
  extendedStatus(4,28,54,WEB_MODE_IDLE);servicePresets();CHECK(countSent(WEB_CMD_SERVO_PRESETS)==3 && servoPreferences.writes==0);
  reset();memcpy(savedPresets,saved,sizeof saved);savedPresetsValid=true;test_ms+=10001;extendedStatus(1,28,54,WEB_MODE_IDLE);servicePresets();CHECK(!presetsPending && countSent(WEB_CMD_SERVO_PRESETS)==0);
}
static void servoFeedbackTests(){
  reset();armed();extendedStatus(2);
  stmJog[1]=20;clawRequest(2,1);CHECK(server.code==409 && countSent(WEB_CMD_CLAW)==0);
  gimbalRequest(3,"120");CHECK(server.code==409 && countSent(WEB_CMD_GIMBAL)==0);
  stmJog[1]=0;clawRequest(4,1);CHECK(server.code==202 && countSent(WEB_CMD_CLAW)==1);
  uint16_t seq=operationSequence;feedback(WEB_CMD_CLAW,seq);CHECK(ackReceived && lastAckCommand==WEB_CMD_CLAW && lastAckSequence==seq);
  auth();handleStatus();CHECK(strstr(server.response.c_str(),"\"ackValid\":true"));
  jogRequest(5,20);clawRequest(6,0);CHECK(server.code==409 && countSent(WEB_CMD_CLAW)==1);
  jogRequest(7,0);clawRequest(8,0);CHECK(server.code==202 && countSent(WEB_CMD_CLAW)==2);
}
static void trayTests(){
  CHECK(server.routes.count("/api/tray-angle")==1);
  reset();armed();extendedStatus(2,30,270,WEB_MODE_MANUAL,85);
  CHECK(traySupported && trayAngle==85 && servoPresetsSupported && gimbalSupported && gimbalAngle==270 && clawAngle==45);
  auth();handleStatus();CHECK(strstr(server.response.c_str(),"\"traySupported\":true") && strstr(server.response.c_str(),"\"trayAngle\":85"));
  trayRequest(2,"0");CHECK(server.code==202 && countSent(WEB_CMD_TRAY_ANGLE)==1);
  CHECK(StmSerial.tx.back().size()==15 && StmSerial.tx.back()[3]==2 && StmSerial.tx.back()[10]==0x0D && Web_Get16(StmSerial.tx.back().data()+11)==0);
  CHECK(trayAngle==85 && !ackReceived); // Queuing cannot stand in for a measured or accepted angle.
  uint16_t sentSequence=operationSequence;
  feedback(WEB_CMD_GIMBAL,sentSequence);CHECK(!ackReceived);
  feedback(WEB_CMD_TRAY_ANGLE,(uint16_t)(sentSequence+1));CHECK(!ackReceived);
  feedback(WEB_CMD_TRAY_ANGLE,sentSequence);CHECK(ackReceived && lastAckCommand==WEB_CMD_TRAY_ANGLE && lastAckSequence==sentSequence);
  trayRequest(3,"270");CHECK(server.code==202 && Web_Get16(StmSerial.tx.back().data()+11)==270 && countSent(WEB_CMD_TRAY_ANGLE)==2);
  unsigned sent=countSent(WEB_CMD_TRAY_ANGLE);
  const char *invalid[]={"-1","271","270.5","", "120x"," 120","120 ","+120","2147483648"};
  uint32_t seq=4;
  for(const char *value:invalid){trayRequest(seq++,value);CHECK(server.code==400 && countSent(WEB_CMD_TRAY_ANGLE)==sent);}
  clearRequest();server.args["seq"]=String(seq);server.args["angle"]="120";handleTrayAngle();CHECK(server.code==403 && countSent(WEB_CMD_TRAY_ANGLE)==sent);
  trayRequest(seq,"120");CHECK(server.code==202);sent=countSent(WEB_CMD_TRAY_ANGLE);
  trayRequest(seq++,"130");CHECK(server.code==409 && countSent(WEB_CMD_TRAY_ANGLE)==sent);
  auth();server.args["seq"]="1.5";server.args["angle"]="120";handleTrayAngle();CHECK(server.code==400);
  gimbalRequest(seq++,"0");CHECK(server.code==202 && Web_Get16(StmSerial.tx.back().data()+11)==0);
  clawAngleRequest(seq++,"270");CHECK(server.code==202 && Web_Get16(StmSerial.tx.back().data()+11)==270);
  presetRequest(seq++);CHECK(server.code==202 && presetsPending);feedback(WEB_CMD_SERVO_PRESETS,presetApplySequence);CHECK(!presetsPending && servoPreferences.writes==1);
  stmJog[3]=1;trayRequest(seq++,"120");CHECK(server.code==409 && countSent(WEB_CMD_TRAY_ANGLE)==sent);stmJog[3]=0;
  stmJog[1]=1;trayRequest(seq++,"120");CHECK(server.code==409 && countSent(WEB_CMD_TRAY_ANGLE)==sent);stmJog[1]=0;
  control.moving=true;trayRequest(seq++,"120");CHECK(server.code==409 && countSent(WEB_CMD_TRAY_ANGLE)==sent);control.moving=false;
  stmFault=1;trayRequest(seq++,"120");CHECK(server.code==409 && countSent(WEB_CMD_TRAY_ANGLE)==sent);stmFault=0;
  stmMode=WEB_MODE_AUTO;trayRequest(seq++,"120");CHECK(server.code==409 && countSent(WEB_CMD_TRAY_ANGLE)==sent);stmMode=WEB_MODE_MANUAL;
  extendedStatus(3,30,120,WEB_MODE_MANUAL,270);CHECK(traySupported && trayAngle==270);
  extendedStatus(2,30,120,WEB_MODE_MANUAL,0);CHECK(trayAngle==270); // Out-of-order feedback cannot overwrite the new target.
  extendedStatus(4,30,120,WEB_MODE_MANUAL,0xFFFF);CHECK(traySupported && trayAngle==-1);
  auth();handleStatus();CHECK(strstr(server.response.c_str(),"\"trayAngle\":-1"));
  extendedStatus(5,30,120,WEB_MODE_MANUAL,271);CHECK(traySupported && trayAngle==-1);
  extendedStatus(6,28);CHECK(!traySupported && trayAngle==-1 && servoPresetsSupported);
  trayRequest(seq++,"120");CHECK(server.code==409 && strstr(server.response.c_str(),"转盘控制固件") && countSent(WEB_CMD_TRAY_ANGLE)==sent);
  auth();handleStatus();CHECK(strstr(server.response.c_str(),"\"traySupported\":false") && strstr(server.response.c_str(),"\"trayAngle\":-1"));
  for(uint8_t length:{8,16,18}){extendedStatus((uint16_t)(7+length),length);trayRequest(seq++,"120");CHECK(server.code==409 && !traySupported && trayAngle==-1 && countSent(WEB_CMD_TRAY_ANGLE)==sent);}
  extendedStatus(30,30);beginSession();CHECK(!traySupported && trayAngle==-1 && !servoPresetsSupported);
  reset();claimed();extendedStatus(1,30);trayRequest(1,"120");CHECK(server.code==409 && countSent(WEB_CMD_TRAY_ANGLE)==0);
  modeRequest(2);trayRequest(3,"120");CHECK(server.code==409 && countSent(WEB_CMD_TRAY_ANGLE)==0);
  reset();armed();extendedStatus(2,30);test_ms+=600;control.lastSeen=test_ms;trayRequest(2,"120");CHECK(server.code==503 && countSent(WEB_CMD_TRAY_ANGLE)==0);
  reset();armed();extendedStatus(2,30);test_ms+=850;trayRequest(2,"120");CHECK(server.code==403 && !control.token && countSent(WEB_CMD_TRAY_ANGLE)==0);
}
static void gimbalTests(){
  reset();armed();extendedStatus(2);
  CHECK(gimbalSupported && gimbalAngle==120 && autoSupported && autoState==3 && autoDrop==1 && autoXcm==160 && autoYcm==170);
  auth();handleStatus();CHECK(strstr(server.response.c_str(),"\"gimbalSupported\":true") && strstr(server.response.c_str(),"\"gimbalAngle\":120"));
  gimbalRequest(2,"54");CHECK(server.code==202 && countSent(WEB_CMD_GIMBAL)==1);
  CHECK(StmSerial.tx.back().size()==15 && StmSerial.tx.back()[3]==2 && StmSerial.tx.back()[10]==WEB_CMD_GIMBAL && StmSerial.tx.back()[11]==54 && StmSerial.tx.back()[12]==0);
  CHECK(gimbalAngle==120); // A queued command must not pretend the STM32 has moved the servo.
  gimbalRequest(3,"229");CHECK(server.code==202 && countSent(WEB_CMD_GIMBAL)==2 && StmSerial.tx.back()[11]==229 && StmSerial.tx.back()[12]==0);
  unsigned sent=countSent(WEB_CMD_GIMBAL);
  gimbalRequest(4,"53");CHECK(server.code==400 && countSent(WEB_CMD_GIMBAL)==sent);
  gimbalRequest(5,"230");CHECK(server.code==400 && countSent(WEB_CMD_GIMBAL)==sent);
  gimbalRequest(6,"120.5");CHECK(server.code==400 && countSent(WEB_CMD_GIMBAL)==sent);
  gimbalRequest(7,"");CHECK(server.code==400 && countSent(WEB_CMD_GIMBAL)==sent);
  gimbalRequest(8,"120x");CHECK(server.code==400 && countSent(WEB_CMD_GIMBAL)==sent);
  clearRequest();server.args["seq"]="9";server.args["angle"]="120";handleGimbal();CHECK(server.code==403 && countSent(WEB_CMD_GIMBAL)==sent);
  jogRequest(9,20);CHECK(control.moving);gimbalRequest(10,"120");CHECK(server.code==409 && countSent(WEB_CMD_GIMBAL)==sent);
  jogRequest(11,0);gimbalRequest(12,"120");CHECK(server.code==202);
  extendedStatus(3,18,229);CHECK(gimbalAngle==229);
  extendedStatus(4,18,0xFFFFU);CHECK(gimbalSupported && gimbalAngle==-1 && autoSupported);
  auth();handleStatus();CHECK(strstr(server.response.c_str(),"\"gimbalAngle\":-1"));
  extendedStatus(5,18,0);CHECK(gimbalSupported && gimbalAngle==-1);
  extendedStatus(6,16);CHECK(!gimbalSupported && gimbalAngle==-1 && autoSupported);
  sent=countSent(WEB_CMD_GIMBAL);gimbalRequest(13,"120");CHECK(server.code==409 && countSent(WEB_CMD_GIMBAL)==sent);
  status(7,WEB_MODE_MANUAL);CHECK(!gimbalSupported && gimbalAngle==-1 && !autoSupported);
  extendedStatus(8);beginSession();CHECK(!gimbalSupported && gimbalAngle==-1 && !autoSupported);
  reset();armed();extendedStatus(2);stmMode=WEB_MODE_AUTO;gimbalRequest(2,"120");CHECK(server.code==409 && countSent(WEB_CMD_GIMBAL)==0);
  reset();claimed();extendedStatus(1);gimbalRequest(1,"120");CHECK(server.code==409 && countSent(WEB_CMD_GIMBAL)==0);
  modeRequest(2);gimbalRequest(3,"120");CHECK(server.code==409 && countSent(WEB_CMD_GIMBAL)==0);
  reset();armed();extendedStatus(2);test_ms+=600;control.lastSeen=test_ms;gimbalRequest(2,"120");CHECK(server.code==503 && countSent(WEB_CMD_GIMBAL)==0);
}
static void tests(){
  reset();claimed();modeRequest();feedback(WEB_CMD_MODE,(uint16_t)(control.armSequence+1));CHECK(!control.armed && control.armPending);
  feedback(WEB_CMD_MODE,control.armSequence);CHECK(control.armed);
  reset();claimed();modeRequest();feedback(WEB_CMD_MODE,control.armSequence,WEB_RESULT_STALE);CHECK(!control.token && !control.armed && countSent(WEB_CMD_STOP)==1);
  reset();claimed();modeRequest();feedback(WEB_CMD_MODE,control.armSequence,WEB_RESULT_OK,WEB_MODE_MANUAL,6);CHECK(!control.token && !control.armed);
  reset();claimed();modeRequest();test_ms+=599;control.lastSeen=test_ms;status(1);feedback(WEB_CMD_MODE,control.armSequence);CHECK(control.armed);
  reset();claimed();modeRequest();test_ms+=600;control.lastSeen=test_ms;status(1);feedback(WEB_CMD_MODE,control.armSequence);CHECK(!control.armed);loop();CHECK(!control.token && countSent(WEB_CMD_STOP)>=1);
  reset();claimed();modeRequest();test_ms+=600;control.lastSeen=test_ms;feedback(WEB_CMD_MODE,control.armSequence);CHECK(!control.armed);loop();CHECK(!control.token);
  reset();claimed();modeRequest();uint16_t pending=control.armSequence;uint64_t old_token=control.token;stopAndRevoke();feedback(WEB_CMD_MODE,pending);CHECK(!control.token && !control.armed);
  clearRequest();char old_text[17];snprintf(old_text,sizeof old_text,"%016llx",(unsigned long long)old_token);server.headers["X-Token"]=old_text;server.args["seq"]="2";server.args["value"]="1";handleMode();CHECK(server.code==403);
  server.args["x"]="0";server.args["y"]="20";server.args["turn"]="0";server.args["lift"]="0";handleJog();CHECK(server.code==403);
  reset();armed();jogRequest(3,0);CHECK(server.code==202 && !control.moving);unsigned sent=countSent(WEB_CMD_JOG);jogRequest(2,20);CHECK(server.code==409 && !control.moving && countSent(WEB_CMD_JOG)==sent);
  reset();armed();jogRequest(2,20);CHECK(server.code==202 && control.moving);uint32_t jog_time=control.lastJog;test_ms+=200;auth();server.routes["/api/ping"]();CHECK(server.code==200 && control.lastJog==jog_time && control.moving);test_ms=jog_time+300;loop();CHECK(!control.token && !control.moving && countSent(WEB_CMD_STOP)>=1);
  reset();claimed();test_ms+=850;auth();server.routes["/api/ping"]();CHECK(server.code==403 && !control.token);
  reset(false);uint32_t session=uartSession;test_ms+=600;loop();CHECK(uartSession!=session && !helloAccepted && !statusReceived && countSent(WEB_CMD_HELLO)>=2);
  reset();armed();status(65535,WEB_MODE_MANUAL);CHECK(statusSequence==1); // Old half-range status cannot rewrite state.
  haveStatusSequence=false;status(65535,WEB_MODE_MANUAL);status(0,WEB_MODE_MANUAL);CHECK(statusSequence==0);
  reset();armed();status(2,WEB_MODE_IDLE);CHECK(!control.token && !control.armed);
  reset();armed();status(2,WEB_MODE_MANUAL,9);CHECK(!control.token && lastFault==9 && stmFault==9);status(3,WEB_MODE_IDLE,0);CHECK(stmFault==0 && lastFault==9);auth();handleStatus();CHECK(strstr(server.response.c_str(),"\"lastFault\":9") && strstr(server.response.c_str(),"\"fault\":0"));
  reset();armed();uint8_t p[]={0,0,0,0};uartSequence=65535;uint16_t wrapped=sendFrame(WEB_CMD_JOG,p,4);CHECK(wrapped==0);feedback(WEB_CMD_JOG,0);CHECK(ackReceived && lastAckSequence==0);
  int32_t value=0;CHECK(!parseInteger("20x",-50,50,value) && !parseInteger("",-50,50,value) && !parseInteger("2147483649",INT32_MIN,INT32_MAX,value));CHECK(parseInteger("-2147483648",INT32_MIN,INT32_MAX,value)&&value==INT32_MIN);
  reset();armed();jogRequest(2,20);auth(3);server.args["x"]="0";server.args["y"]="garbage";server.args["turn"]="0";server.args["lift"]="0";handleJog();CHECK(server.code==400);
  reset();claimed();autoSupported=true;auth(1);handleStart();CHECK(server.code==409 && countSent(WEB_CMD_START)==0);
  auth(2);server.args["positioned"]="1";handleStart();CHECK(server.code==409 && countSent(WEB_CMD_START)==0 && !control.armed);
  status(1,WEB_MODE_AUTO);CHECK(control.token && !control.armed);
  jogRequest(3,20);CHECK(server.code==409);
  auth(4);handlePause();CHECK(server.code==202 && countSent(WEB_CMD_PAUSE)==1);
  reset();claimed();autoSupported=false;auth(1);server.args["positioned"]="1";handleStart();CHECK(server.code==409 && countSent(WEB_CMD_START)==0);
}
static void planArgs(uint32_t sequence){
  auth(sequence);
  server.args["color1"]="4";server.args["shape1"]="1";
  server.args["color2"]="7";server.args["shape2"]="4";
  server.args["color3"]="3";server.args["shape3"]="3";
}
static void taskTests(){
  reset();claimed();extendedStatus(1,32,120,WEB_MODE_IDLE);
  CHECK(taskSupported && stmTaskReady && traySupported && !taskSubmitted);
  planArgs(1);server.args["positioned"]="1";handleStart();CHECK(server.code==409 && countSent(WEB_CMD_START)==0);
  planArgs(2);handleTask();CHECK(server.code==202 && taskPending && !taskSubmitted && countSent(WEB_CMD_TASK)==1);
  uint8_t expected[]={4,1,7,4,3,3};CHECK(StmSerial.tx.back()[3]==6 && memcmp(StmSerial.tx.back().data()+11,expected,6)==0);
  uint16_t sent=taskApplySequence;
  feedback(WEB_CMD_TASK,(uint16_t)(sent+1));CHECK(taskPending && !taskSubmitted);
  feedback(WEB_CMD_CLAW,sent);CHECK(taskPending && !taskSubmitted);
  planArgs(3);handleTask();CHECK(server.code==409 && countSent(WEB_CMD_TASK)==1);
  feedback(WEB_CMD_TASK,sent);CHECK(!taskPending && taskSubmitted && memcmp(taskPlan,expected,6)==0);
  auth();handleStatus();CHECK(strstr(server.response.c_str(),"\"taskAccepted\":true") && strstr(server.response.c_str(),"\"taskPlan\":[4,1,7,4,3,3]"));
  planArgs(4);server.args["positioned"]="1";server.args["color2"]="1";handleStart();CHECK(server.code==409 && countSent(WEB_CMD_START)==0);
  planArgs(5);server.args["positioned"]="1";handleStart();CHECK(server.code==202 && countSent(WEB_CMD_START)==1);
  CHECK(StmSerial.tx.back()[3]==7 && StmSerial.tx.back()[11]==1 && memcmp(StmSerial.tx.back().data()+12,expected,6)==0);
  stmMode=WEB_MODE_AUTO;planArgs(6);handleTask();CHECK(server.code==409 && countSent(WEB_CMD_TASK)==1);stmMode=WEB_MODE_IDLE;
  uint32_t seq=7;
  for(const char* bad:{"0","2","7","-1","3.5","","3x"}){planArgs(seq++);server.args["shape1"]=bad;handleTask();CHECK(server.code==400 && countSent(WEB_CMD_TASK)==1);}
  for(const char* bad:{"0","5","6","8","-1","1.5",""}){planArgs(seq++);server.args["color3"]=bad;handleTask();CHECK(server.code==400 && countSent(WEB_CMD_TASK)==1);}
  planArgs(seq++);server.args["color1"]="1";handleTask();CHECK(server.code==400); // Red hexagon absent from model.
  planArgs(seq++);server.args["color3"]="7";handleTask();CHECK(server.code==400); // Yellow cylinder absent from model.
  planArgs(seq++);server.args["shape3"]="5";server.args["color3"]="7";handleTask();CHECK(server.code==202);feedback(WEB_CMD_TASK,taskApplySequence,WEB_RESULT_BAD_ARGUMENT);CHECK(!taskPending && !taskSubmitted);
  planArgs(seq++);handleTask();CHECK(server.code==202);sent=taskApplySequence;test_ms+=1500;serviceTask();CHECK(!taskPending && !taskSubmitted);feedback(WEB_CMD_TASK,sent);CHECK(!taskSubmitted);
  test_ms=100;control.lastSeen=test_ms;extendedStatus(2,32,120,WEB_MODE_IDLE);planArgs(seq++);handleTask();feedback(WEB_CMD_TASK,taskApplySequence);CHECK(taskSubmitted);
  beginSession();CHECK(!taskSubmitted && !taskPending && !taskSupported && !stmTaskReady);
  reset();claimed();extendedStatus(1,30,120,WEB_MODE_IDLE);planArgs(1);handleTask();CHECK(server.code==409 && countSent(WEB_CMD_TASK)==0);
  reset();claimed();extendedStatus(1,32,120,WEB_MODE_IDLE);stmJog[3]=1;planArgs(1);handleTask();CHECK(server.code==409 && countSent(WEB_CMD_TASK)==0);stmJog[3]=0;
  planArgs(2);handleTask();sent=taskApplySequence;stopAndRevoke();feedback(WEB_CMD_TASK,sent);CHECK(!taskPending && !taskSubmitted);
}
int main(){setup();tests();gimbalTests();servoFeedbackTests();anglePresetTests();presetRestoreTests();trayTests();taskTests();printf("ESP32 API integration review: %u failure(s)\n",failures);return failures?1:0;}
