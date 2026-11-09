/* External six-position material turntable, ESP32-S3 + A4988 driver.
 * Install ../libraries/TurntableNow as an Arduino sketchbook library.
 * USB serial: A = confirm current mechanical index, X = stop/fault.
 * No motion occurs at boot. Configure peer STA MAC before use.
 * A4988 VDD=3.3V; RESET/SLEEP tied HIGH; VMOT supplied separately.
 * Default 1/16 microstepping: wire MS1/MS2/MS3 to VDD (not floating).
 */
#include <Arduino.h>
#include <WiFi.h>
#include <esp_system.h>
#include <turntable_protocol.h>
#include <turntable_radio.h>
#include "stepper42.h"

constexpr uint8_t WIFI_CHANNEL=1;
// Fill in the car's STA MAC, NOT the hotspot/BSSID MAC.
constexpr uint8_t CAR_MAC[6]={0,0,0,0,0,0};
constexpr int STEP_PIN=4,DIR_PIN=5,ENABLE_PIN=6;
// A4988 ENABLE is active LOW. Keep enabled while stationary for holding torque.
constexpr bool FORWARD=true,ENABLE_ACTIVE_LOW=true;
// 42mm is a frame size, not a step angle. Verify the motor's step-angle label.
// 1.8deg motor * 16 microsteps * 1:1 transmission. MS pins are wired externally;
// changing MICROSTEPS here does NOT configure the A4988's physical MS pins.
constexpr uint32_t MOTOR_FULL_STEPS=200,MICROSTEPS=16;
static_assert(MICROSTEPS==1 || MICROSTEPS==2 || MICROSTEPS==4 ||
              MICROSTEPS==8 || MICROSTEPS==16,"A4988 supports 1,2,4,8,16 microsteps only");
constexpr uint32_t GEAR_NUMERATOR=1,GEAR_DENOMINATOR=1;
static_assert(GEAR_DENOMINATOR &&
    (uint64_t(MOTOR_FULL_STEPS)*MICROSTEPS*GEAR_NUMERATOR)%GEAR_DENOMINATOR==0,
    "Configure an integer pulse count per output revolution");
constexpr uint32_t OUTPUT_STEPS_PER_REV=
    uint64_t(MOTOR_FULL_STEPS)*MICROSTEPS*GEAR_NUMERATOR/GEAR_DENOMINATOR;
static_assert(OUTPUT_STEPS_PER_REV>=6,"Too few pulses per output revolution");

table_now::Radio radio;
table_now::TableEngine engine(1);
table_now::SectorSteps sectors(OUTPUT_STEPS_PER_REV);
Stepper42 motor;
bool radioReady=false,motorReady=false,moveStarted=false,replyPending=false;
uint32_t reportAt=0,moveAt=0;
table_now::Packet replyPacket;

uint32_t newBootId() { uint32_t v; do { v=esp_random(); } while (!v); return v; }
void setup() {
    Serial.begin(115200);
    motorReady=motor.begin(STEP_PIN,DIR_PIN,ENABLE_PIN,FORWARD,ENABLE_ACTIVE_LOW);
    WiFi.mode(WIFI_STA);
    radioReady=radio.begin(CAR_MAC,WIFI_CHANNEL);
    engine=table_now::TableEngine(newBootId());
    if (!radioReady || !motorReady) engine.fault();
    Serial.println("Align a tray position to the camera, then send A to confirm. X stops.");
    Serial.printf("A4988 motor=%u radio=%u microsteps=%lu output pulses/rev=%lu\n",
                  motorReady,radioReady,(unsigned long)MICROSTEPS,
                  (unsigned long)OUTPUT_STEPS_PER_REV);
}
void loop() {
    using namespace table_now;
    // Consume bounded USB input; no parseInt, String or blocking 2-second delay.
    for (uint8_t i=0;i<16 && Serial.available();++i) {
        char c=(char)Serial.read();
        if (c=='x' || c=='X') { engine.fault(); motor.stop(); }
        if ((c=='a' || c=='A') && radioReady && motorReady &&
            (engine.state()==IDLE || engine.state()==FAULT || engine.state()==DONE)) {
            motor.stop(); moveStarted=false; sectors.reset();
            engine=TableEngine(newBootId()); engine.confirmPosition();
            replyPending=false; reportAt=millis()-50U;
            Serial.println("Position confirmed; waiting for a new car task.");
        }
    }
    Packet p;
    while (radio.receive(p)) {
        if (p.type==table_now::STATUS) continue;
        replyPacket=engine.handle(p,millis());
        if (p.type!=HEARTBEAT) replyPending=true;
    }
    if (radio.overflowed()) engine.fault();
    uint32_t now=millis(); engine.tick(now);
    if (engine.state()==MOVING) {
        if (!moveStarted) {
            if (!motor.start(sectors.next())) engine.fault();
            else { moveStarted=true; moveAt=now; }
        } else if (!motor.busy()) {
            moveStarted=false; engine.moveComplete(now);
        } else if ((uint32_t)(now-moveAt)>=20000U) engine.fault();
    }
    if (engine.state()==FAULT) { motor.stop(); moveStarted=false; }
    // Refresh cached response state after motor/timeout actions, preserving ACK.
    if (replyPending && engine.state()==FAULT) replyPacket.state=FAULT;
    if (replyPending) {
        if (radio.send(replyPacket)) { replyPending=false; reportAt=now; }
    } else if ((uint32_t)(now-reportAt)>=50U) {
        Packet status=engine.status();
        if (radio.send(status)) reportAt=now;
    }
    delay(1); // Yield to Wi-Fi; motor pulses run on esp_timer.
}
