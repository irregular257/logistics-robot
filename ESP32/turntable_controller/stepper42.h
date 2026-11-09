#pragma once
#include <Arduino.h>
#include <esp_timer.h>
#include <math.h>

// ESP timer generates the edges independently of the networking loop.
// Conservative default: 800 pulses/s, 400 pulses/s^2, 100us minimum pulse.
// Timer-task latency can stretch a pulse/interval; it never adds extra pulses.
// A4988 requires STEP HIGH/LOW >=1us and DIR setup/hold >=200ns.
// DIR stays fixed throughout operation. EN stays LOW to hold between moves.
class Stepper42 {
public:
    bool begin(int stepPin,int dirPin,int enablePin,bool forward,bool enableLow) {
        stepPin_=stepPin; enablePin_=enablePin; enableLevel_=enableLow ? LOW : HIGH;
        pinMode(stepPin,OUTPUT); digitalWrite(stepPin,LOW);
        pinMode(dirPin,OUTPUT); digitalWrite(dirPin,forward ? HIGH : LOW);
        pinMode(enablePin,OUTPUT); digitalWrite(enablePin,enableLevel_);
        esp_timer_create_args_t args={};
        args.callback=timerCallback; args.arg=this; args.dispatch_method=ESP_TIMER_TASK;
        args.name="table-step"; args.skip_unhandled_events=true;
        return esp_timer_create(&args,&timer_)==ESP_OK && esp_timer_start_periodic(timer_,100)==ESP_OK;
    }
    bool start(uint32_t count) {
        if (!count || count>100000U) return false;
        portENTER_CRITICAL(&mux_);
        bool ok=!remaining_ && !high_;
        if (ok) { total_=count; remaining_=count; wait_=1; }
        portEXIT_CRITICAL(&mux_); return ok;
    }
    bool busy() {
        portENTER_CRITICAL(&mux_); bool v=remaining_ || high_; portEXIT_CRITICAL(&mux_); return v;
    }
    void stop() {
        portENTER_CRITICAL(&mux_);
        remaining_=0; high_=false; digitalWrite(stepPin_,LOW);
        portEXIT_CRITICAL(&mux_);
        // Keep EN asserted: stopping pulses retains holding torque.
    }
private:
    esp_timer_handle_t timer_=nullptr;
    portMUX_TYPE mux_=portMUX_INITIALIZER_UNLOCKED;
    int stepPin_=4,enablePin_=6,enableLevel_=LOW;
    uint32_t total_=0,remaining_=0,wait_=0;
    int64_t lastTick_=0;
    bool high_=false;
    static void timerCallback(void *arg) { static_cast<Stepper42 *>(arg)->tick(); }
    void tick() {
        portENTER_CRITICAL(&mux_);
        // A delayed timer task may dispatch adjacent callbacks close together.
        // Count only ticks separated by real 100us; never emit catch-up bursts.
        int64_t now=esp_timer_get_time();
        if (now-lastTick_<100) { portEXIT_CRITICAL(&mux_); return; }
        lastTick_=now;
        if (high_) { digitalWrite(stepPin_,LOW); high_=false; }
        if (remaining_ && wait_ && --wait_==0) {
            digitalWrite(stepPin_,HIGH); high_=true; --remaining_;
            float accelerating=sqrtf(800.0f*float(total_-remaining_));
            float braking=sqrtf(800.0f*float(remaining_+1));
            float speed=fminf(800.0f,fminf(accelerating,braking));
            wait_=(uint32_t)ceilf(10000.0f/speed);
            if (wait_<2) wait_=2;
        }
        portEXIT_CRITICAL(&mux_);
    }
};
