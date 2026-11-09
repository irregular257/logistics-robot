#pragma once
#include "turntable_protocol.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>

namespace table_now {
// One instance per ESP32. Call begin after WiFi.mode/AP creation.
class Radio {
public:
    bool begin(const uint8_t peer[6],uint8_t channel);
    bool send(const Packet &packet);
    bool receive(Packet &packet,uint32_t *receivedAt=nullptr);
    bool overflowed();
private:
    QueueHandle_t queue_=nullptr;
    uint8_t peer_[6]={};
    struct Received { Packet packet; uint32_t at; };
    std::atomic<bool> overflow_{false};
    static Radio *instance_;
    static void accept(const uint8_t *mac,const uint8_t *data,int length);
};
} // namespace table_now
