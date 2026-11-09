#include "turntable_radio.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <atomic>

namespace table_now {
Radio *Radio::instance_=nullptr;
static std::atomic<bool> sending{false};
// Deducing the first argument supports both old MAC and newer TX-info APIs.
template<typename Arg> struct TxCallback {
    static void done(Arg,esp_now_send_status_t) { sending.store(false); }
};
template<typename Arg> static esp_err_t registerTx(void (*)(Arg,esp_now_send_status_t)) {
    return esp_now_register_send_cb(TxCallback<Arg>::done);
}
void Radio::accept(const uint8_t *mac,const uint8_t *data,int length) {
    Radio *r=instance_;
    if (!r || !mac || !data || length!=(int)sizeof(Packet) || memcmp(mac,r->peer_,6)) return;
    Received received; memcpy(&received.packet,data,sizeof(Packet)); received.at=millis();
    if (!validPacket(received.packet)) return;
    if (xQueueSend(r->queue_,&received,0)!=pdTRUE) r->overflow_.store(true);
}
bool Radio::begin(const uint8_t peer[6],uint8_t channel) {
    Serial.printf("ESP-NOW STA MAC: %s; channel=%u\n",WiFi.macAddress().c_str(),channel);
    bool allZero=true,allFF=true;
    for (unsigned i=0;i<6;++i) { allZero &= peer[i]==0; allFF &= peer[i]==0xFF; }
    if (allZero || allFF || (peer[0]&1U) || !channel || channel>13 || instance_) return false;
    memcpy(peer_,peer,6);
    queue_=xQueueCreate(16,sizeof(Received));
    if (!queue_) return false;
    if (!WiFi.setSleep(false) || esp_wifi_set_channel(channel,WIFI_SECOND_CHAN_NONE)!=ESP_OK ||
        esp_now_init()!=ESP_OK) { vQueueDelete(queue_); queue_=nullptr; return false; }
    instance_=this;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    auto receiveCallback=[](const esp_now_recv_info_t *info,const uint8_t *data,int len) {
        if (info) Radio::accept(info->src_addr,data,len);
    };
#else
    auto receiveCallback=[](const uint8_t *mac,const uint8_t *data,int len) { Radio::accept(mac,data,len); };
#endif
    esp_now_peer_info_t target={}; memcpy(target.peer_addr,peer_,6);
    target.channel=channel; target.ifidx=WIFI_IF_STA; target.encrypt=false;
    if (esp_now_register_recv_cb(receiveCallback)!=ESP_OK ||
        registerTx((esp_now_send_cb_t)nullptr)!=ESP_OK || esp_now_add_peer(&target)!=ESP_OK) {
        esp_now_deinit(); instance_=nullptr; vQueueDelete(queue_); queue_=nullptr; return false;
    }
    return true;
}
bool Radio::send(const Packet &p) {
    if (!queue_) return false;
    bool expected=false;
    if (!sending.compare_exchange_strong(expected,true)) return false;
    if (esp_now_send(peer_,reinterpret_cast<const uint8_t *>(&p),sizeof(p))==ESP_OK) return true;
    sending.store(false); return false;
}
bool Radio::receive(Packet &p,uint32_t *receivedAt) {
    Received r;
    if (!queue_ || xQueueReceive(queue_,&r,0)!=pdTRUE) return false;
    p=r.packet; if (receivedAt) *receivedAt=r.at; return true;
}
bool Radio::overflowed() { return overflow_.exchange(false); }
} // namespace table_now
