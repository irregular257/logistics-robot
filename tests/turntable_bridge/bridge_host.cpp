#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <deque>
#include <vector>
#include <turntable_protocol.h>
#include <turntable_radio.h>

uint32_t test_ms = 0;
uint32_t millis() { return test_ms; }
uint32_t esp_random() { static uint32_t n=100; return ++n; }
struct FakeSerial {
  void print(const char *) {} void println(const char *) {}
  template<class... A> void printf(const char *, A...) {}
} Serial;
struct FakeWiFi { const char *macAddress() { return "02:00:00:00:00:01"; } } WiFi;

#define TABLE_NOW_ENABLED 1
#define TABLE_NOW_PEER_MAC_BYTES {2,0,0,0,0,2}
#include "../../ESP32/robot_WebControl/web_protocol.h"
#include "../../ESP32/robot_WebControl/table_bridge.h"

std::deque<table_now::Packet> table_now::Radio::incoming;
std::vector<table_now::Packet> table_now::Radio::outgoing;
bool table_now::Radio::overflow = false;
uint32_t table_now::Radio::receiveAge = 0;
using namespace table_now;

static void request(TableBridge &bridge, uint8_t op, uint32_t task, uint32_t window=0) {
  WebFrame f={}; f.command=WEB_CMD_TABLE_REQUEST; f.length=9;
  f.payload[0]=op; Web_Put32(f.payload+1,task); Web_Put32(f.payload+5,window);
  bridge.onRequest(f);
}
static uint8_t state(TableBridge &bridge, uint8_t *fault=nullptr, uint8_t *picked=nullptr) {
  uint8_t out[15]; bridge.fillStatus(out);
  if (fault) *fault=out[2]; if (picked) *picked=out[1]; return out[0];
}
static void deliver(TableEngine &engine, bool discard=false) {
  auto sent=Radio::outgoing; Radio::outgoing.clear();
  for (const auto &p:sent) {
    Packet reply=engine.handle(p,test_ms);
    if (!discard) Radio::incoming.push_back(reply);
  }
}
static void step(TableBridge &bridge, TableEngine &engine, uint32_t milliseconds=10) {
  test_ms+=milliseconds; engine.tick(test_ms);
  Radio::incoming.push_back(engine.status());
  bridge.tick(true); deliver(engine);
}
static void start(TableBridge &bridge, TableEngine &engine, uint32_t task) {
  assert(bridge.begin()); engine.confirmPosition();
  Radio::incoming.push_back(engine.status()); bridge.tick(true);
  request(bridge,1,task); step(bridge,engine); step(bridge,engine);
  for (int i=0;i<25;++i) step(bridge,engine);
  assert(state(bridge)==READY);
}
static void resetRadio() { Radio::incoming.clear(); Radio::outgoing.clear(); Radio::overflow=false; Radio::receiveAge=0; }

int main() {
  {
    resetRadio(); test_ms=0; TableBridge bridge; TableEngine engine(42); start(bridge,engine,1);
    uint32_t window=engine.window();
    request(bridge,2,1,window); bridge.tick(true);
    uint32_t holdSequence=0;
    for (const auto &p:Radio::outgoing) if(p.type==HOLD) holdSequence=p.sequence;
    assert(holdSequence); deliver(engine,true);
    assert(engine.state()==HELD && state(bridge)==READY); // no ACK -> no grasp
    test_ms+=41; request(bridge,2,1,window); bridge.tick(true);
    bool retried=false;
    for (const auto &p:Radio::outgoing) if(p.type==HOLD) { assert(p.sequence==holdSequence); retried=true; }
    assert(retried); deliver(engine); bridge.tick(true); assert(state(bridge)==HELD);
    for(int i=0;i<20;++i) request(bridge,3,1,window);
    bridge.tick(true); deliver(engine); bridge.tick(true);
    uint8_t picked=0; state(bridge,nullptr,&picked); assert(picked==1);
    for(int i=0;i<20;++i) { request(bridge,3,1,window); step(bridge,engine); }
    assert(engine.picked()==1); // repeated STM request never becomes a new command
  }
  {
    resetRadio(); test_ms=0; TableBridge bridge; TableEngine engine(43); start(bridge,engine,2);
    uint32_t oldWindow=engine.window();
    // Deliver HOLD exactly when the table's 2s window has expired.
    for(int i=0;i<155;++i) step(bridge,engine);
    request(bridge,2,2,oldWindow); bridge.tick(true);
    test_ms+=400; deliver(engine); bridge.tick(true);
    uint8_t fault=0; assert(state(bridge,&fault)==MOVING && fault==0);
    engine.moveComplete(test_ms);
    for(int i=0;i<25;++i) step(bridge,engine);
    assert(state(bridge)==READY && engine.window()!=oldWindow);
    request(bridge,2,2,engine.window()); bridge.tick(true); deliver(engine); bridge.tick(true);
    assert(state(bridge)==HELD);
    TableEngine rebooted(999); Radio::incoming.push_back(rebooted.status()); bridge.tick(true);
    assert(state(bridge,&fault)==FAULT && fault==TableBridge::TABLE_REBOOTED);
  }
  {
    resetRadio(); test_ms=0; TableBridge bridge; TableEngine engine(44); start(bridge,engine,3);
    request(bridge,2,3,engine.window()); bridge.tick(true); deliver(engine); bridge.tick(true);
    assert(state(bridge)==HELD);
    Radio::incoming.clear(); Radio::outgoing.clear(); test_ms+=601; bridge.tick(true);
    uint8_t fault=0; assert(state(bridge,&fault)==FAULT && fault==TableBridge::LINK_LOST);
  }
  {
    resetRadio(); test_ms=0; TableBridge bridge; TableEngine engine(45); start(bridge,engine,4);
    bridge.tick(false); uint8_t fault=0;
    assert(state(bridge,&fault)==FAULT && fault==TableBridge::STM_LINK_LOST);
  }
  {
    resetRadio(); test_ms=0; TableBridge bridge; TableEngine engine(46); start(bridge,engine,5);
    request(bridge,2,5,engine.window()); bridge.tick(true); deliver(engine);
    // The ACK arrived, but the main loop stalled beyond the command deadline.
    test_ms+=601; Radio::receiveAge=601; bridge.tick(true);
    uint8_t fault=0; assert(state(bridge,&fault)==FAULT && fault==TableBridge::COMMAND_TIMEOUT);
  }
  {
    resetRadio(); test_ms=0; TableBridge bridge; TableEngine engine(47); start(bridge,engine,6);
    request(bridge,2,6,engine.window()); bridge.tick(true); deliver(engine);
    // A stale queued ACK cannot grant HELD even before the 600ms command timeout.
    test_ms+=301; Radio::receiveAge=301; bridge.tick(true);
    assert(state(bridge)==READY);
  }
  {
    resetRadio(); test_ms=0; TableBridge bridge; TableEngine engine(48); start(bridge,engine,7);
    Packet unsolicited=engine.status(); unsolicited.state=HELD;
    Radio::incoming.push_back(unsolicited); bridge.tick(true);
    assert(state(bridge)!=HELD); // telemetry alone cannot grant an unrequested hold
    for(int item=0;item<3;++item) {
      request(bridge,2,7,engine.window()); bridge.tick(true); deliver(engine); bridge.tick(true);
      assert(state(bridge)==HELD);
      request(bridge,3,7,engine.window()); bridge.tick(true); deliver(engine); bridge.tick(true);
      assert(engine.picked()==item+1);
      if(item<2) {
        engine.moveComplete(test_ms);
        for(int i=0;i<25;++i) step(bridge,engine);
        assert(state(bridge)==READY);
      }
    }
    assert(state(bridge)==DONE);
    request(bridge,1,8); step(bridge,engine); step(bridge,engine);
    for(int i=0;i<25;++i) step(bridge,engine);
    uint8_t picked=3; assert(state(bridge,nullptr,&picked)==READY && picked==0);
  }
  {
    resetRadio(); test_ms=0; TableBridge bridge; TableEngine engine(49); start(bridge,engine,9);
    uint32_t oldRun=engine.status().run;
    request(bridge,2,9,engine.window()); bridge.tick(true); deliver(engine); bridge.tick(true);
    assert(state(bridge)==HELD);
    bridge.onUartSession();
    uint8_t fault=0; assert(state(bridge,&fault)==FAULT && fault==TableBridge::STM_LINK_LOST);
    bridge.tick(true); deliver(engine); assert(engine.state()==FAULT);
    TableEngine rearmed(149); rearmed.confirmPosition();
    // Manual table re-arm and reconnect alone cannot restart the radio run.
    for(int i=0;i<10;++i) {
      test_ms+=10; Radio::incoming.push_back(rearmed.status()); bridge.tick(true);
      for(const auto &p:Radio::outgoing) assert(p.type!=START);
      deliver(rearmed);
    }
    assert(rearmed.state()==IDLE && state(bridge)==FAULT);
    // The new UART session can explicitly START even if STM reused task ID9.
    request(bridge,1,9); step(bridge,rearmed); step(bridge,rearmed);
    for(int i=0;i<25;++i) step(bridge,rearmed);
    assert(state(bridge)==READY && rearmed.status().run!=oldRun);
    uint32_t newRun=rearmed.status().run;
    for(int i=0;i<10;++i) { request(bridge,1,9); step(bridge,rearmed); }
    assert(rearmed.status().run==newRun); // same-session retransmits stay idempotent
  }
  puts("turntable bridge: ACK gating, retry identity, dedup, late-window recovery, reboot and link loss passed");
}
