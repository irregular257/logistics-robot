#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../ESP32/robot_WebControl/control_state.h"
#include "../../ESP32/robot_WebControl/uart_parser.h"

static unsigned failures;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

static WebFrame source(uint16_t sequence = 65535U) {
  WebFrame f = {};
  f.session = 0x12345678U; f.sequence = sequence;
  f.command = WEB_CMD_STATUS; f.length = 8;
  for (unsigned i = 0; i < f.length; ++i) f.payload[i] = (uint8_t)(i * 17);
  return f;
}
static bool feed(UartParser &p, const uint8_t *wire, unsigned n, uint32_t &now, WebFrame &got) {
  bool found = false;
  for (unsigned i=0; i<n; ++i) {
    if(p.feed(wire[i], now++, got)) found = true;
  }
  return found;
}
static void parserTests() {
  CHECK(Web_Crc16((const uint8_t *)"123456789",9) == 0x29B1U);
  UartParser p; WebFrame f=source(), got={}; uint8_t wire[WEB_MAX_FRAME];
  uint8_t n=Web_Encode(&f,wire); uint32_t now=1;
  CHECK(feed(p,wire,n,now,got));
  CHECK(got.session==f.session && got.sequence==f.sequence && got.length==8 && !memcmp(got.payload,f.payload,8));
  CHECK(p.used==0);
  wire[n-1]^=1;
  CHECK(!feed(p,wire,n,now,got));
  wire[n-1]^=1;
  CHECK(feed(p,wire,n,now,got));
  p.reset(); CHECK(!feed(p,wire,6,now,got)); now+=60;
  CHECK(feed(p,wire,n,now,got));
  p.reset(); const uint8_t illegal[]={0xA5,0x5A,WEB_VERSION,255,0xA5,0x5A,2,0};
  CHECK(!feed(p,illegal,sizeof illegal,now,got));
  CHECK(feed(p,wire,n,now,got));
  p.reset(); f.length=WEB_MAX_PAYLOAD;
  for (unsigned i=0;i<f.length;++i)f.payload[i]=(uint8_t)i;
  n=Web_Encode(&f,wire); CHECK(n==WEB_MAX_FRAME);
  CHECK(feed(p,wire,n,now,got)); CHECK(got.length==WEB_MAX_PAYLOAD);
  p.reset(); now=0xfffffff0U;
  CHECK(feed(p,wire,n,now,got));
  // 50 ms gaps are accepted; >50 ms invalidates a truncated frame.
  p.reset(); now=2; CHECK(!feed(p,wire,5,now,got)); now+=51;
  CHECK(!feed(p,wire+5,n-5,now,got));
  CHECK(feed(p,wire,n,now,got));
  // A corrupt long header may collect two complete short frames before CRC
  // failure. Consuming the first must not discard the following STOP.
  p.reset(); uint8_t joined[WEB_MAX_FRAME]; memset(joined,0x33,sizeof joined);
  joined[0]=0xA5;joined[1]=0x5A;joined[2]=WEB_VERSION;joined[3]=32;
  WebFrame first={};first.session=1;first.sequence=3;first.command=WEB_CMD_JOG;first.length=4;first.payload[1]=20;
  uint8_t first_n=Web_Encode(&first,joined+12);
  WebFrame stop={};stop.session=1;stop.sequence=4;stop.command=WEB_CMD_STOP;
  uint8_t stop_n=Web_Encode(&stop,joined+12+first_n);
  CHECK(first_n==17 && stop_n==13);
  CHECK(feed(p,joined,sizeof joined,now,got));CHECK(got.command==WEB_CMD_JOG);
  CHECK(p.take(got));CHECK(got.command==WEB_CMD_STOP && got.sequence==4);
}
static void stateTests() {
  BrowserControl s; s.claim(99,100);
  CHECK(!s.owns(0) && !s.owns(98) && s.owns(99));
  CHECK(!s.accept(98,1,101) && !s.accept(99,0,101));
  CHECK(s.accept(99,1,101));
  CHECK(!s.accept(99,1,200) && s.lastSeen==101);
  CHECK(s.accept(99,3,201)); CHECK(!s.accept(99,2,202));
  s.moving=true; s.lastJog=200;
  CHECK(s.accept(99,4,400));
  CHECK(!s.jogExpired(499) && s.jogExpired(500)); // Ownership cannot extend motion.
  s.armed=true; s.armPending=true; s.armSequence=65535; s.armSentAt=400;
  s.clear(); CHECK(!s.acceptArmAck(65535,true,401) && !s.armed);
  CHECK(!s.accept(99,5,402)); // STOP revoked token before delayed jog arrived.
  s.claim(100,1000); CHECK(s.accept(100,1,1000));
  s.armPending=true; s.armSequence=3; s.armSentAt=1000;
  CHECK(!s.acceptArmAck(4,true,1100));
  CHECK(s.acceptArmAck(3,true,1100) && s.armed);
  s.armed=false; s.armPending=true; s.armSequence=5; s.armSentAt=1200;
  CHECK(s.accept(100,2,1500));
  CHECK(s.armExpired(1800));
  CHECK(!s.acceptArmAck(5,true,1800)); // A late ACK cannot beat pending-arm timeout.
  CHECK(!s.armed); // Late ACK must fail closed.
  s.claim(101,0xffffff00U);
  CHECK(!s.expired(0x00000100U) && s.expired(0x00000300U));
  s.moving=true; s.lastJog=0xffffff00U;
  CHECK(!s.jogExpired(0x00000000U) && s.jogExpired(0x0000002cU));
  s.claim(102,100); CHECK(s.accept(102,0xffffffffU,101));
  CHECK(!s.accept(102,1,102)); // Browser sequence intentionally does not wrap.
  s.claim(103,103); CHECK(s.accept(103,1,104));
}
int main(){parserTests();stateTests();printf("ESP32 state/parser review: %u failure(s)\n",failures);return failures?1:0;}
