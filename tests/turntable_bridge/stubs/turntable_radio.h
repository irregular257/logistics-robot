#pragma once
#include <deque>
#include <vector>
#include <turntable_protocol.h>
extern uint32_t test_ms;
namespace table_now {
class Radio {
 public:
  static std::deque<Packet> incoming;
  static std::vector<Packet> outgoing;
  static bool overflow;
  static uint32_t receiveAge;
  bool begin(const uint8_t *, uint8_t) { return true; }
  bool send(const Packet &p) { outgoing.push_back(p); return true; }
  bool receive(Packet &p, uint32_t *receivedAt=nullptr) {
    if (incoming.empty()) return false;
    p = incoming.front(); incoming.pop_front();
    if (receivedAt) *receivedAt=::test_ms-receiveAge;
    return true;
  }
  bool overflowed() { bool value=overflow; overflow=false; return value; }
};
}
