#ifndef robot_CONTROL_STATE_H
#define robot_CONTROL_STATE_H
#include <stdint.h>

/* Browser ownership and ordering, independent of Wi-Fi/UART libraries. */
struct BrowserControl {
  uint64_t token = 0;
  uint32_t lastSeen = 0;
  uint32_t lastSequence = 0;
  bool armed = false;
  bool armPending = false;
  uint16_t armSequence = 0;
  uint32_t armSentAt = 0;
  bool moving = false;
  uint32_t lastJog = 0;

  void claim(uint64_t newToken, uint32_t now) {
    clear(); token = newToken; lastSeen = now;
  }
  void clear() {
    token = 0; lastSequence = 0; armed = false; armPending = false;
    moving = false;
  }
  bool owns(uint64_t candidate) const { return token && candidate == token; }
  bool expired(uint32_t now) const {
    return token && (uint32_t)(now - lastSeen) >= 850U;
  }
  bool accept(uint64_t candidate, uint32_t sequence, uint32_t now) {
    if (!owns(candidate) || expired(now) || !sequence || sequence <= lastSequence)
      return false;
    lastSequence = sequence; lastSeen = now; return true;
  }
  bool jogExpired(uint32_t now) const {
    return moving && (uint32_t)(now - lastJog) >= 300U;
  }
  bool armExpired(uint32_t now) const {
    return armPending && (uint32_t)(now - armSentAt) >= 600U;
  }
  bool acceptArmAck(uint16_t sequence, bool ok, uint32_t now) {
    if (!token || expired(now) || !armPending || armExpired(now) || sequence != armSequence)
      return false;
    armPending = false; armed = ok; return true;
  }
};
#endif
