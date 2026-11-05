#ifndef robot_UART_PARSER_H
#define robot_UART_PARSER_H
#include <string.h>
#include "web_protocol.h"

/* Bounded parser; preserve an embedded frame header after corrupt input. */
struct UartParser {
  uint8_t bytes[WEB_MAX_FRAME];
  uint8_t used = 0;
  uint32_t lastByte = 0;
  void reset() { used = 0; }
  void drop(uint8_t count) {
    if (count >= used) used = 0;
    else { used -= count; memmove(bytes, bytes + count, used); }
  }
  bool take(WebFrame &out) {
    while (used >= 2) {
      if (bytes[0] != 0xA5 || bytes[1] != 0x5A) { drop(1); continue; }
      if (used < 4) return false;
      if (bytes[2] != WEB_VERSION || bytes[3] > WEB_MAX_PAYLOAD) {
        drop(1); continue;
      }
      uint8_t size = (uint8_t)(13 + bytes[3]);
      if (used < size) return false;
      if (Web_Crc16(bytes + 2, (uint8_t)(size - 4)) != Web_Get16(bytes + size - 2)) {
        drop(1); continue;
      }
      out.session = Web_Get32(bytes + 4); out.sequence = Web_Get16(bytes + 8);
      out.command = bytes[10]; out.length = bytes[3];
      memcpy(out.payload, bytes + 11, out.length); drop(size); return true;
    }
    return false;
  }
  bool feed(uint8_t value, uint32_t now, WebFrame &out) {
    if (used && (uint32_t)(now - lastByte) > 50U) reset();
    lastByte = now;
    if (used == WEB_MAX_FRAME) drop(1);
    bytes[used++] = value; return take(out);
  }
};
#endif
