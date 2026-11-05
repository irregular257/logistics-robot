#ifndef ESP_REVIEW_PREFERENCES_H
#define ESP_REVIEW_PREFERENCES_H
#include "Arduino.h"
#include <cstring>
class Preferences {
 public:
  bool available = true, failWrite = false;
  unsigned writes = 0;
  std::map<std::string, std::vector<uint8_t>> values;
  bool begin(const char *, bool = false) { return available; }
  size_t getBytesLength(const char *key) { return values[key].size(); }
  size_t getBytes(const char *key, void *out, size_t size) {
    auto &bytes = values[key];
    size_t count = size < bytes.size() ? size : bytes.size();
    if (count) std::memcpy(out, bytes.data(), count);
    return count;
  }
  size_t putBytes(const char *key, const void *data, size_t size) {
    ++writes;
    if (failWrite) return 0;
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    values[key] = std::vector<uint8_t>(bytes, bytes + size);
    return size;
  }
};
#endif
