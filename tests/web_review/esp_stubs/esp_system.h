#include <stdint.h>
inline uint32_t esp_random(){static uint32_t counter=0x13572468;return ++counter;}
