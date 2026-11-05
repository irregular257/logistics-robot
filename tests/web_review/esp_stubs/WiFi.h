#include "Arduino.h"
#define WIFI_AP 1
struct IPAddress{IPAddress(int,int,int,int){}};
struct HostWiFi{void mode(int){} void softAPConfig(IPAddress,IPAddress,IPAddress){} bool softAP(const char*,const char*,int,bool,int){return true;}};
static HostWiFi WiFi;
