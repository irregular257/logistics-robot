#include "Arduino.h"
struct HardwareSerial{
  std::vector<std::vector<uint8_t>> tx;
  std::vector<uint8_t> rx;
  HardwareSerial(int){}
  void setRxBufferSize(int){} void begin(int,int,int,int){}
  size_t write(const uint8_t *p,size_t n){tx.push_back(std::vector<uint8_t>(p,p+n));return n;}
  int available(){return (int)rx.size();}
  int read(){int c=rx.front();rx.erase(rx.begin());return c;}
};
