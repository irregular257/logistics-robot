#ifndef ESP_REVIEW_ARDUINO_H
#define ESP_REVIEW_ARDUINO_H
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <type_traits>
#include <map>
#include <vector>
#include <functional>
#define PROGMEM
#define SERIAL_8N1 0
extern uint32_t test_ms;
inline uint32_t millis(){return test_ms;}
inline void delay(uint32_t n){test_ms+=n;}
class String {
  std::string v;
 public:
  String(){}
  String(const char *p):v(p?p:""){}
  String(const std::string &p):v(p){}
  String(char ch):v(1,ch){}
  template<class T,typename std::enable_if<std::is_integral<T>::value,int>::type=0> String(T n):v(std::to_string(n)){}
  size_t length()const{return v.length();}
  char operator[](size_t n)const{return v[n];}
  void reserve(size_t n){v.reserve(n);}
  String &operator+=(const String &other){v+=other.v;return *this;}
  String &operator+=(char c){v+=c;return *this;}
  const char *c_str()const{return v.c_str();}
  std::string str()const{return v;}
  friend String operator+(const String &a,const String &b){return String(a.v+b.v);}
};
struct HostSerial{void begin(int){} void println(const char*){}};
static HostSerial Serial;
#endif
