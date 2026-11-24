#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>
#include <vector>
using std::min;
using std::max;
constexpr double PI=3.14159265358979323846;
constexpr int OUTPUT=1,LOW=0,ADC_11db=0;
constexpr int INPUT_PULLUP=2,OUTPUT_OPEN_DRAIN=3,HIGH=1;
inline uint64_t fakeUs=0;
inline uint32_t micros(){return uint32_t(fakeUs);}
inline uint32_t millis(){return uint32_t(fakeUs/1000);}
inline void delay(unsigned ms){fakeUs+=ms*1000;}
inline void delayMicroseconds(unsigned us){fakeUs+=us;}
inline int digitalRead(int){return HIGH;}
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline void analogReadResolution(int){}
inline void analogSetPinAttenuation(int,int){}
inline int analogRead(int){return 1234;}
inline int constrain(int v,int lo,int hi){return std::clamp(v,lo,hi);}
struct FakeSerial {
 std::deque<char> rx;
 std::vector<uint8_t> tx;
 void setTxBufferSize(int){}
 void begin(int){}
 int available(){return rx.size();}
 int read(){char c=rx.front();rx.pop_front();return c;}
 int availableForWrite(){return 4096;}
 size_t write(const uint8_t *p,size_t n){tx.insert(tx.end(),p,p+n);return n;}
 void println(const char*){}
 template<typename... T> void printf(const char*,T...){}
 void inject(const std::string &s){for(char c:s)rx.push_back(c);}
};
inline FakeSerial Serial;
