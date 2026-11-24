#pragma once
#include "Arduino.h"
struct FakeWire {
 uint8_t dev=0,reg=0;
 std::vector<uint8_t> outgoing;
 uint8_t regs[256]={};
 int remaining=0;
 void begin(int,int,int){regs[0x75]=0x68;regs[0x3f]=0x40;}
 void end(){}
 void setTimeOut(int){}
 void beginTransmission(uint8_t d){dev=d;outgoing.clear();}
 void write(uint8_t v){outgoing.push_back(v);}
 uint8_t endTransmission(bool=true){
   if(dev!=0x68)return 2;
   if(!outgoing.empty())reg=outgoing[0];
   if(outgoing.size()>1)regs[reg]=outgoing[1];
   return 0;
 }
 size_t requestFrom(uint8_t,size_t n,bool=true){remaining=n;fakeUs+=1500;return n;}
 int read(){--remaining;return regs[reg++];}
};
inline FakeWire Wire;
