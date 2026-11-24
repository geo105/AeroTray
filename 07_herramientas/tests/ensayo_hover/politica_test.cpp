#include "../../firmware/ensayo_hover_1600/PoliticaEnsayo.h"
#include <cassert>
#include <cstdio>
#include <random>
#include <limits>

int main() {
  using namespace ensayo;
  const float rad=3.14159265358979323846f/180;
  assert(tiltDegrees(0,0)==0);
  assert(tiltDegrees(15*rad,0)>=14.999f);
  assert(tiltDegrees(11*rad,11*rad)>15);
  assert(!std::isfinite(tiltDegrees(std::numeric_limits<float>::quiet_NaN(),0)));
  assert(fresh(true,1499,1000)); assert(!fresh(true,1500,1000));
  assert(!fresh(false,10,0));
  assert(fresh(true,20,0xfffffff0u)); // rollover millis
  assert(canArm(true,true,true,false,true,0,0));
  for(int i=0;i<5;++i) {
    bool p=true,m=true,c=true,cal=false,h=true;
    if(i==0)p=false;
    if(i==1)m=false;
    if(i==2)c=false;
    if(i==3)cal=true;
    if(i==4)h=false;
    assert(!canArm(p,m,c,cal,h,0,0));
  }
  assert(!canArm(true,true,true,false,true,4.01f,0));
  assert(!canArm(true,true,true,false,true,0,5.01f));
  float current=1300;
  for(int i=0;i<12000;++i) current=slew(current,1600,.005f);
  assert(current>1599 && current<=1600); // 60 s, nunca salto directo
  assert(slew(1500,1300,.5f)==1490);
  assert(slew(1600,5000,2)==1600);
  float delta[8]={-1,-1,-1,-1,1,1,1,1}, out[8], actual;
  assert(allocate(5,0,5,delta,out,actual)==1);
  assert(actual==4 && out[0]==3 && out[7]==5); // conserva RP al techo
  std::mt19937 rng(34);
  std::uniform_real_distribution<float> d(-8,8), b(0,5);
  for(int n=0;n<100000;++n) {
    for(auto &v:delta)v=d(rng);
    const float scale=allocate(b(rng),0,5,delta,out,actual);
    assert(scale>=0 && scale<=1);
    for(int i=0;i<8;++i) {
      assert(out[i]>=0 && out[i]<=5);
      assert(fabsf((out[i]-out[0])-scale*(delta[i]-delta[0]))<.00001f);
    }
  }
  const char *s="123456789";
  assert(crc16(reinterpret_cast<const uint8_t *>(s),9)==0x29b1);
  puts("politica: limites, arranque, heartbeat, rampa, mezclador y CRC OK");
}
