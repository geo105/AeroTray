#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>

namespace calimu {
struct Registro {
  uint32_t magic=0,version=0;
  float bias[3]={},offset[2]={},rot=0;
  uint32_t flags=0,checksum=0;
};
static_assert(sizeof(Registro)==40,"Formato persistente IMU");
inline uint32_t hash(const Registro& r){
  auto p=reinterpret_cast<const uint8_t*>(&r);uint32_t h=2166136261u;
  for(size_t i=0;i<offsetof(Registro,checksum);i++){h^=p[i];h*=16777619u;}
  return h;
}
inline void sellar(Registro& r){r.magic=0x43414C49;r.version=1;r.checksum=hash(r);}
inline bool valida(const Registro& r){
  if(r.magic!=0x43414C49 || r.version!=1 || r.checksum!=hash(r) || r.flags>15)return false;
  for(float v:r.bias)if(!isfinite(v) || fabsf(v)>30)return false;
  for(float v:r.offset)if(!isfinite(v) || fabsf(v)>45)return false;
  return isfinite(r.rot) && fabsf(r.rot)<=180;
}
}
