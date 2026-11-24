#pragma once
#include <stdint.h>
#include <math.h>

// Modelo sin dependencias de Arduino para probar limites y temporizadores.
namespace bandeja {
struct ModeloServos {
  bool habilitado=false,retornando=false;
  float centro[2]={90,90},actual[2]={90,90},objetivo[2]={90,90},maxDelta=2.0f;
  uint32_t ultimoHb=0,finPrueba=0,finRetorno=0,finConsigna=0;
  int ejePrueba=-1;
  void latido(uint32_t ahora){ultimoHb=ahora;}
  bool configurar(float maxDeg,float c0=90.0f,float c1=90.0f){
    if(!isfinite(maxDeg)||maxDeg<1.0f||maxDeg>18.0f||!isfinite(c0)||!isfinite(c1)||fabsf(c0-90)>12||fabsf(c1-90)>12)return false;
    maxDelta=maxDeg;centro[0]=c0;centro[1]=c1;
    if(!habilitado){actual[0]=objetivo[0]=c0;actual[1]=objetivo[1]=c1;}
    return true;
  }
  void apagar(){habilitado=false;retornando=false;finPrueba=finConsigna=0;ejePrueba=-1;objetivo[0]=centro[0];objetivo[1]=centro[1];}
  bool armar(uint32_t ahora){
    if(!ultimoHb || uint32_t(ahora-ultimoHb)>500)return false;
    habilitado=true;retornando=false;ejePrueba=-1;finPrueba=finConsigna=0;
    actual[0]=objetivo[0]=centro[0];actual[1]=objetivo[1]=centro[1];return true;
  }
  bool probar(int pin,float delta,uint32_t ms,uint32_t ahora){
    if(!habilitado || retornando || ejePrueba!=-1 || finConsigna || uint32_t(ahora-ultimoHb)>500 ||
       (pin!=41 && pin!=42) || !isfinite(delta) || fabsf(delta)>maxDelta || ms<100 || ms>800)return false;
    ejePrueba=pin==41?0:1;objetivo[0]=centro[0];objetivo[1]=centro[1];objetivo[ejePrueba]+=delta;finPrueba=ahora+ms;return true;
  }
  bool consigna(float x,float y,uint32_t ahora){
    if(!habilitado || retornando || ejePrueba!=-1 || uint32_t(ahora-ultimoHb)>500 ||
       !isfinite(x) || !isfinite(y) || fabsf(x-centro[0])>maxDelta+0.01f || fabsf(y-centro[1])>maxDelta+0.01f)return false;
    objetivo[0]=x;objetivo[1]=y;finConsigna=ahora+350;return true;
  }
  void centrar(){objetivo[0]=centro[0];objetivo[1]=centro[1];finPrueba=finConsigna=0;ejePrueba=-1;}
  void tick(uint32_t ahora){
    if(!habilitado)return;
    if(uint32_t(ahora-ultimoHb)>700 && !retornando){centrar();retornando=true;finRetorno=ahora+300;}
    if(finConsigna && int32_t(ahora-finConsigna)>=0){centrar();retornando=true;finRetorno=ahora+300;}
    if(finPrueba && int32_t(ahora-finPrueba)>=0)centrar();
    float paso=maxDelta>2.01f?1.8f:0.4f;
    for(int k=0;k<2;k++){
      float d=objetivo[k]-actual[k];
      actual[k]+=d>paso?paso:(d<-paso?-paso:d);
    }
    if(retornando && int32_t(ahora-finRetorno)>=0)apagar();
  }
};
}
