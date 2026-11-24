#pragma once
#include <Arduino.h>
#include "servo_limites.h"

namespace bandeja {
static ModeloServos modelo;
static portMUX_TYPE candado=portMUX_INITIALIZER_UNLOCKED;
static bool pwm[2]={false,false};
static int errorHardware=0;
constexpr int PIN[2]={41,42},CANAL[2]={2,3};

inline void tarea(void*){
  TickType_t ultima=xTaskGetTickCount();
  for(;;){
    portENTER_CRITICAL(&candado);modelo.tick(millis());ModeloServos copia=modelo;portEXIT_CRITICAL(&candado);
    for(int k=0;k<2;k++){
      if(copia.habilitado && !pwm[k]){
        bool ok=ledcAttachChannel(PIN[k],50,14,CANAL[k]);
        portENTER_CRITICAL(&candado);pwm[k]=ok;if(!ok){errorHardware=1;modelo.apagar();}portEXIT_CRITICAL(&candado);
      }
      if(copia.habilitado && pwm[k]){
        float us=500.0f+copia.actual[k]*(2000.0f/180.0f);
        ledcWrite(PIN[k],(uint32_t)(us*16383.0f/20000.0f+.5f));
      }else if(!copia.habilitado && pwm[k]){
        ledcDetach(PIN[k]);pinMode(PIN[k],INPUT);
        portENTER_CRITICAL(&candado);pwm[k]=false;portEXIT_CRITICAL(&candado);
      }
    }
    vTaskDelayUntil(&ultima,pdMS_TO_TICKS(20));
  }
}
inline bool iniciar(){
  for(int p:PIN)pinMode(p,INPUT);
  return xTaskCreatePinnedToCore(tarea,"servos_bandeja",3072,nullptr,3,nullptr,1)==pdPASS;
}
inline String estadoJson(){
  portENTER_CRITICAL(&candado);ModeloServos s=modelo;bool a=pwm[0],b=pwm[1];int error=errorHardware;portEXIT_CRITICAL(&candado);
  String r="{\"pines\":[41,42],\"centros_deg\":[";
  r+=String(s.centro[0],1);r+=",";r+=String(s.centro[1],1);
  r+="],\"max_delta_deg\":";r+=String(s.maxDelta,1);
  r+=",\"pwm_activo\":";
  r+=(a||b)?"true":"false";
  r+=",\"habilitado\":";r+=s.habilitado?"true":"false";
  r+=",\"mando_deg\":[";r+=String(s.actual[0],2);r+=",";r+=String(s.actual[1],2);r+="]";
  r+=",\"objetivo_deg\":[";r+=String(s.objetivo[0],2);r+=",";r+=String(s.objetivo[1],2);r+="]";
  r+=",\"eje_prueba\":";r+=s.ejePrueba;
  r+=",\"error_hardware\":";r+=error;r+="}";return r;
}
inline bool comando(const char* l){
  if(!strcmp(l,"hb")){portENTER_CRITICAL(&candado);modelo.latido(millis());portEXIT_CRITICAL(&candado);return true;}
  if(!strcmp(l,"servo_off") || !strcmp(l,"!") || !strcmp(l,"stop")){
    portENTER_CRITICAL(&candado);modelo.apagar();portEXIT_CRITICAL(&candado);Serial.println("# SERVO_OFF");return true;
  }
  if(!strcmp(l,"servo_centro")){
    portENTER_CRITICAL(&candado);modelo.centrar();portEXIT_CRITICAL(&candado);Serial.println("# SERVO_CENTRO 90 90");return true;
  }
  if(!strncmp(l,"servo_cfg ",10)){
    float m,c0=90,c1=90;char extra;
    int n=sscanf(l,"servo_cfg %f %f %f %c",&m,&c0,&c1,&extra);
    portENTER_CRITICAL(&candado);bool ok=(n==1||n==3)&&modelo.configurar(m,c0,c1);portEXIT_CRITICAL(&candado);
    if(ok)Serial.printf("# SERVO_CFG %.1f %.1f %.1f\n",m,c0,c1);
    else Serial.println("# ERROR_SERVO cfg requiere max 1..18 y centros 78..102");
    return true;
  }
  if(!strcmp(l,"servo_arm 90 90") || !strncmp(l,"servo_arm ",10)){
    float c0=90,c1=90;char extra;
    bool cfg=sscanf(l,"servo_arm %f %f %c",&c0,&c1,&extra)==2;
    portENTER_CRITICAL(&candado);
    if(cfg && (fabsf(c0-90)>0.01f || fabsf(c1-90)>0.01f))modelo.configurar(modelo.maxDelta,c0,c1);
    bool ok=modelo.armar(millis());
    portEXIT_CRITICAL(&candado);
    Serial.println(ok?"# SERVO_ARM 90 90":"# ERROR_SERVO falta hb reciente");return true;
  }
  int pin;float delta;unsigned ms;char extra;
  if(!strncmp(l,"servo_set ",10)){
    float x,y;bool formato=sscanf(l,"servo_set %f %f %c",&x,&y,&extra)==2;
    portENTER_CRITICAL(&candado);bool ok=formato && modelo.consigna(x,y,millis());portEXIT_CRITICAL(&candado);
    Serial.println(ok?"# SERVO_SET":"# ERROR_SERVO consigna requiere armado, hb y angulos 88..92");return true;
  }
  if(!strncmp(l,"servo_test ",11)){
    bool formato=sscanf(l,"servo_test %d %f %u %c",&pin,&delta,&ms,&extra)==3;
    portENTER_CRITICAL(&candado);bool ok=formato && modelo.probar(pin,delta,ms,millis());portEXIT_CRITICAL(&candado);
    if(ok)Serial.printf("# SERVO_TEST %d %+.1f %u\n",pin,delta,ms);
    else Serial.println("# ERROR_SERVO usa GPIO41|42, delta -2..2, 100..800 ms, armado y hb reciente");
    return true;
  }
  return false;
}
}
