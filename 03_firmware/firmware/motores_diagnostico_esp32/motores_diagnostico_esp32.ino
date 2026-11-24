/* Prueba individual o de los ocho ESC juntos. No mide RPM ni sentido de giro.
   Usar SIN helices. Boot: pines LOW. Requiere latido hb para armar/probar.
   No combinar 5V USB y buck de la galleta. */
#include <Arduino.h>
#ifdef DSM_PRUEBA_HOST
#define Out Serial
inline int leerByteEntrada(){return Serial.available()?Serial.read():-1;}
#else
#include <DSM_EnlaceDron.h>
#include "sensores_diagnostico.h"
#endif
constexpr uint8_t PIN[8]={25,26,27,14,21,19,5,17};
enum {DESARMADO,ARMANDO,LISTO,PROBANDO};
uint8_t estado=DESARMADO,motor=0;
uint32_t ultimoHb=0,plazo=0,ultimaActividad=0;
uint16_t pulso=0;
const char* motivo="inicio";
bool asignado[8]={};
bool pruebaTodos=false;

void detener(const char* por){
  for(uint8_t k=0;k<8;k++){if(asignado[k])ledcDetach(PIN[k]);asignado[k]=false;pinMode(PIN[k],OUTPUT);digitalWrite(PIN[k],LOW);}
  estado=DESARMADO;motor=0;pulso=0;pruebaTodos=false;motivo=por;
#ifndef DSM_PRUEBA_HOST
  sensores_diag::calibrando=false;
#endif
}
void pwm(uint8_t k,uint16_t us){ledcWrite(PIN[k],(uint32_t)(us*16383UL/20000UL));}
void informe(){
  uint32_t restante=(estado==PROBANDO || estado==ARMANDO) && (int32_t)(plazo-millis())>0?plazo-millis():0;
  Out.printf("{\"version\":\"dsm-motores-0.3\",\"ms\":%lu,\"estado\":%u,\"motor\":%u,\"todos\":%s,\"pwm_us\":%u,\"restante_ms\":%lu,\"motivo\":\"%s\",\"sentido_medido\":false",(unsigned long)millis(),estado,motor,pruebaTodos?"true":"false",pulso,(unsigned long)restante,motivo);
#ifndef DSM_PRUEBA_HOST
  sensores_diag::imprimir();
#endif
  Out.println("}");
}
void iniciarPrueba(unsigned n,unsigned us,unsigned ms,bool todos){
  if(estado!=LISTO || (!todos && (n<1 || n>8)) || us<1000 || us>1950 || ms<100 || ms>5000 || millis()-ultimoHb>500){
    Out.println("# ERROR: PWM 1000..1950 us, tiempo 100..5000 ms, ESC LISTO y hb reciente");return;
  }
  for(uint8_t k=0;k<8;k++)pwm(k,(todos || static_cast<unsigned>(k)+1==n)?us:1000);
  motor=todos?0:n;pruebaTodos=todos;pulso=us;estado=PROBANDO;
  plazo=millis()+ms;ultimaActividad=millis();motivo=todos?"prueba de los ocho juntos":"prueba individual";informe();
}
void comando(char* l){
  if(!strcmp(l,"hb")){ultimoHb=millis();return;}
  if(!strcmp(l,"!")||!strcmp(l,"stop")||!strcmp(l,"desarmar")){detener("orden stop");informe();return;}
  if(!strcmp(l,"ver")){informe();return;}
#ifndef DSM_PRUEBA_HOST
  if(estado==DESARMADO && sensores_diag::comando(l)){informe();return;}
  if(sensores_diag::calibrando && !strcmp(l,"armar")){Out.println("# ERROR: esperar fin del nivelado IMU");return;}
#endif
  if(!strcmp(l,"armar")){
    if(estado!=DESARMADO || millis()-ultimoHb>500 || !ultimoHb){Out.println("# ERROR: desarmar y mantener hb antes de armar");return;}
    for(uint8_t k=0;k<8;k++){
      if(!ledcAttach(PIN[k],50,14)){detener("fallo LEDC");informe();return;}
      asignado[k]=true;
      pwm(k,1000);
    }
    estado=ARMANDO;plazo=millis()+3000;ultimaActividad=millis();pulso=1000;motivo="armando a 1000 us";informe();return;
  }
  unsigned n=0,us=0,ms=0;char extra;
  if(sscanf(l,"motor %u %u %u %c",&n,&us,&ms,&extra)==3){
    iniciarPrueba(n,us,ms,false);return;
  }
  if(sscanf(l,"todos %u %u %c",&us,&ms,&extra)==2){iniciarPrueba(0,us,ms,true);return;}
  Out.println("# Comandos: ver | hb | armar | motor N us ms | todos us ms | ! | desarmar");
}
void setup(){
  // ledcDetach de un pin aun no asignado solo produce diagnostico; forzar LOW.
  for(uint8_t p:PIN){pinMode(p,OUTPUT);digitalWrite(p,LOW);}
  Serial.begin(115200);delay(300);
#ifndef DSM_PRUEBA_HOST
  if(enlace::iniciar())Out.println("# WiFi DSM-Octo / UDP 4210 activo; USB solo con LiPo desconectadas o VBUS aislado");
  sensores_diag::iniciar();
#endif
  informe();
}
void loop(){
  static char l[80];static uint8_t n=0;static bool larga=false;static uint32_t imp=0;
  int entrada;
  while((entrada=leerByteEntrada())>=0){
    char c=(char)entrada;
    if(c=='!'){n=0;larga=false;detener("emergencia");informe();continue;}
    if(c=='\r')continue;
    if(c=='\n'){if(!larga){l[n]=0;comando(l);}n=0;larga=false;}
    else if(static_cast<size_t>(n)+1<sizeof(l))l[n++]=c;else larga=true;
  }
  if(estado!=DESARMADO && millis()-ultimoHb>1000){detener("perdida de latido");informe();}
  if(estado==ARMANDO && (int32_t)(millis()-plazo)>=0){estado=LISTO;motivo="listo 1000 us";informe();}
  if(estado==PROBANDO && (int32_t)(millis()-plazo)>=0){for(uint8_t k=0;k<8;k++)pwm(k,1000);estado=LISTO;pulso=1000;pruebaTodos=false;motor=0;motivo="fin tiempo";informe();}
  if(estado==LISTO && millis()-ultimaActividad>15000){detener("inactividad");informe();}
#ifndef DSM_PRUEBA_HOST
  sensores_diag::paso();
#endif
  if(millis()-imp>=50){imp=millis();informe();}
  delay(1);
}
