/* Diagnostico VL53L0X independiente del MPU6050. No habilita PWM ni motores.
   NodeMCU-32S: SDA23, SCL22, logica 3V3. Confirmar alimentacion del breakout.
   No alimentar un modulo con dano termico/cortocircuito sin revisar su hardware. */
#include <Arduino.h>
#include <Wire.h>
#include <DSMLaser.h>

constexpr uint8_t ESC[8] = {25,26,27,14,21,19,5,17};
bool emitir = true;

void estado() {
  Serial.printf("{\"version\":\"dsm-laser-0.1\",\"ms\":%lu,\"seq\":%lu,\"modelo\":%u,\"inicializado\":%s,\"mm\":%u,\"range_status\":%u,\"valida\":%s,\"edad_ms\":%ld,\"validas\":%lu,\"invalidas\":%lu,\"errores_i2c\":%lu,\"error_api\":%d,\"senal_mcps\":%.3f,\"ambiente_mcps\":%.3f,\"motores\":\"LOW_sin_PWM\"}\n",
    (unsigned long)millis(),(unsigned long)vl53::secuencia,vl53::modelo,
    vl53::activo?"true":"false",vl53::ultimoMm,vl53::estadoRango,
    vl53::fresca()&&vl53::estadoRango==0?"true":"false",
    vl53::ultimaValidaMs?(long)(millis()-vl53::ultimaValidaMs):-1L,
    (unsigned long)vl53::validas,(unsigned long)vl53::invalidas,
    (unsigned long)vl53::erroresI2c,vl53::errorApi,vl53::senalMcps,vl53::ambienteMcps);
}
void setup() {
  for (uint8_t p:ESC) { pinMode(p,OUTPUT);digitalWrite(p,LOW); }
  Serial.begin(115200);
  Wire.begin(23,22,400000);
  Wire.setTimeOut(2);
  delay(300);
  Serial.println("# dsm-laser-0.1: sin motores; ver | probar | pausa | iniciar_laser");
  // Primero inspeccion pasiva. La inicializacion se solicita explicitamente.
  vl53::modelo=vl53::rReg(0xC0);
  estado();
}
void loop() {
  static char linea[32];static uint8_t n=0;static uint32_t imp=0;
  uint16_t mm;
  vl53::leerMmNoBloqueante(mm);
  while (Serial.available()) {
    char c=Serial.read();
    if(c=='\r')continue;
    if(c=='!'){emitir=false;n=0;continue;}
    if(c=='\n') {
      linea[n]=0;n=0;
      if(!strcmp(linea,"ver"))estado();
      else if(!strcmp(linea,"iniciar_laser")){vl53::iniciar();estado();}
      else if(!strcmp(linea,"probar"))emitir=true;
      else if(!strcmp(linea,"pausa"))emitir=false;
    }else if(static_cast<size_t>(n)+1<sizeof(linea))linea[n++]=c;
  }
  if(emitir&&millis()-imp>=50){imp=millis();estado();}
  delay(1);
}
