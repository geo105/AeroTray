#pragma once
#include <Wire.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <DSMLaser.h>
#include <DSMMontajeSensores.h>
#include <math.h>
#include "calibracion_imu.h"

// Conserva la disposición de EEPROM y la transformación del controlador
// funcional anterior. No contiene control de vuelo ni genera PWM.
namespace sensores_diag {
struct ConfigOcto {
  uint16_t magico;
  float kpPR,kiPR,kdPR,kpYaw,kdYaw,kpZ,kiZ,kdZ,zRefCm;
  float baseObjetivoUs,usMinActivo,usMax,angMax,trim[8];
  float gyroBias[3],angOffset[2],rotImuDeg;
  float escalaVbatA,escalaVbatB,vbatMin;
  uint8_t flags;
  int frecEscHz;
};
static_assert(sizeof(ConfigOcto)==132,"Conservar la configuracion EEPROM del controlador anterior");
static uint8_t addr=0x68;
static bool presente=false,filtro=false,calibrando=false;
static float bias[3]={},offset[2]={},rot=-135;
static bool swapXY=false;
static int signoP=1,signoR=1,signoY=1;
static float pitch=0,roll=0,yaw=0,gx=0,gy=0,gz=0;
static uint32_t ultimaImu=0,ultimoPaso=0,ultimaBat=0;
static float escalaA=13.3f/3.3f,escalaB=13.3f/3.3f,batA=0,batB=0;
static bool laser=false,baseValida=false;
static float baseMm=0,zCm=0;
static uint16_t ultimoMm=0;
static uint16_t nCal=0;
static double sumas[5]={};
static bool legadoValido=false,calGuardada=false;
static const char* fuenteCal="predeterminada";

inline bool imuValida(){return presente && ultimaImu && millis()-ultimaImu<=100;}
inline float vertical(){return cosf(pitch*PI/180)*cosf(roll*PI/180);}
inline bool zValida(){return laser && baseValida && vl53::fresca() && imuValida();}
inline bool reg(uint8_t r,uint8_t v){Wire.beginTransmission(addr);Wire.write(r);Wire.write(v);return Wire.endTransmission()==0;}
inline void cargarLegado(){
  EEPROM.begin(512);ConfigOcto c;EEPROM.get(0,c);
  if(c.magico!=0x0C79)return;
  for(int i=0;i<3;i++)if(!isfinite(c.gyroBias[i]) || fabsf(c.gyroBias[i])>30)return;
  for(int i=0;i<2;i++)if(!isfinite(c.angOffset[i]) || fabsf(c.angOffset[i])>25)return;
  if(!isfinite(c.rotImuDeg) || fabsf(c.rotImuDeg)>180)return;
  for(int i=0;i<3;i++)bias[i]=c.gyroBias[i];
  for(int i=0;i<2;i++)offset[i]=c.angOffset[i];
  rot=c.rotImuDeg;swapXY=c.flags&1;signoP=c.flags&2?-1:1;signoR=c.flags&4?-1:1;signoY=c.flags&8?-1:1;
  if(isfinite(c.escalaVbatA) && c.escalaVbatA>1 && c.escalaVbatA<10)escalaA=c.escalaVbatA;
  if(isfinite(c.escalaVbatB) && c.escalaVbatB>1 && c.escalaVbatB<10)escalaB=c.escalaVbatB;
  legadoValido=true;fuenteCal="legado";
  Out.printf("# Calibracion IMU anterior recuperada: rot=%.1f grados\n",rot);
}
inline void cargar(){
  cargarLegado();Preferences p;
  if(!p.begin("dsm_diag",true))return;
  calimu::Registro r;
  bool ok=p.getBytesLength("imu")==sizeof(r) && p.getBytes("imu",&r,sizeof(r))==sizeof(r);
  p.end();
  if(!ok || !calimu::valida(r))return;
  for(int i=0;i<3;i++)bias[i]=r.bias[i];
  for(int i=0;i<2;i++)offset[i]=r.offset[i];
  rot=r.rot;swapXY=r.flags&1;signoP=r.flags&2?-1:1;signoR=r.flags&4?-1:1;signoY=r.flags&8?-1:1;
  fuenteCal="dsm_diag";calGuardada=true;
  Out.println("# Calibracion IMU verificada recuperada de dsm_diag");
}
inline void iniciar(){
  cargar();Wire.begin(23,22,400000);Wire.setTimeOut(3);
  for(uint8_t a:{uint8_t(0x68),uint8_t(0x69)}){
    Wire.beginTransmission(a);Wire.write(0x75);
    if(Wire.endTransmission(false)==0 && Wire.requestFrom((int)a,1)==1 && Wire.read()==0x68){addr=a;presente=true;break;}
  }
  if(presente){
    presente=reg(0x6B,1);delay(10);
    presente=presente && reg(0x1A,3) && reg(0x1B,8) && reg(0x1C,8);
  }
  Out.printf("# MPU6050 %s en 0x%02X; SDA23/SCL22\n",presente?"detectado":"NO detectado",addr);
  laser=vl53::iniciar();
  Out.printf("# Laser VL53L0X: %s\n",laser?"iniciado":"sin lectura disponible");
  analogReadResolution(12);ultimoPaso=millis();
}
inline void paso(){
  uint32_t ahora=millis();
  if(ahora-ultimoPaso>=4){
    float dt=(ahora-ultimoPaso)*.001f;ultimoPaso=ahora;
    if(presente){
      Wire.beginTransmission(addr);Wire.write(0x3B);
      if(Wire.endTransmission(false)==0 && Wire.requestFrom((int)addr,14)==14){
        int16_t raw[7];for(int i=0;i<7;i++){uint16_t alto=Wire.read();raw[i]=(int16_t)((alto<<8)|Wire.read());}
        float ax=raw[0]/8192.f,ay=raw[1]/8192.f,az=raw[2]/8192.f;
        float tasas[3]={raw[4]/65.5f,raw[5]/65.5f,raw[6]/65.5f};
        float ap=atan2f(-ax,sqrtf(ay*ay+az*az))*180/PI,ar=atan2f(ay,az)*180/PI;
        float rp=tasas[1]-bias[1],rr=tasas[0]-bias[0];
        if(swapXY){float t=ap;ap=ar;ar=t;t=rp;rp=rr;rr=t;}
        ap*=signoP;ar*=signoR;rp*=signoP;rr*=signoR;
        if(calibrando){
          for(int i=0;i<3;i++)sumas[i]+=tasas[i];sumas[3]+=ap;sumas[4]+=ar;nCal++;
          if(nCal>=300){
            for(int i=0;i<3;i++)bias[i]=sumas[i]/nCal;
            offset[0]=sumas[3]/nCal;offset[1]=sumas[4]/nCal;
            calibrando=false;filtro=false;calGuardada=false;fuenteCal="nivelada_sin_guardar";
            Out.println("# IMU nivelada; guardar para conservar el ajuste");
          }
        }
        ap-=offset[0];ar-=offset[1];
        float c=cosf(rot*PI/180),s=sinf(rot*PI/180);
        float p=ap*c-ar*s,r=ap*s+ar*c;
        gy=rp*c-rr*s;gx=rp*s+rr*c;gz=signoY*(tasas[2]-bias[2]);
        if(fabsf(gz)<.8f && fabsf(gy)<1.5f && fabsf(gx)<1.5f)gz=0;
        if(!filtro || dt>.1f){pitch=p;roll=r;yaw=0;filtro=true;}
        else{float alpha=powf(.994f,dt/.004f);pitch=alpha*(pitch+gy*dt)+(1-alpha)*p;roll=alpha*(roll+gx*dt)+(1-alpha)*r;yaw+=gz*dt;if(yaw>180)yaw-=360;if(yaw< -180)yaw+=360;}
        ultimaImu=ahora;
      }
    }
  }
  if(ahora-ultimaBat>=100){
    ultimaBat=ahora;float a=analogReadMilliVolts(34)*.001f*escalaA,b=analogReadMilliVolts(35)*.001f*escalaB;
    batA=batA==0?a:.85f*batA+.15f*a;batB=batB==0?b:.85f*batB+.15f*b;
  }
  uint16_t mm;
  if(laser && vl53::leerMmNoBloqueante(mm) && imuValida()){
    ultimoMm=mm;float v=mm*vertical();
    if(!baseValida){baseMm=v;baseValida=true;}
    zCm=(v-baseMm)*.1f;
  }
}
inline bool comando(const char* texto){
  if(!strcmp(texto,"estado_montaje")){
    Out.printf("{\"version\":\"dsm-montaje-0.1\",\"revision\":\"%s\",\"imu_brazo\":%u,\"laser_brazo\":%u,\"rot_imu_deg\":%.3f,\"gyro_bias\":[%.5f,%.5f,%.5f],\"ang_offset\":[%.5f,%.5f],\"altura_en_sensor\":true}\n",
      montaje_dsm::REVISION,montaje_dsm::IMU_BRAZO,montaje_dsm::LASER_BRAZO,rot,bias[0],bias[1],bias[2],offset[0],offset[1]);
    return true;
  }
  if(!strcmp(texto,"ver_sensores")){
    ConfigOcto c;EEPROM.get(0,c);
    Out.printf("{\"version\":\"dsm-sensores-0.1\",\"imu_ok\":%s,\"imu_dir\":%u,\"cal_fuente\":\"%s\",\"cal_guardada\":%s,\"legado_valido\":%s,\"legado_magic\":%u,\"rot_imu_deg\":%.2f,\"bias\":[%.4f,%.4f,%.4f],\"offset\":[%.4f,%.4f],\"laser_iniciado\":%s,\"laser_modelo\":%u,\"laser_estado\":%u,\"laser_error_api\":%d,\"laser_mm\":%u,\"laser_secuencia\":%lu,\"laser_validas\":%lu,\"laser_invalidas\":%lu,\"laser_errores_i2c\":%lu,\"laser_edad_ms\":%ld}\n",
      imuValida()?"true":"false",addr,fuenteCal,calGuardada?"true":"false",legadoValido?"true":"false",c.magico,rot,
      bias[0],bias[1],bias[2],offset[0],offset[1],laser?"true":"false",vl53::modelo,vl53::estadoRango,vl53::errorApi,vl53::ultimoMm,
      (unsigned long)vl53::secuencia,(unsigned long)vl53::validas,(unsigned long)vl53::invalidas,(unsigned long)vl53::erroresI2c,
      vl53::ultimaValidaMs?(long)(millis()-vl53::ultimaValidaMs):-1L);
    return true;
  }
  if(!strcmp(texto,"test_laser")){
    laser=vl53::iniciar();
    Out.printf("# Prueba laser: iniciado=%s modelo=%u error_api=%d; esperar medidas nuevas\n",laser?"si":"no",vl53::modelo,vl53::errorApi);
    return true;
  }
  if(!strcmp(texto,"ver_adc")){
    uint32_t rawA=0,rawB=0,mvA=0,mvB=0;
    for(int i=0;i<32;i++){
      rawA+=analogRead(34);rawB+=analogRead(35);
      mvA+=analogReadMilliVolts(34);mvB+=analogReadMilliVolts(35);
    }
    Out.printf("{\"version\":\"dsm-adc-0.1\",\"raw_a\":%.2f,\"raw_b\":%.2f,\"mv_a\":%.2f,\"mv_b\":%.2f,\"escala_a\":%.6f,\"escala_b\":%.6f}\n",rawA/32.f,rawB/32.f,mvA/32.f,mvB/32.f,escalaA,escalaB);
    return true;
  }
  if(!strcmp(texto,"nivel_imu")){
    if(!presente){Out.println("# ERROR: MPU6050 no detectado");return true;}
    for(double &v:sumas)v=0;nCal=0;calibrando=true;
    Out.println("# Mantener dron quieto y nivelado durante 300 muestras IMU");return true;
  }
  if(!strcmp(texto,"cero_z")){
    if(zValida()){baseMm=ultimoMm*vertical();zCm=0;Out.println("# Altura puesta a cero");}
    else Out.println("# ERROR: laser o IMU sin medida valida");return true;
  }
  if(!strcmp(texto,"guardar")){
    if(calibrando || !imuValida()){Out.println("# ERROR: esperar fin de calibracion y medida IMU valida");return true;}
    calimu::Registro r;
    for(int i=0;i<3;i++)r.bias[i]=bias[i];
    for(int i=0;i<2;i++)r.offset[i]=offset[i];
    r.rot=rot;r.flags=(swapXY?1:0)|(signoP<0?2:0)|(signoR<0?4:0)|(signoY<0?8:0);
    calimu::sellar(r);
    if(!calimu::valida(r)){Out.println("# ERROR: parametros IMU fuera de rango; no guardados");return true;}
    Preferences p;
    if(!p.begin("dsm_diag",false)){Out.println("# ERROR: no se pudo abrir memoria de calibracion");return true;}
    bool ok=p.putBytes("imu",&r,sizeof(r))==sizeof(r);calimu::Registro ver;
    ok=ok && p.getBytes("imu",&ver,sizeof(ver))==sizeof(ver) && calimu::valida(ver) && ver.checksum==r.checksum;
    p.end();
    if(!ok){Out.println("# ERROR: calibracion no supero verificacion de escritura");return true;}
    calGuardada=true;fuenteCal="dsm_diag";
    Out.println("# Calibracion IMU guardada y leida de nuevo correctamente (dsm_diag)");return true;
  }
  return false;
}
inline void imprimir(){
  Out.printf(",\"imu_ok\":%s,\"imu_edad_ms\":%ld",imuValida()?"true":"false",ultimaImu?(long)(millis()-ultimaImu):-1L);
  if(imuValida())Out.printf(",\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f",pitch,roll,yaw,gx,gy,gz);
  Out.printf(",\"vbat_a\":%.2f,\"vbat_b\":%.2f,\"z_valida\":%s",batA,batB,zValida()?"true":"false");
  Out.printf(",\"laser_mm\":%u,\"laser_estado\":%u,\"laser_seq\":%lu,\"laser_edad_ms\":%ld",vl53::ultimoMm,vl53::estadoRango,(unsigned long)vl53::secuencia,vl53::ultimaValidaMs?(long)(millis()-vl53::ultimaValidaMs):-1L);
  if(zValida())Out.printf(",\"z_cm\":%.2f",zCm);
}
}
