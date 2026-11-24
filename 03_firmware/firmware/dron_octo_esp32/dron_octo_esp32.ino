/*
  DRON OCTO EN ROTULA - PRUEBA DE GALLETA Y ESTABILIZACION PI-D (8 MOTORES + MPU6050)
  Placa: ESP32 NodeMCU-32S (38 pines, galleta Rev D).
  Incluye TODOS los comandos de diagnostico de Dron_octo_RevD_grupo.pdf (pagina 8) mas el
  controlador en lazo cerrado PI-D a 250 Hz con asignacion acotada de 8 motores y anti-windup.

  MAPA DE MOTORES Y PINES (Dron_octo_RevD_grupo.pdf paginas 5-7):
    M1 (0 deg frente,     ESC1 -> GPIO25, rama A, giro igual a M1)
    M2 (90 deg derecha,   ESC2 -> GPIO26, rama B, giro igual a M1)
    M3 (180 deg atras,    ESC3 -> GPIO27, rama A, giro igual a M1)
    M4 (270 deg izq.,     ESC4 -> GPIO14, rama B, giro igual a M1)
    M5 (45 deg fr-der,    ESC5 -> GPIO21, rama A, giro inverso)
    M6 (135 deg at-der,   ESC6 -> GPIO19, rama B, giro inverso)
    M7 (225 deg at-izq,   ESC7 -> GPIO5,  rama A, giro inverso)
    M8 (315 deg fr-izq,   ESC8 -> GPIO17, rama B, giro inverso)

  SENSORES EN LA GALLETA:
    IMU MPU-6050 (GY-521): I2C SDA = GPIO23, SCL = GPIO22 (400 kHz)
    Divisor bateria rama A: GPIO34 (10k / 3.3k -> 12.6 V = 3.13 V)
    Divisor bateria rama B: GPIO35 (10k / 3.3k -> 12.6 V = 3.13 V)
    Sensor ToF VL53L0X (opcional en el mismo bus I2C 0x29): lectura de altura z [cm]

  SEGURIDAD:
    - Al arrancar, los 8 pines de ESC quedan en LOW (ningun ESC se arma solo).
    - El caracter '!' o el comando 'stop' corta inmediatamente los 8 motores en cualquier momento.
    - Corte automatico si |pitch| o |roll| supera ang_max (25 deg por defecto) o si VBAT_A / VBAT_B
      cae por debajo de 10.5 V bajo carga.
*/
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <DSMMontajeSensores.h>
#include "control_octo.h"
#include "enlace_wifi.h"
#include "vl53l0x_min.h"
#include "estado_sensores.h"
#include "control_referencia_sim.h"
#include "politica_ppo_sim.h"
#include "actuadores_reales.h"

// Este candidato se entrega para banco. La compilacion normal NO puede
// activar ESC, calibrarlos ni poner sus pines en HIGH mediante comandos.
#ifndef DSM_PERMITIR_MOTORES
#define DSM_PERMITIR_MOTORES 0
#endif

// ===================== PINES HARDWARE (GALLETA REV D) =====================
const uint8_t PIN_ESC[8] = {25, 26, 27, 14, 21, 19, 5, 17};
const char *RAMA_ESC[8] = {"A", "B", "A", "B", "A", "B", "A", "B"};
const int ANG_ESC[8] = {0, 90, 180, 270, 45, 135, 225, 315};
const uint8_t PIN_VBAT_A = 34;
const uint8_t PIN_VBAT_B = 35;
const uint8_t PIN_SERVO_OLD[2] = {18, 16};  // Los servos ahora van en la ESP32-S3-CAM
uint8_t PIN_SDA = 23;
uint8_t PIN_SCL = 22;

// ===================== PWM ESC (LEDC + TAREA NUCLEO 0) =====================
int frecEscHz = 250;        // 250 Hz (periodo 4 ms) o 50 Hz; rango 1000..2000 us
const int RES_ESC = 14;     // 16384 niveles
bool pwmConectado = false;
volatile float pulsoObjUs[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
float pulsoActUs[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
volatile bool salidasActivas = false;

void escribirEscHardware(uint8_t idx, float us) {
  if (!pwmConectado || idx >= 8) return;
  float periodoUs = 1000000.0f / (float)frecEscHz;
  us = constrain(us, 0.0f, periodoUs * 0.95f);
  uint32_t maxDuty = (1UL << RES_ESC) - 1;
  uint32_t duty = (uint32_t)((us / periodoUs) * maxDuty + 0.5f);
  ledcWrite(PIN_ESC[idx], duty);
}

void tareaEsc(void *) {
  float ult[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
  for (;;) {
    if (pwmConectado && salidasActivas) {
      for (uint8_t i = 0; i < 8; i++) {
        float u = pulsoObjUs[i];
        if (fabsf(u - ult[i]) >= 0.25f) {
          escribirEscHardware(i, u);
          ult[i] = u;
        }
      }
    } else {
      for (uint8_t i = 0; i < 8; i++) ult[i] = -1;
    }
    vTaskDelay(1);
  }
}

void conectarEscPwm() {
  salidasActivas = false;
  delay(20);
  if (pwmConectado) {
    for (uint8_t i = 0; i < 8; i++) ledcDetach(PIN_ESC[i]);
  }
  for (uint8_t i = 0; i < 8; i++) {
    ledcAttach(PIN_ESC[i], frecEscHz, RES_ESC);
    escribirEscHardware(i, 1000.0f);
    pulsoObjUs[i] = 1000.0f;
    pulsoActUs[i] = 1000.0f;
  }
  pwmConectado = true;
  salidasActivas = true;
}

void desconectarEscLow() {
  salidasActivas = false;
  delay(15);
  if (pwmConectado) {
    for (uint8_t i = 0; i < 8; i++) ledcDetach(PIN_ESC[i]);
    pwmConectado = false;
  }
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(PIN_ESC[i], OUTPUT);
    digitalWrite(PIN_ESC[i], LOW);
    pulsoObjUs[i] = 0.0f;
    pulsoActUs[i] = 0.0f;
  }
}

void mandarTodosUs(float us) {
  if (!pwmConectado) conectarEscPwm();
  for (uint8_t i = 0; i < 8; i++) {
    pulsoActUs[i] = us;
    pulsoObjUs[i] = us;
  }
}

// ===================== BATERIAS (VBAT_A GPIO34, VBAT_B GPIO35) =====================
float escalaVbatA = 13.3f / 3.3f;
float escalaVbatB = 13.3f / 3.3f;
float vbatMin = 10.5f;
bool corteVbatActivo = true;
float vbatA = 0.0f, vbatB = 0.0f;
int mvRawA = 0, mvRawB = 0;

void leerBaterias() {
  long sa = 0, sb = 0;
  for (int i = 0; i < 8; i++) {
    sa += analogReadMilliVolts(PIN_VBAT_A);
    sb += analogReadMilliVolts(PIN_VBAT_B);
  }
  mvRawA = sa / 8;
  mvRawB = sb / 8;
  float va = (mvRawA * 1e-3f) * escalaVbatA;
  float vb = (mvRawB * 1e-3f) * escalaVbatB;
  if (mvRawA < 80) va = 0.0f;
  if (mvRawB < 80) vb = 0.0f;
  vbatA = (vbatA == 0.0f) ? va : (0.85f * vbatA + 0.15f * va);
  vbatB = (vbatB == 0.0f) ? vb : (0.85f * vbatB + 0.15f * vb);
}

// ===================== IMU MPU-6050 + OPCIONAL VL53L0X =====================
uint8_t dirMpu = 0x68;
bool imuLista = false;
bool ultimaImuOk = false;
uint32_t ultimaImuMs = 0;
float gyroBias[3] = {0, 0, 0};     // deg/s
float angOffset[2] = {0, 0};       // pitch, roll en reposo [deg] (marco sensor antes de rotImuDeg)
float rotImuDeg = -135.0f;         // Transformacion anterior conservada: IMU ahora en M8, misma orientacion fisica.
float rawPitchSinRot = 0.0f, rawRollSinRot = 0.0f;
bool swapXYImu = false;
int8_t signoPitchImu = 1, signoRollImu = 1, signoYawImu = 1;

float pitchDeg = 0.0f, rollDeg = 0.0f, yawDeg = 0.0f;
float gxDegS = 0.0f, gyDegS = 0.0f, gzDegS = 0.0f;
bool filtroImuIniciado = false;

// Lectura de VL53L0X (0x29) + estimador inercial vertical con ZUPT
bool tofPresente = false;
float zBaseMm = 0.0f;
uint16_t ultLaserMm = 0;
float zLaserCm = 0.0f;
float zActualCm = 0.0f;
bool zValida = false;
float velocidadLaserMS=0;
uint32_t laserPrevMs=0;
float laserPrevCm=0;
octo::ReferenciaAltura alturaRef;

float proyeccionVertical() {
  return cosf(pitchDeg*PI/180.0f)*cosf(rollDeg*PI/180.0f);
}

void actualizarLaser() {
  uint16_t mm = 0;
  if (tofPresente && vl53::leerMmNoBloqueante(mm)) {
    ultLaserMm = mm;
    // No corregir altura con una actitud antigua o desconocida.
    if (octo::imuVigente(ultimaImuOk,millis(),ultimaImuMs) && alturaRef.actualizar(mm,proyeccionVertical(),millis())) {
      uint32_t ahora=millis();
      if(laserPrevMs && ahora-laserPrevMs<=200){float v=(alturaRef.zCm-laserPrevCm)*0.01f/((ahora-laserPrevMs)*0.001f);velocidadLaserMS+=0.3f*(v-velocidadLaserMS);}else velocidadLaserMS=0;
      laserPrevMs=ahora;laserPrevCm=alturaRef.zCm;
      zBaseMm = alturaRef.baseVerticalMm;
      zLaserCm = alturaRef.zCm;
      zActualCm = zLaserCm;
    }
  }
  zValida = tofPresente && vl53::fresca() && alturaRef.valida(millis()) && octo::imuVigente(ultimaImuOk,millis(),ultimaImuMs);
}

// Vector unitario vertical en el marco del sensor y estimador Z por IMU (ZUPT)
float uzVec[3] = {0.0f, 0.0f, 1.0f};
float gBaseImu = 1.0f;
float azBiasCmS2 = 0.0f;
float vzImuCmS = 0.0f;
float zImuCm = 0.0f;
uint16_t contQuietoZ = 0;

void reiniciarAlturaZ() {
  velocidadLaserMS=0;laserPrevMs=0;laserPrevCm=0;
  vzImuCmS = 0.0f;
  zImuCm = 0.0f;
  zActualCm = 0.0f;
  if (tofPresente && vl53::fresca() && octo::imuVigente(ultimaImuOk,millis(),ultimaImuMs)) {
    alturaRef.cero(ultLaserMm,proyeccionVertical(),millis());
    zBaseMm = alturaRef.baseVerticalMm;
  } else alturaRef.invalidar();
  zValida = alturaRef.valida(millis());
}

bool escribirRegI2C(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool iniciarMpu6050() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Wire.setTimeOut(2);
  delay(20);
  const uint8_t candidatas[2] = {0x68, 0x69};
  imuLista = false;
  for (uint8_t k = 0; k < 2; k++) {
    uint8_t a = candidatas[k];
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      dirMpu = a;
      imuLista = true;
      break;
    }
  }
  if (!imuLista) {
    Out.printf("# AVISO: no se detecto MPU-6050 en SDA=%u SCL=%u (0x68/0x69)\n", PIN_SDA, PIN_SCL);
    return false;
  }
  escribirRegI2C(dirMpu, 0x6B, 0x01);  // Salir de sleep, reloj PLL X
  delay(10);
  escribirRegI2C(dirMpu, 0x1A, 0x03);  // DLPF ~44 Hz (filtra vibraciones de los 8 rotores)
  escribirRegI2C(dirMpu, 0x1B, 0x08);  // Gyro +-500 deg/s (65.5 LSB/deg/s)
  escribirRegI2C(dirMpu, 0x1C, 0x08);  // Accel +-4g (8192 LSB/g)
  Out.printf("# MPU-6050 detectado en 0x%02X (SDA=GPIO%u, SCL=GPIO%u)\n", dirMpu, PIN_SDA, PIN_SCL);

  return true;
}

bool leerMpuPaso(float dt) {
  ultimaImuOk = false;
  if (!imuLista) return false;
  Wire.beginTransmission(dirMpu);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)dirMpu, 14) != 14) return false;

  int16_t axRaw = (Wire.read() << 8) | Wire.read();
  int16_t ayRaw = (Wire.read() << 8) | Wire.read();
  int16_t azRaw = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // temperatura
  int16_t gxRaw = (Wire.read() << 8) | Wire.read();
  int16_t gyRaw = (Wire.read() << 8) | Wire.read();
  int16_t gzRaw = (Wire.read() << 8) | Wire.read();

  float ax = axRaw / 8192.0f;
  float ay = ayRaw / 8192.0f;
  float az = azRaw / 8192.0f;
  float gx = gxRaw / 65.5f - gyroBias[0];
  float gy = gyRaw / 65.5f - gyroBias[1];
  float gz = gzRaw / 65.5f - gyroBias[2];

  // Angulos de acelerometro (convencion: pitch = inclinacion hacia M1 frente, roll = hacia M2 derecha)
  float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / PI);
  float accRoll  = atan2f(ay, az) * (180.0f / PI);
  float ratePitch = gy;
  float rateRoll  = gx;

  if (swapXYImu) {
    float tmpA = accPitch; accPitch = accRoll; accRoll = tmpA;
    float tmpR = ratePitch; ratePitch = rateRoll; rateRoll = tmpR;
  }
  accPitch  = signoPitchImu * accPitch - angOffset[0];
  accRoll   = signoRollImu * accRoll - angOffset[1];
  ratePitch = signoPitchImu * ratePitch;
  rateRoll  = signoRollImu * rateRoll;
  rawPitchSinRot = accPitch;
  rawRollSinRot  = accRoll;

  if (fabsf(rotImuDeg) > 0.01f) {
    float rad = rotImuDeg * (PI / 180.0f);
    float cR = cosf(rad), sR = sinf(rad);
    float pRot = accPitch * cR - accRoll * sR;
    float rRot = accPitch * sR + accRoll * cR;
    float rpRot = ratePitch * cR - rateRoll * sR;
    float rrRot = ratePitch * sR + rateRoll * cR;
    accPitch = pRot; accRoll = rRot;
    ratePitch = rpRot; rateRoll = rrRot;
  }

  float rateYaw = signoYawImu * gz;
  // Zona muerta en yaw cuando el dron esta en reposo para evitar deriva lenta en RViz
  if (fabsf(rateYaw) < 0.8f && fabsf(ratePitch) < 1.5f && fabsf(rateRoll) < 1.5f) {
    rateYaw = 0.0f;
  }

  gxDegS = rateRoll;
  gyDegS = ratePitch;
  gzDegS = rateYaw;

  if (!filtroImuIniciado) {
    pitchDeg = accPitch;
    rollDeg = accRoll;
    yawDeg = 0.0f;
    float normA = sqrtf(ax * ax + ay * ay + az * az);
    if (normA > 0.2f) {
      gBaseImu = normA;
      uzVec[0] = ax / normA; uzVec[1] = ay / normA; uzVec[2] = az / normA;
    }
    vzImuCmS = 0.0f;
    zImuCm = 0.0f;
    filtroImuIniciado = true;
  } else {
    const float alpha = 0.994f;
    pitchDeg = alpha * (pitchDeg + ratePitch * dt) + (1.0f - alpha) * accPitch;
    rollDeg  = alpha * (rollDeg  + rateRoll  * dt) + (1.0f - alpha) * accRoll;
    yawDeg  += rateYaw * dt;
    if (yawDeg > 180.0f) yawDeg -= 360.0f;
    if (yawDeg < -180.0f) yawDeg += 360.0f;

    // --- Estimador de altura Z por IMU (proyeccion vertical + ZUPT) + fusion opcional con VL53L0X ---
    float wx = gx * (PI / 180.0f), wy = gy * (PI / 180.0f), wz = gz * (PI / 180.0f);
    float ux = uzVec[0] - (wy * uzVec[2] - wz * uzVec[1]) * dt;
    float uy = uzVec[1] - (wz * uzVec[0] - wx * uzVec[2]) * dt;
    float uz = uzVec[2] - (wx * uzVec[1] - wy * uzVec[0]) * dt;
    float normA = sqrtf(ax * ax + ay * ay + az * az);
    if (normA > 0.2f) {
      float kG = (fabsf(normA - gBaseImu) < 0.06f) ? 0.015f : 0.002f;
      ux = (1.0f - kG) * ux + kG * (ax / normA);
      uy = (1.0f - kG) * uy + kG * (ay / normA);
      uz = (1.0f - kG) * uz + kG * (az / normA);
    }
    float normU = sqrtf(ux * ux + uy * uy + uz * uz);
    if (normU > 0.1f) { uzVec[0] = ux / normU; uzVec[1] = uy / normU; uzVec[2] = uz / normU; }

    float aVertG = (ax * uzVec[0] + ay * uzVec[1] + az * uzVec[2]) - gBaseImu;
    float azCmS2 = aVertG * 980.665f - azBiasCmS2;
    bool quieto = (fabsf(aVertG) < 0.028f) && (fabsf(gx) + fabsf(gy) < 7.0f);
    if (quieto) {
      if (contQuietoZ < 1000) contQuietoZ++;
      if (contQuietoZ > 25) {
        azBiasCmS2 += 0.02f * azCmS2;
        vzImuCmS *= 0.75f;
        if (fabsf(vzImuCmS) < 0.4f) vzImuCmS = 0.0f;
      }
    } else {
      contQuietoZ = 0;
      if (fabsf(azCmS2) > 18.0f) {
        vzImuCmS = constrain(vzImuCmS * 0.992f + azCmS2 * dt, -80.0f, 80.0f);
      }
    }
    zImuCm = constrain(zImuCm + vzImuCmS * dt, 0.0f, 35.0f);

  }
  ultimaImuOk = isfinite(pitchDeg) && isfinite(rollDeg) && isfinite(yawDeg) && isfinite(gxDegS) && isfinite(gyDegS) && isfinite(gzDegS);
  if (ultimaImuOk) ultimaImuMs = millis();
  return ultimaImuOk;
}

void calibrarImuEnReposo() {
  if (!imuLista && !iniciarMpu6050()) return;
  Out.println(F("# Calibrando IMU (mantener el dron quieto y horizontal ~1.5 s)..."));
  double sgx = 0, sgy = 0, sgz = 0;
  double sp = 0, sr = 0;
  int n = 0;
  for (int i = 0; i < 300; i++) {
    Wire.beginTransmission(dirMpu);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)dirMpu, 14) == 14) {
      int16_t axRaw = (Wire.read() << 8) | Wire.read();
      int16_t ayRaw = (Wire.read() << 8) | Wire.read();
      int16_t azRaw = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read();
      int16_t gxRaw = (Wire.read() << 8) | Wire.read();
      int16_t gyRaw = (Wire.read() << 8) | Wire.read();
      int16_t gzRaw = (Wire.read() << 8) | Wire.read();
      sgx += gxRaw / 65.5;
      sgy += gyRaw / 65.5;
      sgz += gzRaw / 65.5;
      float ax = axRaw / 8192.0f, ay = ayRaw / 8192.0f, az = azRaw / 8192.0f;
      float ap = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / PI);
      float ar = atan2f(ay, az) * (180.0f / PI);
      if (swapXYImu) { float t = ap; ap = ar; ar = t; }
      sp += signoPitchImu * ap;
      sr += signoRollImu * ar;
      n++;
    }
    delay(4);
  }
  if (n < 100) {
    Out.println(F("# ERROR: lecturas insuficientes de la IMU."));
    return;
  }
  gyroBias[0] = sgx / n;
  gyroBias[1] = sgy / n;
  gyroBias[2] = sgz / n;
  angOffset[0] = sp / n;
  angOffset[1] = sr / n;
  filtroImuIniciado = false;
  leerMpuPaso(0.004f);
  Out.printf("# IMU calibrada: bias gyro=(%+.2f, %+.2f, %+.2f) deg/s | offset pitch=%+.2f roll=%+.2f deg | rot=%.1f deg\n",
                gyroBias[0], gyroBias[1], gyroBias[2], angOffset[0], angOffset[1], rotImuDeg);
}

void calibrarFrenteImu() {
  if (!imuLista && !iniciarMpu6050()) return;
  Out.println(F("# Calibrando FRENTE (M1): mantén el frente M1 inclinado hacia ABAJO (~10 deg) durante 1 s..."));
  double sp = 0, sr = 0;
  int n = 0;
  for (int i = 0; i < 150; i++) {
    if (leerMpuPaso(0.004f)) {
      sp += rawPitchSinRot;
      sr += rawRollSinRot;
      n++;
    }
    delay(4);
  }
  if (n < 50) return;
  float p0 = sp / n, r0 = sr / n;
  float mag = hypotf(p0, r0);
  if (mag < 3.0f) {
    Out.printf("# ERROR: inclinacion muy pequeña (%.1f deg). Inclina M1 hacia abajo al menos 8 grados y repite 'frente_imu'.\n", mag);
    return;
  }
  // Queremos que al bajar M1 (frente), pitchDeg sea negativo (-mag) y rollDeg sea 0:
  rotImuDeg = atan2f(r0, -p0) * (180.0f / PI);
  filtroImuIniciado = false;
  leerMpuPaso(0.004f);
  Out.printf("# FRENTE M1 ALINEADO: rot_imu = %+.1f deg -> ahora pitch=%+.1f deg, roll=%+.1f deg (usa 'guardar')\n",
             rotImuDeg, pitchDeg, rollDeg);
}

// ===================== CONTROLADOR Y ESTADOS =====================
enum {
  EST_DESARMADO = 0,
  EST_ARMADO = 1,
  EST_CONTROL = 2,
  EST_PRUEBA_MOTOR = 3,
  EST_CAL_ESC_ESPERA_OK = 4,
  EST_ATERRIZANDO = 5
};
uint8_t estado = EST_DESARMADO;
const char *motivoParada = "inicio";

octo::EjePID pidPitch, pidRoll, pidYaw;
octo::ControlAltura ctrlZ;
octo::AsignadorOcto mezclador;
simref::ControlReferencia controlSim;
bool sombraSim=false,sombraPpo=false;
float fuerzasSim[8]={},accionesPpoPrev[8]={},referenciaActualM=0;
float spPitchDeg = 0.0f, spRollDeg = 0.0f, spYawDeg = 0.0f;

bool calcularReferenciaSim(float dt){
  if(!zValida || !octo::imuVigente(ultimaImuOk,millis(),ultimaImuMs)){
    controlSim.reiniciar();for(int i=0;i<8;i++){fuerzasSim[i]=0;accionesPpoPrev[i]=0;}return false;
  }
  // Transformacion usada por el puente RViz. El registro indica que su
  // correspondencia con el marco CAD y los motores reales sigue pendiente.
  float r=-rollDeg*PI/180,p=-pitchDeg*PI/180,y=(yawDeg-spYawDeg)*PI/180;
  float cr=cosf(r/2),sr=sinf(r/2),cp=cosf(p/2),sp=sinf(p/2),cy=cosf(y/2),sy=sinf(y/2);
  float q[4]={cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
  if(q[0]<0)for(float &v:q)v=-v;
  float norm=sqrtf(q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  float factor=norm>1e-7f?2*atan2f(norm,q[0])/norm:2;
  simref::Estado e{zActualCm*0.01f,velocidadLaserMS,{q[1]*factor,q[2]*factor,q[3]*factor},
    {-gxDegS*PI/180,-gyDegS*PI/180,gzDegS*PI/180},proyeccionVertical()};
  float d=ctrlZ.zRefCm*0.01f-referenciaActualM;
  referenciaActualM+=octo::limitar(d,-0.05f*dt,0.05f*dt);
  if(sombraPpo){
    dsm_ppo::paso_red(e.z,e.v,e.rot,e.omega,referenciaActualM,accionesPpoPrev,fuerzasSim);
    return true;
  }
  return controlSim.paso(e,dt,referenciaActualM,fuerzasSim);
}

float baseObjetivoUs = 1420.0f;     // empuje colectivo inicial para dron solo (2.3 kg) [us]
float baseActualUs = 1120.0f;       // rampa suave al iniciar/aterrizar
float rampaUsPorSeg = 220.0f;       // velocidad de rampa de arranque/aterrizaje [us/s]
float angMaxSeguridad = 25.0f;      // corte por inclinacion excesiva [deg]

// Temporizador de prueba individual/grupal de motores (comandos m, todos, giro)
uint32_t finPruebaMs = 0;
bool enSecuenciaGiro = false;
uint8_t motorGiroActual = 0;
float usGiroPrueba = 1150.0f;
uint32_t tCambioGiroMs = 0;

// Tiempos de lazo (250 Hz = 4000 us)
uint32_t periodoUs = 4000;
uint32_t proximoUs = 0, ultimoUs = 0;
uint32_t usCiclo = 0, usCicloMax = 0, nPaso = 0;
uint8_t nivelTelemetria = 1;  // 0=off, 1=25Hz, 2=50Hz, 3=50Hz + reposo 10Hz (puente ROS 2)

// Watchdog del enlace serie: si el puente ROS 2 deja de mandar 'hb' en vuelo -> aterrizaje suave
bool watchdogSerie = true;
const uint32_t WATCHDOG_MS = 1000;
uint32_t ultimoRxMs = 0;

// ===================== EEPROM =====================
const int TAM_EEPROM = 512;
const int DIR_CFG = 0;
const uint16_t MAGICO_OCTO = 0x0C79;
struct ConfigOcto {
  uint16_t magico;
  float kpPR, kiPR, kdPR;
  float kpYaw, kdYaw;
  float kpZ, kiZ, kdZ, zRefCm;
  float baseObjetivoUs, usMinActivo, usMax, angMax;
  float trim[8];
  float gyroBias[3], angOffset[2], rotImuDeg;
  float escalaVbatA, escalaVbatB, vbatMin;
  uint8_t flags;  // bit0: swapXY, bit1: invPitch, bit2: invRoll, bit3: invYaw, bit4: corteVbat, bit5: ctrlZ
  int frecEscHz;
};

void guardarConfig() {
  ConfigOcto c = {};
  c.magico = MAGICO_OCTO;
  c.kpPR = pidPitch.kp; c.kiPR = pidPitch.ki; c.kdPR = pidPitch.kd;
  c.kpYaw = pidYaw.kp; c.kdYaw = pidYaw.kd;
  c.kpZ = ctrlZ.kp; c.kiZ = ctrlZ.ki; c.kdZ = ctrlZ.kd; c.zRefCm = ctrlZ.zRefCm;
  c.baseObjetivoUs = baseObjetivoUs;
  c.usMinActivo = mezclador.usMinActivo;
  c.usMax = mezclador.usMax;
  c.angMax = angMaxSeguridad;
  for (int i = 0; i < 8; i++) c.trim[i] = mezclador.trim[i];
  for (int i = 0; i < 3; i++) c.gyroBias[i] = gyroBias[i];
  c.angOffset[0] = angOffset[0]; c.angOffset[1] = angOffset[1];
  c.rotImuDeg = rotImuDeg;
  c.escalaVbatA = escalaVbatA; c.escalaVbatB = escalaVbatB; c.vbatMin = vbatMin;
  c.flags = (swapXYImu ? 1 : 0) | (signoPitchImu < 0 ? 2 : 0) | (signoRollImu < 0 ? 4 : 0) |
            (signoYawImu < 0 ? 8 : 0) | (corteVbatActivo ? 16 : 0) | (ctrlZ.activo ? 32 : 0);
  c.frecEscHz = frecEscHz;
  EEPROM.put(DIR_CFG, c);
  EEPROM.commit();
  Out.println(F("# configuracion del octo guardada en EEPROM"));
}

void cargarConfig() {
  ConfigOcto c;
  EEPROM.get(DIR_CFG, c);
  if (c.magico != MAGICO_OCTO) {
    Out.println(F("# config: valores por defecto (usa 'nivel_imu' y 'guardar')"));
    return;
  }
  pidPitch.kp = pidRoll.kp = constrain(c.kpPR, 0.0f, 30.0f);
  pidPitch.ki = pidRoll.ki = constrain(c.kiPR, 0.0f, 20.0f);
  pidPitch.kd = pidRoll.kd = constrain(c.kdPR, 0.0f, 15.0f);
  pidYaw.kp = constrain(c.kpYaw, 0.0f, 20.0f);
  pidYaw.kd = constrain(c.kdYaw, 0.0f, 15.0f);
  ctrlZ.kp = constrain(c.kpZ, 0.0f, 50.0f);
  ctrlZ.ki = constrain(c.kiZ, 0.0f, 30.0f);
  ctrlZ.kd = constrain(c.kdZ, 0.0f, 30.0f);
  ctrlZ.zRefCm = 30.0f; // Objetivo actual solicitado; se modifica con zref.
  baseObjetivoUs = constrain(c.baseObjetivoUs, 1100.0f, 1900.0f);
  mezclador.usMinActivo = constrain(c.usMinActivo, 1050.0f, 1350.0f);
  mezclador.usMax = constrain(c.usMax, 1500.0f, 2000.0f);
  angMaxSeguridad = constrain(c.angMax, 10.0f, 45.0f);
  for (int i = 0; i < 8; i++) mezclador.trim[i] = constrain(c.trim[i], -100.0f, 100.0f);
  for (int i = 0; i < 3; i++) gyroBias[i] = constrain(c.gyroBias[i], -30.0f, 30.0f);
  angOffset[0] = constrain(c.angOffset[0], -25.0f, 25.0f);
  angOffset[1] = constrain(c.angOffset[1], -25.0f, 25.0f);
  rotImuDeg = constrain(c.rotImuDeg, -180.0f, 180.0f);
  if (c.escalaVbatA > 1.0f && c.escalaVbatA < 10.0f) escalaVbatA = c.escalaVbatA;
  if (c.escalaVbatB > 1.0f && c.escalaVbatB < 10.0f) escalaVbatB = c.escalaVbatB;
  vbatMin = constrain(c.vbatMin, 9.0f, 12.0f);
  swapXYImu = (c.flags & 1) != 0;
  signoPitchImu = (c.flags & 2) ? -1 : 1;
  signoRollImu  = (c.flags & 4) ? -1 : 1;
  signoYawImu   = (c.flags & 8) ? -1 : 1;
  corteVbatActivo = (c.flags & 16) != 0;
  ctrlZ.activo = false;  // Nunca restaurar automatico el control de altura al arrancar.
  if (c.frecEscHz >= 50 && c.frecEscHz <= 400) frecEscHz = c.frecEscHz;
  Out.println(F("# configuracion del octo cargada desde EEPROM"));
}

// ===================== PARADA Y ARMADO =====================
void cortarMotores(const char *razon, bool dejarArmadoEn1000 = true) {
  sombraSim=false;
  controlSim.reiniciar();
  enSecuenciaGiro = false;
  finPruebaMs = 0;
  pidPitch.reiniciar();
  pidRoll.reiniciar();
  pidYaw.reiniciar();
  ctrlZ.reiniciar();
  motivoParada = razon;
  if (dejarArmadoEn1000 && pwmConectado) {
    mandarTodosUs(1000.0f);
    estado = EST_ARMADO;
  } else {
    desconectarEscLow();
    estado = EST_DESARMADO;
  }
  Out.printf("# CORTE MOTORES (%s) -> estado %s\n", razon, estado == EST_ARMADO ? "ARMADO (1000 us)" : "DESARMADO (LOW)");
}

void armarEsc() {
  Out.println(F("# Armando los 8 ESC a 1000 us durante 2.5 s..."));
  conectarEscPwm();
  mandarTodosUs(1000.0f);
  delay(2500);
  estado = EST_ARMADO;
  motivoParada = "armado 1000 us";
  Out.println(F("# ESC ARMADOS (1000 us). Listo para 'giro', 'm', 'todos' o 'iniciar'. ('!' corta todo)"));
}

// ===================== PRUEBAS DEL PDF REV D (PAGINA 8) =====================
void pruebaPinAlto(uint8_t numMotor) {
  if (numMotor < 1 || numMotor > 8) return;
  cortarMotores("prueba alto GPIO", false);
  uint8_t idx = numMotor - 1;
  Out.printf("# ALTO M%u (GPIO%u, rama %s) a 3.3 V durante 5 s (demas en 0 V)...\n",
                numMotor, PIN_ESC[idx], RAMA_ESC[idx]);
  digitalWrite(PIN_ESC[idx], HIGH);
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    int c = leerByteEntrada();
    if (c == '!') break;
    delay(20);
  }
  digitalWrite(PIN_ESC[idx], LOW);
  Out.printf("# Fin prueba alto M%u (GPIO%u vuelve a 0 V)\n", numMotor, PIN_ESC[idx]);
}

void mostrarBaterias() {
  leerBaterias();
  Out.printf("# Nodo A (GPIO34): %.2f V (%d mV en pin) | Nodo B (GPIO35): %.2f V (%d mV en pin) | Corte < %.2f V (%s)\n",
                vbatA, mvRawA, vbatB, mvRawB, vbatMin, corteVbatActivo ? "ACTIVO" : "APAGADO");
}

void iniciarCalibracionEsc() {
  Out.println(F("# CALIBRACION ESC: enviando 2000 us a los 8 motores."));
  Out.println(F("#   1) Conecta ahora las baterias (sin helices) y espera los tonos de maximo."));
  Out.println(F("#   2) Escribe 'ok' + Enter para bajar a 1000 us (o '!' para cancelar)."));
  conectarEscPwm();
  mandarTodosUs(2000.0f);
  estado = EST_CAL_ESC_ESPERA_OK;
  motivoParada = "calibrando ESC (2000 us)";
}

void confirmarCalibracionEsc() {
  if (estado != EST_CAL_ESC_ESPERA_OK) {
    Out.println(F("# 'ok' solo se usa durante 'cal si'"));
    return;
  }
  mandarTodosUs(1000.0f);
  delay(3000);
  estado = EST_ARMADO;
  motivoParada = "ESC calibrados (1000 us)";
  Out.println(F("# CALIBRACION ESC COMPLETADA (1000 us enviados).Espera los tonos de confirmacion."));
}

// ===================== ESTADO, TABLA Y TELEMETRIA =====================
void escanearBusI2C() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  Out.printf("# Escaneando bus I2C en SDA=GPIO%u, SCL=GPIO%u:\n", PIN_SDA, PIN_SCL);
  int hallados = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      const char *nombre = "desconocido";
      if (a == 0x29) nombre = "VL53L0X / VL53L1X (laser altura)";
      else if (a == 0x53 || a == 0x1D) nombre = "ADXL345 (acelerometro)";
      else if (a == 0x68) nombre = "MPU-6050 principal (AD0=GND)";
      else if (a == 0x69) nombre = "MPU-6050 secundario (AD0=3V3)";
      Out.printf("#   -> 0x%02X : %s\n", a, nombre);
      hallados++;
    }
  }
  if (!hallados) Out.println(F("#   (Ningun dispositivo respondio en SDA=23, SCL=22)"));
}

void probarLaserVl53() {
  cortarMotores("diagnostico laser",false);
  Out.println(F("# PRUEBA DE SENSOR LASER VL53L0X (0x29 en SDA=GPIO23, SCL=GPIO22, 10 s):"));
  tofPresente = vl53::iniciar();
  if (!tofPresente) {
    uint8_t idC0 = vl53::rReg(0xC0);
    Out.printf("# ERROR: VL53L0X no responde en 0x29 (reg 0xC0=0x%02X, esperado 0xEE).\n", idC0);
    Out.println(F("#   Revisa en el conector morado 'sensor I2C extra' de la galleta:"));
    Out.println(F("#   SCL -> GPIO22 | SDA -> GPIO23 | VCC -> 3V3 | GND -> GND (y XSHUT libre o a 3V3)."));
    escanearBusI2C();
    return;
  }
  Out.println(F("# VL53L0X OK (0xEE). Mueve la mano o levanta el dron durante 10 s ('!' para salir):"));
  alturaRef.invalidar();
  uint32_t t0 = millis(), tImp = 0, tAnt = micros();
  while (millis() - t0 < 10000) {
    int c = leerByteEntrada();
    if (c == '!' || c == '\n') break;
    uint32_t ahora = micros();
    float dt = constrain((ahora - tAnt) * 1e-6f, 0.001f, 0.05f);
    tAnt = ahora;
    leerMpuPaso(dt);
    actualizarLaser();
    if (millis() - tImp >= 150) {
      tImp = millis();
      Out.printf("# LASER seq=%lu mm=%u status=%u valida=%d edad_ms=%ld i2c=%lu | Z=%+.1f cm valida_z=%d\n",
        (unsigned long)vl53::secuencia,vl53::ultimoMm,vl53::estadoRango,
        vl53::fresca()&&vl53::estadoRango==0,vl53::ultimaValidaMs?(long)(millis()-vl53::ultimaValidaMs):-1L,
        (unsigned long)vl53::erroresI2c,zActualCm,zValida);
    }
    delay(4);
  }
  Out.println(F("# Fin de prueba VL53L0X. Usa 'cero_z' con el dron en la mesa para fijar Z = 0.0 cm."));
}

void verificarImuEnVivo() {
  cortarMotores("verificacion IMU sin PWM",false);
  if (!imuLista && !iniciarMpu6050()) {
    Out.println(F("# ERROR: MPU-6050 no detectado en SDA=GPIO23, SCL=GPIO22."));
    return;
  }
  Out.println(F("# VERIFICACION IMU EN VIVO (10 s, motores APAGADOS):"));
  Out.println(F("#   - Inclina el dron y comprueba que los motores del lado que BAJA suben de pulso (>1420 us)."));
  Out.println(F("#   - Si bajan M1(0 deg) -> M1,M5,M8 deben subir | Si baja M2(90 deg) -> M2,M5,M6 deben subir."));
  Out.println(F("#   - Envia '!' para terminar antes."));
  uint32_t t0 = millis(), tImp = 0, tAnt = micros();
  while (millis() - t0 < 10000) {
    int c = leerByteEntrada();
    if (c == '!' || c == '\n') break;
    uint32_t ahora = micros();
    float dt = constrain((ahora - tAnt) * 1e-6f, 0.001f, 0.05f);
    tAnt = ahora;
    leerMpuPaso(dt);
    if (millis() - tImp >= 250) {
      tImp = millis();
      float uPitch = -pidPitch.kp * pitchDeg;
      float uRoll  = -pidRoll.kp * rollDeg;
      float sal[8];
      mezclador.mezclar(1420.0f, uPitch, uRoll, 0.0f, sal);
      Out.printf("# pitch=%+6.2f deg  roll=%+6.2f deg  yaw=%+6.1f deg | M1=%.0f M2=%.0f M3=%.0f M4=%.0f | M5=%.0f M6=%.0f M7=%.0f M8=%.0f\n",
                    pitchDeg, rollDeg, yawDeg,
                    sal[0], sal[1], sal[2], sal[3], sal[4], sal[5], sal[6], sal[7]);
    }
    delay(4);
  }
  Out.println(F("# Fin de verificacion IMU. Si algun eje esta invertido, usa: orientacion_imu <swapXY 0|1> <sPitch 1|-1> <sRoll 1|-1> <sYaw 1|-1>"));
}

void ver() {
  leerBaterias();
  const char *nomEst[6] = {"DESARMADO(LOW)", "ARMADO(1000us)", "CONTROL_ACTIVO", "PRUEBA_MOTOR", "CAL_ESC_2000", "ATERRIZANDO"};
  Out.printf("# Estado: %s (%s) | Frec ESC: %d Hz | Ciclo: %lu us (max %lu us)\n",
                nomEst[estado <= 5 ? estado : 0], motivoParada, frecEscHz, (unsigned long)usCiclo, (unsigned long)usCicloMax);
  Out.printf("# IMU (%s, 0x%02X): pitch=%+.2f deg  roll=%+.2f deg  yaw=%+.1f deg | gyro=(%+.1f, %+.1f, %+.1f) deg/s\n",
                imuLista ? "OK" : "SIN_CONEXION", dirMpu, pitchDeg, rollDeg, yawDeg, gxDegS, gyDegS, gzDegS);
  Out.printf("# Baterias: Nodo A=%.2f V  Nodo B=%.2f V (corte %.2f V %s) | Altura Z=%.1f cm (%s)\n",
                vbatA, vbatB, vbatMin, corteVbatActivo ? "ON" : "OFF",
                zActualCm, ctrlZ.activo ? "control Z ON" : "manual");
  Out.printf("# Base: obj=%.0f us (actual=%.0f us, min=%.0f, max=%.0f) | SP pitch=%+.1f roll=%+.1f deg\n",
                baseObjetivoUs, baseActualUs, mezclador.usMinActivo, mezclador.usMax, spPitchDeg, spRollDeg);
  Out.printf("# PID Pitch/Roll: Kp=%.2f Ki=%.2f Kd=%.2f | Yaw: Kp=%.2f Kd=%.2f | Z: Kp=%.1f Ki=%.1f Kd=%.1f\n",
                pidPitch.kp, pidPitch.ki, pidPitch.kd, pidYaw.kp, pidYaw.kd, ctrlZ.kp, ctrlZ.ki, ctrlZ.kd);
  Out.print(F("# Salidas [us]: "));
  for (int i = 0; i < 8; i++) {
    Out.printf("M%d(G%u)=%.0f ", i + 1, PIN_ESC[i], pulsoActUs[i]);
  }
  Out.println();
}

void mostrarTablaMotores() {
  Out.println(F("# --- MAPA DE 8 MOTORES (Rev D, R = 350 mm) ---"));
  Out.println(F("# Motor | Posicion | GPIO | Rama | Giro vs M1 | Mix Pitch | Mix Roll | Mix Yaw | Trim [us]"));
  for (int i = 0; i < 8; i++) {
    Out.printf("#  M%d   |  %3d deg |  %2u  |  %s   | %s |   %+6.3f  |  %+6.3f  |  %+5.1f  |  %+5.1f\n",
                  i + 1, ANG_ESC[i], PIN_ESC[i], RAMA_ESC[i],
                  (octo::MIX_YAW[i] > 0) ? "igual M1  " : "al reves  ",
                  octo::MIX_PITCH[i], octo::MIX_ROLL[i], octo::MIX_YAW[i], mezclador.trim[i]);
  }
  Out.println(F("# --- COMANDOS ---"));
  Out.println(F("# ! o stop             corta inmediatamente los 8 motores"));
  Out.println(F("# aterrizar            rampa suave de descenso hasta apagar motores manteniendo actitud"));
  Out.println(F("# + | -                sube o baja empuje base en +25 / -25 us en vivo"));
  Out.println(F("# ver | b | ?          estado completo | voltaje baterias A y B | esta ayuda"));
  Out.println(F("# alto 1 .. alto 8     pone el GPIO del motor N en 3.3 V por 5 s (prueba sin bateria)"));
  Out.println(F("# cal si -> ok         calibra rango 2000..1000 us de los 8 ESC"));
  Out.println(F("# armar | desarmar     envia 1000 us para armar ESC | pone pines en LOW"));
  Out.println(F("# giro [1150]          gira M1..M8 uno por uno durante 2 s para verificar sentido"));
  Out.println(F("# m <1..8> <us> [s]    gira solo el motor N a <us> microsegundos durante [s] seg"));
  Out.println(F("# todos <us> [s]       gira los 8 motores juntos a <us> durante [s] seg (ej: todos 1150 3)"));
  Out.println(F("# nivel_imu            calibra bias de giroscopio y cero de pitch/roll/Z en reposo"));
  Out.println(F("# i2c | test_laser     escanea el bus I2C (23/22) | prueba el sensor VL53L0X (0x29) por 10 s"));
  Out.println(F("# cero_z               pone en 0.0 cm la altura Z actual (laser + IMU)"));
  Out.println(F("# frente_imu           inclina M1 (frente) hacia abajo y ejecuta esto para alinear IMU en cualquier brazo"));
  Out.println(F("# rot_imu <grados>     rota manualmente los ejes X/Y de la IMU (ej: rot_imu 45 si va en M5)"));
  Out.println(F("# verificar_imu        prueba de 10 s sin motores para comprobar signos de inclinacion"));
  Out.println(F("# orientacion_imu <swapXY 0|1> <sPitch 1|-1> <sRoll 1|-1> <sYaw 1|-1>"));
  Out.println(F("# iniciar [base_us]    arranca estabilizacion en lazo cerrado con rampa suave"));
  Out.println(F("# base <1120..1900>    ajusta el empuje base en microsegundos"));
  Out.println(F("# sp <pitch> <roll>    consigna de inclinacion en grados (-15..+15)"));
  Out.println(F("# pid <kp> <ki> <kd>   ganancias PI-D de Pitch y Roll | pid_yaw <kp> <kd>"));
  Out.println(F("# trim <1..8> <du>     ajuste fino de cada motor (-100..+100 us)"));
  Out.println(F("# vbat_corte 0|1 | vbat_min 10.5 | frec_esc 50..400 | p 0|1|2|3 | guardar | fabrica"));
  Out.println(F("# watchdog 0|1         aterriza solo si el PC deja de mandar 'hb' > 1 s (lo activa el puente ROS 2)"));
}

void emitirTelemetria() {
  if (nivelTelemetria == 0) return;
  bool motoresActivos = (estado == EST_CONTROL || estado == EST_PRUEBA_MOTOR || estado == EST_ATERRIZANDO);
  // p 3 (lo activa el puente ROS 2): tambien emite en reposo a 10 Hz para ver la IMU en RViz
  if (!motoresActivos && nivelTelemetria < 3) return;
  // Lazo a 250 Hz: cada 5 pasos = 50 Hz (p 2/3), cada 10 pasos = 25 Hz (p 1), reposo 10 Hz
  uint32_t div = !motoresActivos ? 25 : ((nivelTelemetria >= 2) ? 5 : 10);
  if (nPaso % div != 0) return;
  Out.printf("%lu,%u,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n",
                (unsigned long)millis(), estado, pitchDeg, rollDeg, yawDeg,
                gxDegS, gyDegS, gzDegS, spPitchDeg, spRollDeg, zActualCm,
                vbatA, vbatB,
                pulsoActUs[0], pulsoActUs[1], pulsoActUs[2], pulsoActUs[3],
                pulsoActUs[4], pulsoActUs[5], pulsoActUs[6], pulsoActUs[7]);
}

// ===================== INTERPRETE DE COMANDOS SERIE =====================
char linea[96];
uint8_t nL = 0;

void procesarComando(char *s) {
  char *tok[6];
  int n = 0;
  for (char *t = strtok(s, " \t"); t && n < 6; t = strtok(NULL, " \t")) tok[n++] = t;
  if (!n) return;
  const char *c = tok[0];
  float v[5] = {0, 0, 0, 0, 0};
  for (int i = 1; i < n; i++) {
    if (!strcmp(c,"cal") && !strcmp(tok[i],"si")) continue;
    char *fin = nullptr;
    v[i-1] = strtof(tok[i],&fin);
    if (!fin || fin==tok[i] || *fin || !isfinite(v[i-1])) {
      Out.println(F("# ERROR: parametro numerico invalido")); return;
    }
  }

  if (!DSM_PERMITIR_MOTORES && (!strcmp(c,"armar") || !strcmp(c,"cal") || !strcmp(c,"alto") ||
      !strcmp(c,"giro") || !strcmp(c,"m") || !strcmp(c,"todos") || !strcmp(c,"iniciar"))) {
    Out.println(F("# BLOQUEADO: candidato de banco, compilado SIN salidas de motores")); return;
  }

  if (!strcmp(c, "!") || !strcmp(c, "stop")) { cortarMotores("orden stop", true); return; }
  if (!strcmp(c, "desarmar")) { cortarMotores("desarmado manual", false); return; }
  if (!strcmp(c, "ver")) { ver(); return; }
  if (!strcmp(c, "b")) { mostrarBaterias(); return; }
  if (!strcmp(c, "estado_montaje")) {
    Out.printf("{\"version\":\"dsm-montaje-0.1\",\"revision\":\"%s\",\"imu_brazo\":%u,\"laser_brazo\":%u,\"rot_imu_deg\":%.3f,\"gyro_bias\":[%.5f,%.5f,%.5f],\"ang_offset\":[%.5f,%.5f],\"altura_en_sensor\":true}\n",
      montaje_dsm::REVISION,montaje_dsm::IMU_BRAZO,montaje_dsm::LASER_BRAZO,rotImuDeg,
      gyroBias[0],gyroBias[1],gyroBias[2],angOffset[0],angOffset[1]);
    return;
  }
  if (!strcmp(c, "estado_sensores") || !strcmp(c, "ver_sensores")) {
    Out.printf("{\"version\":\"dsm-octo-0.4-banco\",\"estado\":%u,\"motores_bloqueados\":%s,\"imu_ok\":%s,\"imu_edad_ms\":%lu,\"laser_presente\":%s,\"laser_estado\":%u,\"laser_mm\":%u,\"laser_edad_ms\":%ld,\"z_valida\":%s,\"z_cm\":%.2f,\"objetivo_cm\":%.1f}\n",
      estado,DSM_PERMITIR_MOTORES?"false":"true",octo::imuVigente(ultimaImuOk,millis(),ultimaImuMs)?"true":"false",
      (unsigned long)(millis()-ultimaImuMs),tofPresente?"true":"false",vl53::estadoRango,vl53::ultimoMm,
      vl53::ultimaValidaMs?(long)(millis()-vl53::ultimaValidaMs):-1L,zValida?"true":"false",zActualCm,ctrlZ.zRefCm);
    return;
  }
  if(!strcmp(c,"sim_control") && n==2){
    cortarMotores("referencia simulada sin PWM",false);
    sombraSim=v[0]!=0;sombraPpo=false;controlSim.reiniciar();spYawDeg=yawDeg;referenciaActualM=zActualCm*0.01f;
    Out.println(F("# PI-D de simulacion en banco: fuerzas en N; mapa CAD/hardware y curva PWM NO verificados"));return;
  }
  if(!strcmp(c,"ppo_control") && n==2){
    cortarMotores("red neuronal PPO sin PWM",false);
    sombraSim=v[0]!=0;sombraPpo=v[0]!=0;for(float &a:accionesPpoPrev)a=0;spYawDeg=yawDeg;referenciaActualM=zActualCm*0.01f;
    Out.println(F("# Red neuronal PPO (Isaac Lab 17->64->64->8) en banco: fuerzas en N a partir de IMU+laser reales"));return;
  }
  if(!strcmp(c,"pid") || !strcmp(c,"pid_yaw") || !strcmp(c,"pid_z")){
    Out.println(F("# Este candidato usa ganancias SI exportadas de ControlReferencia; PID heredado en us no se aplica"));return;
  }
  if (!strcmp(c, "i2c")) {
    if(estado!=EST_DESARMADO && estado!=EST_ARMADO){Out.println(F("# i2c solo con motores detenidos"));return;}
    escanearBusI2C(); return;
  }
  if (!strcmp(c, "cero_z") || !strcmp(c, "z0")) {
    reiniciarAlturaZ();
    Out.printf("# Altura Z puesta a 0.0 cm (base laser = %.0f mm)\n", zBaseMm);
    return;
  }
  if (!strcmp(c, "?") || !strcmp(c, "ayuda")) { mostrarTablaMotores(); return; }
  if (!strcmp(c, "ok")) { confirmarCalibracionEsc(); return; }
  if (!strcmp(c, "p") && n == 2) { nivelTelemetria = (uint8_t)constrain((int)v[0], 0, 3); return; }
  if (!strcmp(c, "hb")) return;  // latido del puente ROS 2 (solo refresca el watchdog)
  if (!strcmp(c, "watchdog") && n == 2) {
    watchdogSerie = ((int)v[0] != 0);
    Out.printf("# watchdog serie = %s (sin datos > %lu ms en vuelo -> aterrizar)\n",
                  watchdogSerie ? "ACTIVO" : "APAGADO", (unsigned long)WATCHDOG_MS);
    return;
  }

  // Ajustes permitidos en vivo
  if (!strcmp(c, "aterrizar") || !strcmp(c, "bajar_suave")) {
    if (estado == EST_CONTROL) {
      estado = EST_ATERRIZANDO;
      motivoParada = "aterrizaje suave";
      Out.println(F("# ATERRIZANDO: bajando empuje en rampa suave con control de actitud activo..."));
    } else {
      cortarMotores("orden aterrizar", true);
    }
    return;
  }
  if (!strcmp(c, "+") || !strcmp(c, "subir")) {
    float paso = (n >= 2 && v[0] >= 5.0f && v[0] <= 100.0f) ? v[0] : 25.0f;
    baseObjetivoUs = constrain(baseObjetivoUs + paso, 1100.0f, 1900.0f);
    Out.printf("# base objetivo (+) = %.0f us\n", baseObjetivoUs);
    return;
  }
  if (!strcmp(c, "-") || !strcmp(c, "bajar")) {
    float paso = (n >= 2 && v[0] >= 5.0f && v[0] <= 100.0f) ? v[0] : 25.0f;
    baseObjetivoUs = constrain(baseObjetivoUs - paso, 1100.0f, 1900.0f);
    Out.printf("# base objetivo (-) = %.0f us\n", baseObjetivoUs);
    return;
  }
  if (!strcmp(c, "base") && n == 2 && v[0] >= 1000 && v[0] <= 1920) {
    baseObjetivoUs = v[0];
    Out.printf("# base objetivo = %.0f us\n", baseObjetivoUs);
    return;
  }
  if (!strcmp(c, "sp") && n >= 3) {
    spPitchDeg = constrain(v[0], -15.0f, 15.0f);
    spRollDeg  = constrain(v[1], -15.0f, 15.0f);
    Out.printf("# consigna pitch=%+.1f roll=%+.1f deg\n", spPitchDeg, spRollDeg);
    return;
  }
  if (!strcmp(c, "centro")) { spPitchDeg = 0.0f; spRollDeg = 0.0f; return; }
  if (!strcmp(c, "pid") && n == 4) {
    pidPitch.kp = pidRoll.kp = constrain(v[0], 0.0f, 30.0f);
    pidPitch.ki = pidRoll.ki = constrain(v[1], 0.0f, 20.0f);
    pidPitch.kd = pidRoll.kd = constrain(v[2], 0.0f, 15.0f);
    Out.printf("# PID Pitch/Roll: Kp=%.2f Ki=%.2f Kd=%.2f\n", pidPitch.kp, pidPitch.ki, pidPitch.kd);
    return;
  }
  if (!strcmp(c, "pid_yaw") && n == 3) {
    pidYaw.kp = constrain(v[0], 0.0f, 20.0f);
    pidYaw.kd = constrain(v[1], 0.0f, 15.0f);
    Out.printf("# PID Yaw: Kp=%.2f Kd=%.2f\n", pidYaw.kp, pidYaw.kd);
    return;
  }
  if (!strcmp(c, "pid_z") && n == 4) {
    ctrlZ.kp = constrain(v[0], 0.0f, 50.0f);
    ctrlZ.ki = constrain(v[1], 0.0f, 30.0f);
    ctrlZ.kd = constrain(v[2], 0.0f, 30.0f);
    Out.printf("# PID Z: Kp=%.1f Ki=%.1f Kd=%.1f\n", ctrlZ.kp, ctrlZ.ki, ctrlZ.kd);
    return;
  }
  if (!strcmp(c, "zref") && n == 2) {
    ctrlZ.zRefCm = constrain(v[0], 1.0f, 50.0f);
    Out.printf("# z_ref = %.1f cm\n", ctrlZ.zRefCm);
    return;
  }
  if (!strcmp(c, "modo_z") && n == 2) {
    if(v[0]!=0 && !zValida){Out.println(F("# ERROR: altura laser sin medida valida reciente; modo_z sigue apagado"));return;}
    ctrlZ.activo = ((int)v[0] != 0);
    ctrlZ.reiniciar();
    Out.printf("# control altura Z = %s\n", ctrlZ.activo ? "ACTIVO" : "APAGADO");
    return;
  }

  if (estado == EST_CONTROL || estado == EST_ATERRIZANDO || estado == EST_PRUEBA_MOTOR || estado == EST_CAL_ESC_ESPERA_OK) {
    Out.println(F("# Primero detén los motores con 'stop', 'aterrizar' o '!'"));
    return;
  }

  // ---- Comandos solo con motores detenidos ----
  if (!strcmp(c, "alto") && n == 2) { pruebaPinAlto((uint8_t)v[0]); return; }
  if (!strcmp(c, "armar")) { armarEsc(); return; }
  if (!strcmp(c, "cal") && n == 2 && !strcmp(tok[1], "si")) { iniciarCalibracionEsc(); return; }
  if (!strcmp(c, "nivel_imu") || !strcmp(c, "imu")) { calibrarImuEnReposo(); reiniciarAlturaZ(); return; }
  if (!strcmp(c, "test_laser") || !strcmp(c, "laser") || !strcmp(c, "vl53")) { probarLaserVl53(); return; }
  if (!strcmp(c, "frente_imu")) { calibrarFrenteImu(); return; }
  if (!strcmp(c, "rot_imu") && n == 2) {
    rotImuDeg = constrain(v[0], -180.0f, 180.0f);
    filtroImuIniciado = false;
    Out.printf("# rot_imu = %+.1f deg\n", rotImuDeg);
    return;
  }
  if (!strcmp(c, "verificar_imu") || !strcmp(c, "test_imu")) { verificarImuEnVivo(); return; }
  if (!strcmp(c, "orientacion_imu") && n == 5) {
    swapXYImu = ((int)v[0] != 0);
    signoPitchImu = (v[1] < 0) ? -1 : 1;
    signoRollImu  = (v[2] < 0) ? -1 : 1;
    signoYawImu   = (v[3] < 0) ? -1 : 1;
    filtroImuIniciado = false;
    Out.printf("# orientacion IMU: swapXY=%d sPitch=%+d sRoll=%+d sYaw=%+d\n",
                  swapXYImu, signoPitchImu, signoRollImu, signoYawImu);
    return;
  }
  if (!strcmp(c, "trim") && n == 3 && v[0] >= 1 && v[0] <= 8) {
    int idx = (int)v[0] - 1;
    mezclador.trim[idx] = constrain(v[1], -100.0f, 100.0f);
    Out.printf("# trim M%d = %+.1f us\n", idx + 1, mezclador.trim[idx]);
    return;
  }
  if (!strcmp(c, "vbat_corte") && n == 2) {
    corteVbatActivo = ((int)v[0] != 0);
    Out.printf("# corte por bateria baja = %s\n", corteVbatActivo ? "ACTIVO" : "APAGADO");
    return;
  }
  if (!strcmp(c, "vbat_min") && n == 2 && v[0] >= 9.0f && v[0] <= 12.0f) {
    vbatMin = v[0];
    Out.printf("# vbat_min = %.2f V\n", vbatMin);
    return;
  }
  if (!strcmp(c, "frec_esc") && n == 2 && v[0] >= 50 && v[0] <= 400) {
    frecEscHz = (int)v[0];
    if (pwmConectado) conectarEscPwm();
    Out.printf("# frecuencia PWM ESC = %d Hz\n", frecEscHz);
    return;
  }
  if (!strcmp(c, "giro")) {
    usGiroPrueba = (n >= 2) ? constrain(v[0], 1050.0f, 1450.0f) : 1150.0f;
    if (estado == EST_DESARMADO) armarEsc();
    mandarTodosUs(1000.0f);
    enSecuenciaGiro = true;
    motorGiroActual = 0;
    tCambioGiroMs = millis();
    pulsoActUs[0] = pulsoObjUs[0] = usGiroPrueba;
    estado = EST_PRUEBA_MOTOR;
    motivoParada = "secuencia giro M1..M8";
    Out.printf("# GIRO: M1 (GPIO%u, rama %s) a %.0f us por 2 s...\n", PIN_ESC[0], RAMA_ESC[0], usGiroPrueba);
    return;
  }
  if (!strcmp(c, "m") && n >= 3 && v[0] >= 1 && v[0] <= 8) {
    uint8_t idx = (uint8_t)v[0] - 1;
    float us = constrain(v[1], 1000.0f, 2000.0f);
    float seg = (n >= 4) ? constrain(v[2], 0.2f, 15.0f) : 2.0f;
    if (estado == EST_DESARMADO) armarEsc();
    mandarTodosUs(1000.0f);
    pulsoActUs[idx] = pulsoObjUs[idx] = us;
    finPruebaMs = millis() + (uint32_t)(seg * 1000.0f);
    estado = EST_PRUEBA_MOTOR;
    motivoParada = "prueba individual m";
    Out.printf("# Motor M%u (GPIO%u) a %.0f us durante %.1f s ('!' para cortar)\n", idx + 1, PIN_ESC[idx], us, seg);
    return;
  }
  if (!strcmp(c, "todos") && n >= 2) {
    float us = constrain(v[0], 1000.0f, 2000.0f);
    float seg = (n >= 3) ? constrain(v[1], 0.2f, 20.0f) : 3.0f;
    if (estado == EST_DESARMADO) armarEsc();
    mandarTodosUs(us);
    finPruebaMs = millis() + (uint32_t)(seg * 1000.0f);
    estado = EST_PRUEBA_MOTOR;
    motivoParada = "prueba grupal todos";
    Out.printf("# TODOS los 8 motores a %.0f us durante %.1f s ('!' para cortar)\n", us, seg);
    return;
  }
    if (!strcmp(c, "iniciar")) {
    if(!hardware_real::VERIFICADO){Out.println(F("# BLOQUEADO: falta curva empuje-PWM y mapa CAD/motores fisicos verificados"));return;}
    if (!imuLista && !iniciarMpu6050()) {
      Out.println(F("# ERROR: no se puede iniciar estabilizacion sin MPU-6050."));
      return;
    }
    leerMpuPaso(0.004f);actualizarLaser();leerBaterias();
    if(!octo::imuVigente(ultimaImuOk,millis(),ultimaImuMs) || fabsf(pitchDeg)>10 || fabsf(rollDeg)>10){Out.println(F("# ERROR: IMU no vigente o inclinacion inicial >10 deg"));return;}
    if(watchdogSerie && millis()-ultimoRxMs>WATCHDOG_MS){Out.println(F("# ERROR: enlace sin latido reciente"));return;}
    if(!corteVbatActivo || vbatA<vbatMin || vbatB<vbatMin){Out.println(F("# ERROR: confirmar ambas baterias y corte de baja tension"));return;}
    if(ctrlZ.activo && !zValida){Out.println(F("# ERROR: sin altura valida no se inicia control Z"));return;}
    if (n >= 2 && v[0] >= 1120 && v[0] <= 1900) baseObjetivoUs = v[0];
    if (estado == EST_DESARMADO) armarEsc();
    pidPitch.reiniciar();
    sombraSim=false;
    pidRoll.reiniciar();
    pidYaw.reiniciar();
    ctrlZ.reiniciar();
    controlSim.reiniciar();referenciaActualM=zActualCm*0.01f;
    spYawDeg = yawDeg;
    baseActualUs = mezclador.usMinActivo;
    usCicloMax = 0;
    estado = EST_CONTROL;
    motivoParada = "control PI-D activo";
    Out.printf("# CONTROL OCTO ACTIVO: rampa hacia base=%.0f us ('!' o 'stop' para cortar)\n", baseObjetivoUs);
    return;
  }
  if (!strcmp(c, "guardar")) { guardarConfig(); return; }
  if (!strcmp(c, "fabrica")) {
    EEPROM.put(DIR_CFG, (uint16_t)0);
    EEPROM.commit();
    Out.println(F("# configuracion borrada; reinicia o ajusta valores"));
    return;
  }
  Out.println(F("# comando no reconocido ('?' para ayuda)"));
}

void leerPuertoSerie() {
  int chInt;
  while ((chInt = leerByteEntrada()) >= 0) {
    char ch = (char)chInt;
    ultimoRxMs = millis();
    if (ch == '!') {
      nL = 0;
      cortarMotores("parada de emergencia '!'", true);
      continue;
    }
    if (ch == '\r') continue;
    if (ch == '\n') {
      linea[nL] = 0;
      if (nL) procesarComando(linea);
      nL = 0;
    } else if (nL < sizeof(linea) - 1) {
      linea[nL++] = ch;
    }
  }
}

// ===================== SETUP Y LOOP A 250 HZ =====================
void setup() {
  desconectarEscLow();
  for (int i = 0; i < 2; i++) pinMode(PIN_SERVO_OLD[i], INPUT);
  analogReadResolution(12);
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  delay(350);
  Out.println(F("\n# DSM DRON OCTO EN ROTULA (8 MOTORES + MPU6050 + CONTROL PI-D) - Rev D"));
  Out.printf("# dsm-octo-0.4-banco | motores %s | candidato sin validacion de vuelo libre\n",DSM_PERMITIR_MOTORES?"HABILITADOS EN COMPILACION":"BLOQUEADOS SIN PWM");

  EEPROM.begin(TAM_EEPROM);
  // Ganancias por defecto diferenciadas para Yaw
  pidYaw.kp = 3.0f;
  pidYaw.ki = 0.0f;
  pidYaw.kd = 1.0f;
  pidYaw.uMax = 100.0f;
  cargarConfig();

  iniciarMpu6050();
  tofPresente=vl53::iniciar();
  leerBaterias();
  xTaskCreatePinnedToCore(tareaEsc, "esc8", 2048, nullptr, 2, nullptr, 0);
  if (enlace::iniciar()) {
    Out.printf("# WiFi AP activo: SSID '%s' | clave '%s' | UDP 192.168.4.1:%u\n",
               enlace::RED, enlace::CLAVE, enlace::PUERTO);
  }

  Out.println(F("# DESARMADO (8 pines en LOW). Escribe '?' para ver la tabla de motores y comandos."));
  proximoUs = ultimoUs = micros();
}

void loop() {
  leerPuertoSerie();

  uint32_t ahoraUs = micros();
  if ((int32_t)(ahoraUs - proximoUs) < 0) return;
  proximoUs += periodoUs;
  if ((int32_t)(ahoraUs - proximoUs) > 0) proximoUs = ahoraUs + periodoUs;
  uint32_t dtUs = ahoraUs - ultimoUs;
  ultimoUs = ahoraUs;
  float dt = constrain(dtUs * 1e-6f, 0.001f, 0.05f);
  nPaso++;

  uint32_t t0 = micros();
  leerMpuPaso(dt);
  actualizarLaser();
  if(sombraSim){
    bool ok=calcularReferenciaSim(dt);
    if(nPaso%25==0)Out.printf("{\"version\":\"%s\",\"valido\":%s,\"objetivo_cm\":%.1f,\"referencia_rampa_cm\":%.2f,\"fuerzas_N\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],\"pwm_habilitado\":false,\"marco_CAD_verificado\":false}\n",
      sombraPpo?"dsm-ppo-red-banco":"dsm-control-sim-banco",ok?"true":"false",ctrlZ.zRefCm,referenciaActualM*100,fuerzasSim[0],fuerzasSim[1],fuerzasSim[2],fuerzasSim[3],fuerzasSim[4],fuerzasSim[5],fuerzasSim[6],fuerzasSim[7]);
  }
  if((estado==EST_CONTROL || estado==EST_ATERRIZANDO) &&
      (!octo::imuVigente(ultimaImuOk,millis(),ultimaImuMs) || dtUs>25000)) {
    cortarMotores("IMU no valida o periodo de control >25 ms",false);return;
  }
  if(estado==EST_CONTROL && ctrlZ.activo && !zValida){
    ctrlZ.activo=false;ctrlZ.reiniciar();estado=EST_ATERRIZANDO;
    motivoParada="perdida de altura laser valida";
    Out.println(F("# Altura perdida: retirar correccion Z y reducir empuje; no es aterrizaje autonomo validado"));
  }
  if (nPaso % 25 == 0) {
    leerBaterias();
    // Corte por bateria baja (solo si la bateria esta conectada, es decir > 6.5 V)
    if (corteVbatActivo && (estado == EST_CONTROL || estado == EST_PRUEBA_MOTOR)) {
      if ((vbatA > 6.5f && vbatA < vbatMin) || (vbatB > 6.5f && vbatB < vbatMin)) {
        cortarMotores("bateria baja (< vbat_min)", true);
      }
    }
  }

  if (estado == EST_PRUEBA_MOTOR) {
    if (enSecuenciaGiro) {
      if (millis() - tCambioGiroMs >= 2000) {
        pulsoActUs[motorGiroActual] = pulsoObjUs[motorGiroActual] = 1000.0f;
        motorGiroActual++;
        if (motorGiroActual >= 8) {
          cortarMotores("fin secuencia giro", true);
        } else {
          tCambioGiroMs = millis();
          pulsoActUs[motorGiroActual] = pulsoObjUs[motorGiroActual] = usGiroPrueba;
          Out.printf("# GIRO: M%u (GPIO%u, rama %s) a %.0f us por 2 s...\n",
                        motorGiroActual + 1, PIN_ESC[motorGiroActual], RAMA_ESC[motorGiroActual], usGiroPrueba);
        }
      }
    } else if (finPruebaMs && (int32_t)(millis() - finPruebaMs) >= 0) {
      cortarMotores("fin tiempo de prueba", true);
    }
  } else if (estado == EST_CONTROL || estado == EST_ATERRIZANDO) {
    if (estado == EST_CONTROL && watchdogSerie && (millis() - ultimoRxMs) > WATCHDOG_MS) {
      estado = EST_ATERRIZANDO;
      motivoParada = "watchdog: se perdio el enlace serie";
      Out.println(F("# WATCHDOG: sin latido del PC -> ATERRIZANDO"));
    }
    if (fabsf(pitchDeg) > angMaxSeguridad || fabsf(rollDeg) > angMaxSeguridad) {
      cortarMotores("inclinacion excesiva (> ang_max)", true);
      return;
    }

    if (estado == EST_ATERRIZANDO) {
      baseActualUs -= rampaUsPorSeg * dt;
      if (baseActualUs <= mezclador.usMinActivo) {
        cortarMotores("aterrizaje suave completado", true);
        return;
      }
    } else {
      // Rampa suave hacia baseObjetivoUs
      if (baseActualUs < baseObjetivoUs) {
        baseActualUs = min(baseObjetivoUs, baseActualUs + rampaUsPorSeg * dt);
      } else if (baseActualUs > baseObjetivoUs) {
        baseActualUs = max(baseObjetivoUs, baseActualUs - rampaUsPorSeg * dt);
      }
    }

    float sal[8];
    if(!calcularReferenciaSim(dt) || !hardware_real::convertir(fuerzasSim,sal)){
      cortarMotores("referencia sin sensores o conversion empuje-PWM no verificada",false);return;
    }
    for (uint8_t i = 0; i < 8; i++) {
      pulsoActUs[i] = sal[i];
      pulsoObjUs[i] = sal[i];
    }
  }

  emitirTelemetria();
  usCiclo = micros() - t0;
  if (estado == EST_CONTROL && usCiclo > usCicloMax) usCicloMax = usCiclo;
}
