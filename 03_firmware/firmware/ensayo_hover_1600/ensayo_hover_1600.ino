// Ensayo guiado: consigna roll/pitch cero, limite absoluto 1600 us.
// Derivado del control M8 probado. NO tiene control de altura ni detector de hover.
// Boot/STOP: OFF y 1000 us. Arranque exclusivamente por armar + arrancar.
// IMU M8, +X=M8 +Y=M7. ToF M1 desactivado en este ensayo.
#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#include <driver/mcpwm_prelude.h>
#include <math.h>
#include <string.h>
#include "PoliticaEnsayo.h"

#if !defined(CONFIG_IDF_TARGET_ESP32) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Seleccionar ESP32 clasico y Arduino-ESP32 3.x."
#endif

constexpr uint8_t pins[] = {25, 26, 27, 14, 21, 19, 5, 17};
constexpr int stopUs = 1000, spinUs = 1140, floorUs = 1180;
constexpr int baseUs = ensayo::baseUs, maxUs = ensayo::maxUs;
constexpr uint32_t sampleUs = 5000, periodUs = 20000;
constexpr float rad = PI / 180.0f;
constexpr float cutoffTiltDeg = ensayo::cutoffTiltDeg;
// FLU: +X hacia M8, +Y hacia M7, +Z arriba. Orden fisico M1..M8.
constexpr float L = .350f, D = .24748737f;
constexpr float x[] = { D, -D, -D, D, 0, -L, 0, L};
constexpr float y[] = {-D, -D,  D, D,-L,  0, L, 0};
// Visto desde arriba: M1..M4 horario (-1), M5..M8 antihorario (+1).
constexpr int spin[] = {-1, -1, -1, -1, 1, 1, 1, 1};
constexpr bool enableYaw = false;
constexpr float kpAngle[] = {4.62f, 4.65f};
constexpr float kdAngle[] = {.12f, .12f};
constexpr float kiAngle = .8f;
constexpr float angleIntegralRateLimit = 10*rad;
constexpr float angleDerivativeTau = .025f; // seconds
constexpr float kpRate[] = {.848f, .842f, .378f};
constexpr float kiRate[] = {.100f, .100f, .288f};

enum Mode { OFF, PREVIEW, ARMED, RAMP, CONTROL };
Mode mode = OFF;
bool pwmOK = false, imuOK = false, calibrated = false, calibrating = false;
uint8_t address = 0;
uint32_t bootMs, armMs, lastSample, lastOutput, lastReport;
int rampPulse = stopUs;
int commandUs[8];
int appliedUs[8];
mcpwm_cmpr_handle_t pwmComparator[8];
mcpwm_gen_handle_t pwmGenerator[8];
uint8_t i2cStatus = 0;
size_t i2cBytes = 0;
uint32_t pwmWriteUs = 0;
uint32_t peakPwmWriteUs = 0;
float roll, pitch, yaw, rates[3], accelNorm, referenceNorm = 1;
ensayo::AccelGuard accelGuard;
float gyroBias[3], correction[3][3], integral[3];
float filteredAngleRate[2];
float angleIntegralRate[2];
float yawRow[8], yawNull[8], yawAuthority;
float scaleRP = 1, scaleYaw = 1;
float forceBase, forceMin, forceMax;
float sums[6], squares[6];
unsigned calCount = 0;
bool hbSeen = false;
uint32_t hbMs = 0, runMs = 0, lastAdcMs = 0, intervalLastUs = 0, imuReadLastUs = 0;
uint32_t telemetrySequence = 0, telemetryDropped = 0;
float collectiveUs = baseUs;
int targetUs = baseUs;
uint16_t adcRaw[2] = {}, cutReason = 0;
// cutReason bits: 1 manual, 2 heartbeat, 4 tilt, 8 yaw, 16 yaw-rate,
// 32 accel, 64 IMU/timing, 128 PWM, 256 duration, 512 rejected command.
struct __attribute__((packed)) Telemetry {
  uint32_t magic;
  uint16_t size, version;
  uint32_t sequence, ms, us;
  uint8_t mode, flags;
  uint16_t collectiveUs;
  float modelBase, attitude[3], rates[3];
  int16_t raw[6];
  float scaleRP, scaleYaw;
  uint16_t pwm[8];
  float angleIntegral[2], rateIntegral[2];
  uint32_t intervalUs, readUs, pwmWriteUs, dropped;
  uint16_t adc[2], cutReason;
  float accelRaw, accelFiltered;
  uint32_t accelOutsideUs;
  uint8_t accelTrip, accelRevision;
  uint16_t reserved, crc;
};
static_assert(sizeof(Telemetry) == 144, "Protocol size");
char line[48];
size_t length = 0;
bool discard = false;

// Respaldo circular: solo los ultimos ~8 s. El PC guarda el flujo completo de 200 Hz.
struct RawImu { int16_t accel[3], gyro[3]; };
struct LogSample {
  uint32_t tUs;
  RawImu imu;
  float attitude[3], scale;
  uint16_t pwm[8];
  uint8_t phase;
};
constexpr size_t logCapacity = 1600;
LogSample flightLog[logCapacity];
size_t logNext = 0, logCount = 0, logSent = 0, logByteOffset = 0;
uint32_t runStartUs = 0, markUs = 0;
bool marked = false, exportingLog = false;

void saveSample(uint32_t now, const RawImu &imu) {
  LogSample &sample = flightLog[logNext];
  sample.tUs = now-runStartUs;
  sample.imu = imu;
  sample.attitude[0] = roll;
  sample.attitude[1] = pitch;
  sample.attitude[2] = yaw;
  sample.scale = scaleRP;
  for (int i = 0; i < 8; ++i) sample.pwm[i] = appliedUs[i];
  sample.phase = mode;
  logNext = (logNext+1) % logCapacity;
  if (logCount < logCapacity) ++logCount;
}

void startLogExport() {
  if (mode != OFF || calibrating || !logCount) {
    Serial.println("# LOG disponible desarmado, despues de una prueba.");
    return;
  }
  logSent = 0;
  logByteOffset = 0;
  exportingLog = true;
  Serial.printf("# LOG INICIO muestras=%u marca_us=%ld\n",
    static_cast<unsigned>(logCount), marked ? static_cast<long>(markUs) : -1L);
  Serial.println("t_us,fase,ax_g,ay_g,az_g,gx_sensor_dps,gy_sensor_dps,gz_sensor_dps,roll_deg,pitch_deg,yaw_rel_deg,escala_RP,M1_us,M2_us,M3_us,M4_us,M5_us,M6_us,M7_us,M8_us");
}

void streamLog() {
  if (!exportingLog) return;
  if (logSent == logCount) {
    Serial.println("# LOG FIN");
    exportingLog = false;
    return;
  }
  const size_t i = (logNext+logCapacity-logCount+logSent) % logCapacity;
  const LogSample &s = flightLog[i];
  char row[256];
  const int n = snprintf(row,sizeof(row),
    "%lu,%s,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%u\n",
    static_cast<unsigned long>(s.tUs), s.phase == RAMP ? "RAMPA" : "CONTROL",
    s.imu.accel[0]/16384.0f, s.imu.accel[1]/16384.0f, s.imu.accel[2]/16384.0f,
    s.imu.gyro[0]/131.0f, s.imu.gyro[1]/131.0f, s.imu.gyro[2]/131.0f,
    s.attitude[0]/rad, s.attitude[1]/rad, s.attitude[2]/rad, s.scale,
    unsigned(s.pwm[0]), unsigned(s.pwm[1]), unsigned(s.pwm[2]), unsigned(s.pwm[3]),
    unsigned(s.pwm[4]), unsigned(s.pwm[5]), unsigned(s.pwm[6]), unsigned(s.pwm[7]));
  if (n <= 0 || n >= int(sizeof(row))) {
    Serial.println("# LOG ERROR de formato");
    exportingLog = false;
  } else {
    const int bytes = min(Serial.availableForWrite(),n-int(logByteOffset));
    if (bytes <= 0) return;
    logByteOffset += Serial.write(reinterpret_cast<const uint8_t *>(row)+logByteOffset,bytes);
    if (logByteOffset == size_t(n)) {
      logByteOffset = 0;
      ++logSent;
    }
  }
}

float clip(float value, float lo, float hi) {
  return fminf(hi, fmaxf(lo, value));
}

bool driving() { return mode == RAMP || mode == CONTROL; }

float modelForce(int pulse) {
  // Curva provisional de MATLAB, con la misma escala nominal para los 8.
  return 5.3955f * powf(fmaxf(0, (pulse - spinUs) / 860.0f), 1.6f);
}

int pulseForForce(float force) {
  const float pulse = spinUs + 860.0f * powf(force / 5.3955f, 1.0f / 1.6f);
  return lroundf(clip(pulse, floorUs, maxUs));
}

void disablePWM() {
  pwmOK = false;
  for (auto generator : pwmGenerator) {
    if (generator) mcpwm_generator_set_force_level(generator, 0, true);
  }
}

esp_err_t initPWM() {
  // Un timer de 50 Hz por grupo y dos operadores con dos salidas cada uno.
  mcpwm_timer_handle_t timers[2] = {};
  for (int group = 0; group < 2; ++group) {
    mcpwm_timer_config_t timerConfig = {};
    timerConfig.group_id = group;
    timerConfig.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timerConfig.resolution_hz = 1000000;
    timerConfig.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timerConfig.period_ticks = periodUs;
    esp_err_t err = mcpwm_new_timer(&timerConfig, &timers[group]);
    if (err != ESP_OK) return err;

    for (int pair = 0; pair < 2; ++pair) {
      mcpwm_operator_config_t operatorConfig = {};
      operatorConfig.group_id = group;
      mcpwm_oper_handle_t pwmOperator = nullptr;
      if ((err = mcpwm_new_operator(&operatorConfig, &pwmOperator)) != ESP_OK) return err;
      if ((err = mcpwm_operator_connect_timer(pwmOperator, timers[group])) != ESP_OK) return err;

      for (int output = 0; output < 2; ++output) {
        const int i = 4*group + 2*pair + output;
        mcpwm_comparator_config_t compareConfig = {};
        compareConfig.flags.update_cmp_on_tez = true;
        if ((err = mcpwm_new_comparator(pwmOperator, &compareConfig, &pwmComparator[i])) != ESP_OK) return err;
        if ((err = mcpwm_comparator_set_compare_value(pwmComparator[i], stopUs)) != ESP_OK) return err;

        mcpwm_generator_config_t generatorConfig = {};
        generatorConfig.gen_gpio_num = pins[i];
        if ((err = mcpwm_new_generator(pwmOperator, &generatorConfig, &pwmGenerator[i])) != ESP_OK) return err;
        if ((err = mcpwm_generator_set_action_on_timer_event(pwmGenerator[i],
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
              MCPWM_GEN_ACTION_HIGH))) != ESP_OK) return err;
        if ((err = mcpwm_generator_set_action_on_compare_event(pwmGenerator[i],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, pwmComparator[i],
              MCPWM_GEN_ACTION_LOW))) != ESP_OK) return err;
        appliedUs[i] = stopUs;
      }
    }
  }
  for (auto timer : timers) {
    esp_err_t err = mcpwm_timer_enable(timer);
    if (err != ESP_OK) return err;
    if ((err = mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP)) != ESP_OK) return err;
  }
  return ESP_OK;
}

bool writePWM(const int *pulses) {
  if (!pwmOK) return false;
  const uint32_t start = micros();
  for (int i = 0; i < 8; ++i) {
    const int pulse = constrain(pulses[i], stopUs, maxUs);
    // Cada tick vale 1 us. El hardware adopta el valor en el siguiente periodo.
    if (mcpwm_comparator_set_compare_value(pwmComparator[i], pulse) != ESP_OK) {
      disablePWM();
      return false;
    }
    appliedUs[i] = pulse;
  }
  pwmWriteUs = micros()-start;
  if (driving()) peakPwmWriteUs = max(peakPwmWriteUs, pwmWriteUs);
  return true;
}

bool uniformPWM(int pulse) {
  for (int &out : commandUs) out = pulse;
  return writePWM(commandUs);
}

void stop(const char *reason) {
  const bool wasDriving = driving();
  mode = OFF;
  calibrating = false;
  rampPulse = stopUs;
  collectiveUs = baseUs;
  targetUs = baseUs;
  memset(integral, 0, sizeof(integral));
  memset(angleIntegralRate, 0, sizeof(angleIntegralRate));
  memset(filteredAngleRate, 0, sizeof(filteredAngleRate));
  uniformPWM(stopUs);
  Serial.println(reason);
  if (wasDriving) Serial.printf("# PWM escritura_max_us=%lu\n", static_cast<unsigned long>(peakPwmWriteUs));
  if (!pwmOK) Serial.println("# PWM SIN SENAL. Cortar potencia y revisar.");
}

bool readRegs(uint8_t dev, uint8_t reg, uint8_t *out, size_t n) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  i2cStatus = Wire.endTransmission(false);
  i2cBytes = 0;
  if (i2cStatus != 0) return false;
  i2cBytes = Wire.requestFrom(dev, n, true);
  if (i2cBytes != n) return false;
  for (size_t i = 0; i < n; ++i) out[i] = Wire.read();
  return true;
}

bool setReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  uint8_t actual;
  return Wire.endTransmission() == 0 && readRegs(address, reg, &actual, 1)
    && actual == value;
}

bool initializeIMU() {
  // Solo OFF: un reset del ESP32 puede interrumpir una transaccion y dejar SDA
  // retenida por el sensor. Recuperar el bus antes de identificar/configurar.
  if (mode != OFF) return false;
  calibrated = false;
  imuOK = false;
  address = 0;
  Wire.end();
  pinMode(23, INPUT_PULLUP);
  pinMode(22, OUTPUT_OPEN_DRAIN);
  digitalWrite(22, HIGH);
  delayMicroseconds(20);
  for (int i=0;i<9;++i) {
    digitalWrite(22,LOW); delayMicroseconds(10);
    digitalWrite(22,HIGH); delayMicroseconds(10);
  }
  digitalWrite(22,LOW);
  pinMode(23,OUTPUT_OPEN_DRAIN); digitalWrite(23,LOW);
  delayMicroseconds(10);
  digitalWrite(22,HIGH); delayMicroseconds(10);
  digitalWrite(23,HIGH); delayMicroseconds(10); // STOP
  pinMode(22,INPUT_PULLUP); pinMode(23,INPUT_PULLUP);
  const int sda=digitalRead(23), scl=digitalRead(22);
  Wire.begin(23,22,100000);
  Wire.setTimeOut(8);
  delay(100); // alimentacion/arranque de la IMU antes de consultar WHO_AM_I
  for (int attempt=0;attempt<3 && !imuOK;++attempt) {
    for (uint8_t dev=0x68;dev<=0x69;++dev) {
      uint8_t id=0;
      if (readRegs(dev,0x75,&id,1) && id==0x68) { address=dev; break; }
    }
    if (address) {
      imuOK=setReg(0x6B,0x01);
      delay(100);
      imuOK=imuOK && setReg(0x6C,0) && setReg(0x1A,3) && setReg(0x19,4)
        && setReg(0x1B,0) && setReg(0x1C,0);
    }
    Serial.printf("# IMU_INICIO intento=%d direccion=0x%02X ok=%d SDA=%d SCL=%d i2c=%u bytes=%u\n",
      attempt+1,address,imuOK,sda,scl,unsigned(i2cStatus),unsigned(i2cBytes));
    if (!imuOK) { address=0; delay(50); }
  }
  lastSample=micros();
  return imuOK;
}

bool readIMU(float *a, float *g, RawImu &raw) {
  uint8_t data[14];
  if (!readRegs(address, 0x3B, data, sizeof(data))) return false;
  for (int i = 0; i < 3; ++i) {
    raw.accel[i] = int16_t(uint16_t(data[2*i]) << 8 | data[2*i+1]);
    raw.gyro[i] = int16_t(uint16_t(data[8+2*i]) << 8 | data[9+2*i]);
    a[i] = raw.accel[i] / 16384.0f;
    g[i] = raw.gyro[i] / 131.0f * rad;
  }
  return true;
}

void collectCalibration(const float *a, const float *g) {
  for (int i = 0; i < 6; ++i) {
    const float v = i < 3 ? a[i] : g[i-3];
    sums[i] += v;
    squares[i] += v*v;
  }
  if (++calCount < 1000) return;
  calibrating = false;
  bool still = true;
  for (int i = 0; i < 6; ++i) {
    sums[i] /= calCount;
    const float sd = sqrtf(fmaxf(0, squares[i]/calCount - sums[i]*sums[i]));
    still &= i < 3 ? sd < .025f : sd < .8f*rad && fabsf(sums[i]) < 10*rad;
  }
  const float ph = atan2f(sums[1], sums[2]);
  const float th = atan2f(-sums[0], hypotf(sums[1], sums[2]));
  referenceNorm = hypotf(hypotf(sums[0], sums[1]), sums[2]);
  if (!still || hypotf(ph, th) > 10*rad || referenceNorm < .85f || referenceNorm > 1.15f) {
    Serial.println("# CAL RECHAZADA: nivelar, inmovilizar y repetir.");
    return;
  }
  const float cp = cosf(ph), sp = sinf(ph), ct = cosf(th), st = sinf(th);
  const float rotation[3][3] = {
    {ct, st*sp, st*cp}, {0, cp, -sp}, {-st, ct*sp, ct*cp}
  };
  memcpy(correction, rotation, sizeof(correction));
  for (int i = 0; i < 3; ++i) gyroBias[i] = sums[i+3];
  roll = pitch = yaw = 0;
  calibrated = true;
  accelGuard.reset(referenceNorm,micros());
  Serial.printf("# CAL_DATOS muestras=%u norma_g=%.6f bias_rad_s=%.7f,%.7f,%.7f referencia_sensor_deg=%.4f,%.4f\n",
    calCount, referenceNorm, gyroBias[0], gyroBias[1], gyroBias[2], ph/rad, th/rad);
  for (int i=0;i<3;++i) Serial.printf("# CAL_ROT fila=%d %.7f %.7f %.7f\n",
    i, correction[i][0], correction[i][1], correction[i][2]);
  Serial.println("# CAL OK. Enviar vista para revisar signos con potencia desconectada.");
}

void updateAttitude(const float *rawA, const float *rawG, float dt) {
  float a[3] = {};
  for (int i = 0; i < 3; ++i) {
    rates[i] = 0;
    for (int j = 0; j < 3; ++j) {
      a[i] += correction[i][j] * rawA[j];
      rates[i] += correction[i][j] * (rawG[j] - gyroBias[j]);
    }
  }
  accelNorm = hypotf(hypotf(a[0], a[1]), a[2]);
  const float s = sinf(roll), c = cosf(roll);
  const float side = s*rates[1] + c*rates[2];
  const float cosPitch = fmaxf(.25f, cosf(pitch));
  roll += dt * (rates[0] + sinf(pitch)/cosPitch * side);
  pitch += dt * (c*rates[1] - s*rates[2]);
  yaw += dt * side / cosPitch; // giro relativo desde iniciar, solo para corte y registro
  if (fabsf(accelNorm - referenceNorm) < .15f) {
    const float weight = dt / (.25f + dt);
    roll += weight * (atan2f(a[1], a[2]) - roll);
    pitch += weight * (atan2f(-a[0], hypotf(a[1], a[2])) - pitch);
  }
}

void mix(float rollTorque, float pitchTorque, float yawTorque) {
  (void)yawTorque; // yaw sigue deshabilitado, igual que el controlador probado.
  float delta[8], forces[8];
  for (int i=0; i<8; ++i)
    delta[i] = (y[i]*rollTorque-x[i]*pitchTorque)/(4*L*L);
  const int requested = mode == RAMP ? max(floorUs,rampPulse) : lroundf(collectiveUs);
  scaleRP = ensayo::allocate(modelForce(requested), forceMin, forceMax, delta, forces, forceBase);
  scaleYaw = 1;
  for (int i=0; i<8; ++i) commandUs[i] = pulseForForce(forces[i]);
}

void controlStep(float dt) {
  const float sinRoll = sinf(roll), cosRoll = cosf(roll);
  const float tanPitch = tanf(pitch);
  // Differentiate the measured attitude using the gyro, excluding accel corrections.
  const float rollRate = rates[0] + tanPitch*(sinRoll*rates[1] + cosRoll*rates[2]);
  const float pitchRate = cosRoll*rates[1] - sinRoll*rates[2];
  const float alpha = dt/(angleDerivativeTau + dt);
  filteredAngleRate[0] += alpha*(rollRate - filteredAngleRate[0]);
  filteredAngleRate[1] += alpha*(pitchRate - filteredAngleRate[1]);
  const float rollError = -roll, pitchError = -pitch;
  const float rollRateRequest = kpAngle[0]*rollError - kdAngle[0]*filteredAngleRate[0] + angleIntegralRate[0];
  const float pitchRateRequest = kpAngle[1]*pitchError - kdAngle[1]*filteredAngleRate[1] + angleIntegralRate[1];
  const float phiDot = clip(rollRateRequest, -25*rad, 25*rad);
  const float thetaDot = clip(pitchRateRequest, -25*rad, 25*rad);
  const float qRef = thetaDot/fmaxf(.5f, cosRoll);
  const float pRef = phiDot - sinRoll*tanPitch*qRef;
  const float error[] = {pRef-rates[0], qRef-rates[1], -rates[2]};
  float torque[3];
  for (int i = 0; i < 3; ++i) torque[i] = kpRate[i]*error[i] + kiRate[i]*integral[i];
  mix(torque[0], torque[1], clip(torque[2], -.025f, .025f));
  if (mode != CONTROL) return;
  if (scaleRP > .999f) {
    if (fabsf(rollRateRequest) < 25*rad && fabsf(pitchRateRequest) < 25*rad) {
      angleIntegralRate[0] = clip(angleIntegralRate[0]+dt*kiAngle*rollError,
                                  -angleIntegralRateLimit, angleIntegralRateLimit);
      angleIntegralRate[1] = clip(angleIntegralRate[1]+dt*kiAngle*pitchError,
                                  -angleIntegralRateLimit, angleIntegralRateLimit);
    }
    for (int i = 0; i < 2; ++i) integral[i] = clip(integral[i]+dt*error[i], -.3f, .3f);
  }
  if (enableYaw && scaleYaw > .999f && fabsf(torque[2]) < .025f)
    integral[2] = clip(integral[2]+dt*error[2], -.08f, .08f);
}

bool heartbeatFresh() { return ensayo::fresh(hbSeen, millis(), hbMs); }

bool readyToArm() {
  return ensayo::canArm(pwmOK, imuOK, calibrated, calibrating, heartbeatFresh(),
    ensayo::tiltDegrees(roll,pitch), hypotf(hypotf(rates[0],rates[1]),rates[2])/rad);
}

void command(const char *s) {
  if (!*s) return;
  if (!strcmp(s,"parar") || !strcmp(s,"apagar")) {
    cutReason = 1;
    stop("# PARADA MANUAL: DESARMADO, 1000 us. Rearme manual obligatorio.");
    return;
  }
  if (!strcmp(s,"hb")) { hbSeen = true; hbMs = millis(); return; }
  if (!strcmp(s,"estado")) {
    Serial.printf("# ENSAYO_HOVER_V1 modo=%d cal=%d calibrando=%d imu=%d pwm=%d hb=%d colectivo=%.1f objetivo=%d max=%d corte=%.0f motivo=%u\n",
      int(mode), calibrated, calibrating, imuOK, pwmOK, heartbeatFresh(), collectiveUs, targetUs,
      maxUs, cutoffTiltDeg, unsigned(cutReason));
    Serial.printf("# ACCEL rev=%u tau_ms=20 persist_ms=40 limites_filtrados=0.55..1.55 extremos_raw=0.20..1.90 raw_g=%.4f filtrada_g=%.4f fuera_us=%lu ultimo_tipo=%u\n",
      unsigned(ensayo::accelPolicyRevision),accelGuard.raw,accelGuard.filtered,
      static_cast<unsigned long>(accelGuard.outsideUs),unsigned(accelGuard.lastTrip));
    return;
  }
  if (!strcmp(s,"imu_reiniciar") && mode==OFF && !calibrating) {
    initializeIMU();
    Serial.println("# IMU reiniciada en OFF. Repetir cal antes de armar.");
    return;
  }
  if (exportingLog) { Serial.println("# Esperar LOG FIN antes de otra orden."); return; }
  if (!strcmp(s,"log")) { startLogExport(); return; }
  if (!strcmp(s,"cal") && mode == OFF && imuOK && millis()-bootMs >= 4000) {
    stop("# CAL: 5 s inmovil y nivelado fisicamente, motores apagados.");
    calibrated = false;
    calCount = 0;
    memset(sums,0,sizeof(sums)); memset(squares,0,sizeof(squares));
    calibrating = true;
    return;
  }
  if (!strcmp(s,"vista") && mode == OFF && calibrated && !calibrating) {
    mode=PREVIEW;
    Serial.println("# VISTA: motores a 1000 us. Telemetria binaria = salidas aplicadas.");
    return;
  }
  if (!strcmp(s,"armar") && !driving() && mode != ARMED && readyToArm()) {
    exportingLog = false;
    memset(integral,0,sizeof(integral));
    memset(angleIntegralRate,0,sizeof(angleIntegralRate));
    memset(filteredAngleRate,0,sizeof(filteredAngleRate));
    if (!uniformPWM(stopUs)) { cutReason=128; stop("# FALLO PWM"); return; }
    cutReason=0;
    accelGuard.reset(accelNorm,micros());
    mode=ARMED; armMs=millis();
    Serial.println("# ARMADO 10 s. Enviar arrancar. ! = apagar.");
    return;
  }
  if ((!strcmp(s,"arrancar") || !strcmp(s,"iniciar")) && mode == ARMED
      && millis()-armMs < 10000 && readyToArm()) {
    memset(integral,0,sizeof(integral));
    memset(filteredAngleRate,0,sizeof(filteredAngleRate));
    memset(angleIntegralRate,0,sizeof(angleIntegralRate));
    collectiveUs=baseUs; targetUs=baseUs; rampPulse=stopUs;
    peakPwmWriteUs=0; scaleRP=scaleYaw=1; yaw=0;
    logNext=logCount=0; marked=false;
    runStartUs=lastOutput=micros(); runMs=millis(); mode=RAMP;
    Serial.println("# RAMPA INICIAL a 1300 us. Luego mantener. ! o apagar = parada.");
    return;
  }
  if (mode == CONTROL) {
    if (!strcmp(s,"mantener")) targetUs=lroundf(collectiveUs);
    else if (!strcmp(s,"subir")) targetUs=min(maxUs,targetUs+10);
    else if (!strcmp(s,"bajar")) targetUs=max(baseUs,targetUs-10);
    else if (!strcmp(s,"rampa")) targetUs=maxUs;
    else if (!strcmp(s,"rampa1400")) targetUs=1400;
    else if (!strcmp(s,"hover") || !strcmp(s,"marca")) {
      targetUs=lroundf(collectiveUs);
      markUs=micros()-runStartUs; marked=true;
      Serial.printf("# MARCA_MANUAL_SUSTENTACION us=%lu colectivo=%d; NO altura medida ni hover confirmado.\n",
        static_cast<unsigned long>(markUs),targetUs);
    } else {
      cutReason=512; stop("# ORDEN RECHAZADA: DESARMADO."); return;
    }
    Serial.printf("# COLECTIVO objetivo=%d actual=%.1f subida=5_us_s bajada=20_us_s max_motor=1600\n",targetUs,collectiveUs);
    return;
  }
  cutReason=512;
  stop("# ORDEN RECHAZADA: cal | vista | armar | arrancar | apagar | ! | estado");
}

void serialInput() {
  for (int i = 0; i < 32 && Serial.available(); ++i) {
    const char c = Serial.read();
    if (c == '!') {
      cutReason=1;
      stop("# PARADA MANUAL !: DESARMADO, 1000 us.");
      length = 0;
      discard = true;
    } else if (c == '\n' || c == '\r') {
      if (!discard) {
        line[length] = 0;
        command(line);
      }
      length = 0;
      discard = false;
    } else if (!discard) {
      if (length >= sizeof(line)-1 || c < 32 || c > 126) {
        cutReason=512;
        stop("# LINEA INVALIDA");
        discard = true;
        length = 0;
      } else line[length++] = c;
    }
  }
}

void sendTelemetry(uint32_t now, const RawImu &raw) {
  Telemetry t = {};
  t.magic=0x484d5344; t.size=sizeof(t); t.version=2;
  t.sequence=telemetrySequence++; t.ms=millis(); t.us=now;
  t.mode=uint8_t(mode);
  t.flags=(calibrated?1:0)|(calibrating?2:0)|(pwmOK?4:0)|(imuOK?8:0)
    |(driving()?16:0)|(heartbeatFresh()?32:0);
  t.collectiveUs=mode==RAMP ? rampPulse : lroundf(collectiveUs);
  t.modelBase=forceBase;
  for (int i=0;i<3;++i) {
    t.attitude[i]=(i==0?roll:i==1?pitch:yaw)/rad;
    t.rates[i]=rates[i]/rad;
    t.raw[i]=raw.accel[i]; t.raw[i+3]=raw.gyro[i];
  }
  t.scaleRP=scaleRP; t.scaleYaw=scaleYaw;
  for (int i=0;i<8;++i) t.pwm[i]=appliedUs[i];
  for (int i=0;i<2;++i) { t.angleIntegral[i]=angleIntegralRate[i]; t.rateIntegral[i]=integral[i]; }
  t.intervalUs=intervalLastUs; t.readUs=imuReadLastUs; t.pwmWriteUs=pwmWriteUs;
  t.dropped=telemetryDropped; t.adc[0]=adcRaw[0]; t.adc[1]=adcRaw[1]; t.cutReason=cutReason;
  t.accelRaw=accelGuard.raw; t.accelFiltered=accelGuard.filtered;
  t.accelOutsideUs=accelGuard.outsideUs; t.accelTrip=uint8_t(accelGuard.lastTrip);
  t.accelRevision=ensayo::accelPolicyRevision;
  t.crc=ensayo::crc16(reinterpret_cast<const uint8_t *>(&t),sizeof(t)-2);
  if (Serial.availableForWrite() >= int(sizeof(t))) Serial.write(reinterpret_cast<const uint8_t *>(&t),sizeof(t));
  else ++telemetryDropped;
}

void checkAttitudeCutoff() {
  const float tilt = ensayo::tiltDegrees(roll,pitch);
  const bool excessiveTilt = tilt >= cutoffTiltDeg;
  const bool excessiveYaw = fabsf(yaw) >= 20*rad;
  const bool excessiveYawRate = fabsf(rates[2]) >= 35*rad;
  const bool excessiveAccel = accelGuard.update(accelNorm,micros(),true)!=ensayo::AccelTrip::NONE;
  if (!(excessiveTilt || excessiveYaw || excessiveYawRate || excessiveAccel)) return;

  const char *phase = mode == ARMED ? "ARMADO" : mode == RAMP ? "RAMPA" : "CONTROL";
  cutReason=(excessiveTilt?4:0)|(excessiveYaw?8:0)|(excessiveYawRate?16:0)|(excessiveAccel?32:0);
  stop("# CORTE: LIMITE IMU, DESARMADO 1000 us.");
  Serial.printf("# DIAG IMU modo=%s tilt=%d yaw_rel=%d yaw_vel=%d accel=%d\n",
    phase, excessiveTilt, excessiveYaw, excessiveYawRate, excessiveAccel);
  Serial.printf("# MUESTRA tilt_deg=%.3f roll_deg=%.3f pitch_deg=%.3f yaw_deg=%.3f p_dps=%.3f q_dps=%.3f r_dps=%.3f accel_g=%.3f yaw_PI=%d\n",
    tilt, roll/rad, pitch/rad, yaw/rad, rates[0]/rad, rates[1]/rad, rates[2]/rad,
    accelNorm, enableYaw);
  Serial.printf("# DIAG ACCEL rev=%u raw_g=%.4f filtrada_g=%.4f fuera_us=%lu tipo=%u\n",
    unsigned(ensayo::accelPolicyRevision),accelGuard.raw,accelGuard.filtered,
    static_cast<unsigned long>(accelGuard.outsideUs),unsigned(accelGuard.lastTrip));
}

void imuFault(const char *reason, uint32_t intervalUs, uint32_t readUs, bool attempted) {
  const bool announce = calibrated || calibrating || mode != OFF;
  calibrated = false;
  if (!announce) return;
  int minimum = appliedUs[0], maximum = appliedUs[0];
  for (int pulse : appliedUs) {
    minimum = min(minimum, pulse);
    maximum = max(maximum, pulse);
  }
  const uint32_t lastPwmWriteUs = pwmWriteUs;
  cutReason=64;
  stop(reason);
  Serial.printf("# DIAG intervalo_us=%lu lectura_us=%lu i2c_estado=%d bytes=%d pwm_previo_us=%d..%d escritura_pwm_us=%lu\n",
    static_cast<unsigned long>(intervalUs), static_cast<unsigned long>(readUs),
    attempted ? int(i2cStatus) : -1, attempted ? int(i2cBytes) : -1,
    minimum, maximum, static_cast<unsigned long>(lastPwmWriteUs));
  Serial.println("# Repetir cal antes de armar.");
}

void setup() {
  Serial.setTxBufferSize(4096);
  Serial.begin(460800);
  Serial.println("\n# ENSAYO: INICIO DESARMADO");
  for (uint8_t pin : pins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  const esp_err_t pwmInit = initPWM();
  pwmOK = pwmInit == ESP_OK;
  if (!pwmOK) {
    disablePWM();
    Serial.printf("# ERROR INICIO MCPWM: %s\n", esp_err_to_name(pwmInit));
  }
  uniformPWM(stopUs);
  initializeIMU();
  // ToF deliberadamente sin inicializar ni consultar: altura INVALIDA.
  // ADC se guarda crudo; NO se usa como voltaje ni corte de bateria.
  analogReadResolution(12);
  analogSetPinAttenuation(34, ADC_11db);
  analogSetPinAttenuation(35, ADC_11db);
  forceBase = modelForce(baseUs);
  forceMin = modelForce(floorUs);
  forceMax = modelForce(maxUs);
  float mean = 0, projectionX = 0, projectionY = 0;
  for (int i = 0; i < 8; ++i) {
    yawRow[i] = -.020f*spin[i];
    mean += yawRow[i]/8;
    projectionX += yawRow[i]*x[i]/(4*L*L);
    projectionY += yawRow[i]*y[i]/(4*L*L);
  }
  for (int i = 0; i < 8; ++i) {
    yawNull[i] = yawRow[i] - mean - projectionX*x[i] - projectionY*y[i];
    yawAuthority += yawRow[i]*yawNull[i];
  }
  // Activar yaw requiere signos validos y pares balanceados al colectivo base.
  if (enableYaw && (yawAuthority < 1e-6f || fabsf(mean) > 1e-6f)) imuOK = false;
  bootMs = millis();
  lastSample = micros();
  Serial.printf("# DESARMADO. IMU=%d MCPWM=%d yaw_PI=%d. Esperar 4 s y enviar cal.\n",
    imuOK, pwmOK, enableYaw);
  Serial.println("# ENSAYO_HOVER_V1 protocolo=2 bytes=144 baud=460800 max_motor=1600 corte_tilt=15 hb_ms=500 duracion_ms=90000 accel_rev=2");
  Serial.println("# ACCEL: filtro=20_ms persistencia=40_ms fuera_0.55..1.55_g; corte_inmediato_raw<=0.20_o_>=1.90_g");
  Serial.println("# +X=M8 +Y=M7. IMU brazo 8; ToF brazo 1 DESACTIVADO. Altura INVALIDA.");
  Serial.println("# armar + arrancar. rampa/subir/bajar/mantener/hover. ! o apagar: 1000 us y desarmar.");
}

void loop() {
  serialInput();
  if ((mode==ARMED || driving()) && !heartbeatFresh()) {
    cutReason=2; stop("# CORTE: COMUNICACION PC PERDIDA, DESARMADO 1000 us.");
  }
  if (mode==ARMED && millis()-armMs >= 10000) stop("# ARMADO VENCIDO");
  if (driving() && millis()-runMs >= ensayo::maxRunMs) {
    cutReason=256; stop("# CORTE: DURACION MAXIMA 90 s.");
  }
  const uint32_t now=micros(), elapsed=now-lastSample;
  if (imuOK && elapsed>=sampleUs) {
    lastSample=now; intervalLastUs=elapsed;
    float a[3],g[3]; RawImu raw={};
    if (elapsed>30000) {
      imuFault("# CORTE: CICLO TARDIO",elapsed,0,false);
      return;
    }
    const bool readOK=readIMU(a,g,raw);
    const uint32_t readUs=micros()-now;
    imuReadLastUs=readUs;
    if (!readOK || readUs>15000) {
      imuFault(readOK?"# CORTE: LECTURA IMU LENTA":"# CORTE: ERROR I2C MPU6050",elapsed,readUs,true);
      return;
    }
    const bool guardedBefore=driving() || mode==ARMED;
    accelNorm=hypotf(hypotf(a[0],a[1]),a[2]);
    if (calibrating) collectCalibration(a,g);
    else if (calibrated) {
      const float dt=elapsed*1e-6f;
      updateAttitude(a,g,dt);
      if (driving() || mode==ARMED) checkAttitudeCutoff();
      if (mode==CONTROL) collectiveUs=ensayo::slew(collectiveUs,float(targetUs),dt);
      if (mode==PREVIEW || mode==CONTROL || (mode==RAMP && rampPulse>=floorUs)) controlStep(dt);
    }
    // Mantener diagnostico vivo tambien en OFF/VISTA, sin armar ni borrar el ultimo corte.
    if (!guardedBefore) accelGuard.update(accelNorm,micros(),false);
    if (driving() && now-lastOutput>=periodUs) {
      lastOutput=now;
      if (mode==RAMP) {
        rampPulse=min(baseUs,rampPulse+5);
        if (rampPulse<floorUs) {
          if (!uniformPWM(rampPulse)) { cutReason=128; stop("# FALLO PWM"); }
        } else {
          // Recalcular al nuevo colectivo, sin integrar durante el arranque.
          controlStep(0);
          if (!writePWM(commandUs)) { cutReason=128; stop("# FALLO PWM"); }
          else if (rampPulse==baseUs) mode=CONTROL;
        }
      } else if (!writePWM(commandUs)) { cutReason=128; stop("# FALLO PWM"); }
    }
    if (driving()) saveSample(now,raw);
    // Solo un ADC por ciclo, 10 Hz cada canal. No convertir a voltaje sin calibrar.
    if (millis()-lastAdcMs>=50) {
      lastAdcMs=millis(); static unsigned channel=0;
      adcRaw[channel]=analogRead(channel?35:34); channel^=1;
    }
    if (!exportingLog) sendTelemetry(now,raw);
  }
  streamLog();
  delay(1);
}
