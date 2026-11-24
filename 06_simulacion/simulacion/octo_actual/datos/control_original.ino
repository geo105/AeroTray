// Control de actitud para banco con altura inmovilizada. ESP32 clasico, core 3.x.
#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#include <driver/mcpwm_prelude.h>
#include <math.h>
#include <string.h>

#if !defined(CONFIG_IDF_TARGET_ESP32) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Seleccionar ESP32 clasico y Arduino-ESP32 3.x."
#endif

constexpr uint8_t pins[] = {25, 26, 27, 14, 21, 19, 5, 17};
constexpr int stopUs = 1000, spinUs = 1140, floorUs = 1180;
constexpr int baseUs = 1300, maxUs = 1500;
constexpr uint32_t sampleUs = 5000, periodUs = 20000;
constexpr float rad = PI / 180.0f;
// FLU: X frente, Y izquierda, Z arriba. Orden fisico M1..M8.
constexpr float L = .350f, D = .24748737f;
constexpr float x[] = { L, 0, -L, 0, D, -D, -D, D};
constexpr float y[] = { 0, -L, 0, L, -D, -D, D, D};
// Visto desde arriba: M1..M4 horario (-1), M5..M8 antihorario (+1).
constexpr int spin[] = {-1, -1, -1, -1, 1, 1, 1, 1};
constexpr bool enableYaw = false;
constexpr float kpAngle[] = {5.00f, 5.03f};
constexpr float kpRate[] = {.760f, .753f, .378f};
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
float gyroBias[3], correction[3][3], integral[3];
float yawRow[8], yawNull[8], yawAuthority;
float scaleRP = 1, scaleYaw = 1;
float forceBase, forceMin, forceMax;
float sums[6], squares[6];
unsigned calCount = 0;
char line[48];
size_t length = 0;
bool discard = false;

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
  memset(integral, 0, sizeof(integral));
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

bool readIMU(float *a, float *g) {
  uint8_t data[14];
  if (!readRegs(address, 0x3B, data, sizeof(data))) return false;
  for (int i = 0; i < 3; ++i) {
    a[i] = int16_t(uint16_t(data[2*i]) << 8 | data[2*i+1]) / 16384.0f;
    g[i] = int16_t(uint16_t(data[8+2*i]) << 8 | data[9+2*i]) / 131.0f * rad;
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

float allocationScale(const float *forces, const float *delta) {
  float scale = 1;
  for (int i = 0; i < 8; ++i) {
    if (delta[i] > 1e-7f) scale = fminf(scale, (forceMax - forces[i])/delta[i]);
    if (delta[i] < -1e-7f) scale = fminf(scale, (forceMin - forces[i])/delta[i]);
  }
  return clip(scale, 0, 1);
}

void mix(float rollTorque, float pitchTorque, float yawTorque) {
  const float leverNorm = 4*L*L;
  float forces[8], delta[8], yawFromRP = 0;
  for (int i = 0; i < 8; ++i) {
    forces[i] = forceBase;
    delta[i] = (y[i]*rollTorque - x[i]*pitchTorque)/leverNorm;
    yawFromRP += yawRow[i]*delta[i];
  }
  if (enableYaw) {
    for (int i = 0; i < 8; ++i) delta[i] -= yawNull[i]*yawFromRP/yawAuthority;
  }
  scaleRP = allocationScale(forces, delta);
  for (int i = 0; i < 8; ++i) {
    forces[i] += scaleRP*delta[i];
    delta[i] = enableYaw ? yawNull[i]*yawTorque/yawAuthority : 0;
  }
  scaleYaw = allocationScale(forces, delta);
  for (int i = 0; i < 8; ++i) commandUs[i] = pulseForForce(forces[i] + scaleYaw*delta[i]);
}

void controlStep(float dt) {
  const float phiDot = clip(-kpAngle[0]*roll, -25*rad, 25*rad);
  const float thetaDot = clip(-kpAngle[1]*pitch, -25*rad, 25*rad);
  const float qRef = thetaDot/fmaxf(.5f, cosf(roll));
  const float pRef = phiDot - sinf(roll)*tanf(pitch)*qRef;
  const float error[] = {pRef-rates[0], qRef-rates[1], -rates[2]};
  float torque[3];
  for (int i = 0; i < 3; ++i) torque[i] = kpRate[i]*error[i] + kiRate[i]*integral[i];
  mix(torque[0], torque[1], clip(torque[2], -.025f, .025f));
  if (mode != CONTROL) return;
  if (scaleRP > .999f) {
    for (int i = 0; i < 2; ++i) integral[i] = clip(integral[i]+dt*error[i], -.3f, .3f);
  }
  if (enableYaw && scaleYaw > .999f && fabsf(torque[2]) < .025f)
    integral[2] = clip(integral[2]+dt*error[2], -.08f, .08f);
}

void command(const char *s) {
  if (!*s) return;
  if (!strcmp(s, "parar")) { stop("# PARADA"); return; }
  if (!strcmp(s, "cal") && !driving() && mode != ARMED && imuOK && millis()-bootMs >= 4000) {
    stop("# CAL: 5 s inmovil y nivelado fisicamente.");
    calibrated = false;
    calCount = 0;
    memset(sums, 0, sizeof(sums));
    memset(squares, 0, sizeof(squares));
    calibrating = true;
    return;
  }
  if (!strcmp(s, "vista") && calibrated && !driving() && mode != ARMED) {
    mode = PREVIEW;
    Serial.println("# VISTA: PWM calculado en CSV. Salidas a 1000 us.");
    return;
  }
  if (!strcmp(s, "arm BANCO_FIJO") && !driving() && pwmOK && imuOK && calibrated) {
    if (hypotf(roll,pitch) > 4*rad || hypotf(hypotf(rates[0],rates[1]),rates[2]) > 5*rad) {
      stop("# RECHAZADO: nivelar y esperar inmovilidad.");
      return;
    }
    mode = ARMED;
    armMs = millis();
    Serial.println("# ARMADO. Enviar iniciar antes de 10 s.");
    return;
  }
  if (!strcmp(s, "iniciar") && mode == ARMED && millis()-armMs < 10000
      && hypotf(roll,pitch) <= 4*rad && hypotf(hypotf(rates[0],rates[1]),rates[2]) <= 5*rad) {
    mode = RAMP;
    rampPulse = stopUs;
    peakPwmWriteUs = 0;
    yaw = 0;
    lastOutput = micros();
    Serial.println("# RAMPA a 1300 us. Enviar ! o parar para detener.");
    return;
  }
  stop("# ORDEN RECHAZADA. cal | vista | arm BANCO_FIJO | iniciar | parar | !");
}

void serialInput() {
  for (int i = 0; i < 32 && Serial.available(); ++i) {
    const char c = Serial.read();
    if (c == '!') {
      stop("# PARADA");
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
        stop("# LINEA INVALIDA");
        discard = true;
        length = 0;
      } else line[length++] = c;
    }
  }
}

void report() {
  if (millis()-lastReport < 100 || mode == OFF) return;
  lastReport = millis();
  char row[224];
  const char *labels[] = {"OFF", "VISTA", "ARMADO", "RAMPA", "CONTROL"};
  const int *pulses = mode == PREVIEW ? commandUs : appliedUs;
  const int n = snprintf(row, sizeof(row),
    "%lu,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%.3f\n",
    static_cast<unsigned long>(lastReport), labels[mode], roll/rad, pitch/rad, yaw/rad,
    rates[0]/rad, rates[1]/rad, rates[2]/rad, scaleRP, scaleYaw,
    pulses[0], pulses[1], pulses[2], pulses[3],
    pulses[4], pulses[5], pulses[6], pulses[7], accelNorm);
  if (n > 0 && n < int(sizeof(row)) && Serial.availableForWrite() >= n)
    Serial.write(reinterpret_cast<const uint8_t *>(row), n);
}

void checkAttitudeCutoff() {
  const float tilt = hypotf(roll, pitch)/rad;
  const bool excessiveTilt = tilt >= 12;
  const bool excessiveYaw = fabsf(yaw) >= 20*rad;
  const bool excessiveYawRate = fabsf(rates[2]) >= 35*rad;
  const bool excessiveAccel = accelNorm < .55f || accelNorm > 1.55f;
  if (!(excessiveTilt || excessiveYaw || excessiveYawRate || excessiveAccel)) return;

  const char *phase = mode == RAMP ? "RAMPA" : "CONTROL";
  stop("# CORTE: LIMITE IMU");
  Serial.printf("# DIAG IMU modo=%s tilt=%d yaw_rel=%d yaw_vel=%d accel=%d\n",
    phase, excessiveTilt, excessiveYaw, excessiveYawRate, excessiveAccel);
  Serial.printf("# MUESTRA tilt_deg=%.3f roll_deg=%.3f pitch_deg=%.3f yaw_deg=%.3f p_dps=%.3f q_dps=%.3f r_dps=%.3f accel_g=%.3f yaw_PI=%d\n",
    tilt, roll/rad, pitch/rad, yaw/rad, rates[0]/rad, rates[1]/rad, rates[2]/rad,
    accelNorm, enableYaw);
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
  stop(reason);
  Serial.printf("# DIAG intervalo_us=%lu lectura_us=%lu i2c_estado=%d bytes=%d pwm_previo_us=%d..%d escritura_pwm_us=%lu\n",
    static_cast<unsigned long>(intervalUs), static_cast<unsigned long>(readUs),
    attempted ? int(i2cStatus) : -1, attempted ? int(i2cBytes) : -1,
    minimum, maximum, static_cast<unsigned long>(lastPwmWriteUs));
  Serial.println("# Repetir cal antes de armar.");
}

void setup() {
  Serial.setTxBufferSize(512);
  Serial.begin(115200);
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
  Wire.begin(23, 22, 100000);
  Wire.setTimeOut(8);
  for (uint8_t dev = 0x68; dev <= 0x69; ++dev) {
    uint8_t id;
    if (readRegs(dev, 0x75, &id, 1) && id == 0x68) { address = dev; break; }
  }
  if (address) {
    imuOK = setReg(0x6B, 0x01);
    delay(100);
    imuOK = imuOK && setReg(0x6C, 0) && setReg(0x1A, 3) && setReg(0x19, 4)
      && setReg(0x1B, 0) && setReg(0x1C, 0);
  }
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
  Serial.println("ms,modo,roll_deg,pitch_deg,yaw_rel_deg,p_dps,q_dps,r_dps,escala_RP,escala_yaw,M1_us,M2_us,M3_us,M4_us,M5_us,M6_us,M7_us,M8_us,accel_g");
}

void loop() {
  serialInput();
  if (mode == ARMED && millis()-armMs >= 10000) stop("# ARMADO VENCIDO");
  const uint32_t now = micros();
  const uint32_t elapsed = now-lastSample;
  if (imuOK && elapsed >= sampleUs) {
    lastSample = now;
    float a[3], g[3];
    if (elapsed > 30000) {
      imuFault("# CORTE: CICLO TARDIO", elapsed, 0, false);
      return;
    }
    const bool readOK = readIMU(a, g);
    const uint32_t readUs = micros()-now;
    if (!readOK || readUs > 15000) {
      imuFault(readOK ? "# CORTE: LECTURA IMU LENTA" : "# CORTE: ERROR I2C MPU6050",
        elapsed, readUs, true);
      return;
    }
    if (calibrating) collectCalibration(a, g);
    else if (calibrated) {
      const float dt = elapsed*1e-6f;
      updateAttitude(a, g, dt);
      if (driving()) checkAttitudeCutoff();
      if (mode == PREVIEW || mode == CONTROL) controlStep(dt);
    }
  }
  if (driving() && now-lastOutput >= periodUs) {
    lastOutput = now;
    if (mode == RAMP) {
      rampPulse = min(baseUs, rampPulse+5);
      if (!uniformPWM(rampPulse)) stop("# FALLO PWM");
      else if (rampPulse == baseUs) mode = CONTROL;
    } else if (!writePWM(commandUs)) stop("# FALLO PWM");
  }
  report();
  delay(1);
}
