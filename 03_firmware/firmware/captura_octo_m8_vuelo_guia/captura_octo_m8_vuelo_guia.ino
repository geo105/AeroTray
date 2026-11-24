// VUELO EN GUIA (octo M8). ESP32 clasico, core 3.x.
// Base: captura_octo_m8_banco_angulo_laser (el firmware que funciona en el robot): mismo estimador, mismas
// ganancias del control de angulo, mismo mezclador por fuerzas y mismos cortes (inclinacion 12 deg, yaw 20 deg,
// giro de yaw 35 deg/s, IMU). Cambios para despegar sobre la guia:
//  - colectivo variable 1300..1950 us (antes fijo en 1300) que cambia con rampa: sube 25 us/s y baja 60 us/s;
//  - ordenes nuevas (solo en CONTROL): flotar (1750), maximo (1950), subir / bajar (+-25 us), mantener,
//    aterrizar (1300);
//  - techo por motor 2000 us (tope del ESC): con el colectivo en 1950 la correccion de angulo usa los 50 us que
//    quedan y, si no alcanza, el mezclador baja el lado contrario (prioriza el angulo sobre el empuje total);
//  - maximo 20 s seguidos sobre 1850 us: despues baja solo a 1800 (corriente cerca de los fusibles de 30 A);
//  - vigilancia del enlace: el PC manda "hb" cada 0,3 s; sin hb no deja pasar de 1400 us y, si el hb se corta
//    1,5 s con el colectivo alto, baja solo a 1300 (no corta en el aire);
//  - limite de 90 s por encima de 1500 us: despues baja solo a 1300;
//  - log de 16 s a 100 Hz con colectivo y laser.
// "parar" y "!" siguen cortando al instante; cualquier orden desconocida sigue parando los motores.
#include <Arduino.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#include <driver/mcpwm_prelude.h>
#include <math.h>
#include <string.h>
#include <DSMLaser.h>
// VARIANTE CON LASER de captura_octo_m8_banco_angulo: mismo control, ganancias y cortes. Solo agrega:
//  - lectura del VL53L0X (0x29, mismo bus I2C 23/22), solo para informar: no entra en el control ni en cortes;
//  - dos columnas al final de la telemetria: laser_mm (ultima medida, valida o no) y laser_estado:
//    0 = medida valida y reciente, 1..5 = RangeStatus del sensor (2/4: senal debil o fuera de rango),
//    254 = sin medida valida reciente, 255 = laser no iniciado o desactivado;
//  - telemetria tambien en OFF (10 por segundo) para revisar los sensores sin armar; se pausa al exportar el log;
//  - escaneo del bus I2C al arrancar.
// Si el laser da 20 errores de bus seguidos se deja de consultar para no retrasar la IMU.

#if !defined(CONFIG_IDF_TARGET_ESP32) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Seleccionar ESP32 clasico y Arduino-ESP32 3.x."
#endif

constexpr uint8_t pins[] = {25, 26, 27, 14, 21, 19, 5, 17};
constexpr int stopUs = 1000, spinUs = 1140, floorUs = 1180;
constexpr int baseUs = 1300, maxUs = 2000;              // 2000 us = tope del ESC
constexpr int colectivoTechoUs = 1950, flotarUs = 1750, pasoUs = 25, colectivoSinHbUs = 1400;
constexpr int potenciaAltaUs = 1850;                    // sobre esto cuenta el tiempo a maxima potencia
constexpr float subidaUsPorS = 25.0f, bajadaUsPorS = 60.0f;
// v9: cerca del despegue (sobre 1650 us) sube despacio, 8 us/s: de 1650 a 1750 tarda 12,5 s.
constexpr float subidaLentaUsPorS = 8.0f, subidaLentaDesdeUs = 1650.0f;
constexpr uint32_t hbTimeoutMs = 1500, maxAltoMs = 90000, maxPotenciaAltaMs = 20000;
constexpr uint32_t sampleUs = 5000, periodUs = 20000;
constexpr float rad = PI / 180.0f;
constexpr float cutoffTiltDeg = 12.0f;
// Variante experimental de banco. Excesos de norma se registran, no cortan.
// El angulo del interlock es ESTIMADO, no una medicion independiente.
// FLU: +X hacia M8, +Y hacia M7, +Z arriba. Orden fisico M1..M8.
constexpr float L = .350f, D = .24748737f;
constexpr float x[] = { D, -D, -D, D, 0, -L, 0, L};
constexpr float y[] = {-D, -D,  D, D,-L,  0, L, 0};
// Visto desde arriba: M1..M4 horario (-1), M5..M8 antihorario (+1).
constexpr int spin[] = {-1, -1, -1, -1, 1, 1, 1, 1};
constexpr bool enableYaw = true;   // v4: el dron giraba sobre la rotula a mas de 1450 us
constexpr float yawMaxTorque = .06f, kYawAngle = .8f;   // v6: vuelta al rumbo mas lenta (antes 1.5)
constexpr float yawKpVuelo = .20f;          // v6: ganancia de yaw (antes .378)
constexpr float yawSlewNmPorS = .10f;       // v6: el par de yaw cambia como maximo 0,1 N.m/s
constexpr int colectivoVueloUs = 1650;      // v6: sobre esto ya flota: ganancias de roll/pitch al 75 %
constexpr float escalaVuelo = 1.0f;   // v7: al 75 % no alcanzo a frenar el cabeceo al despegar  // par maximo de yaw (N.m) y vuelta al rumbo inicial (1/s)
constexpr float kpAngle[] = {4.62f, 4.65f};
constexpr float kdAngle[] = {.12f, .12f};
constexpr float kiAngle = .8f;
// v8: con el pedido de giro recortado a 25 dps y la integral de giro a 0,3, el par maximo era ~0,37 N.m,
// menos que el desbalance real (~0,42 N.m): se inclinaba despacio sin poder volver. Ya flotando
// (colectivo > colectivoVueloUs) se abre el margen; en el apoyo quedan los limites del v7.
constexpr float rateRequestLimit = 25*rad, rateRequestLimitVuelo = 60*rad;
constexpr float rateIntegralLimit = .3f, rateIntegralLimitVuelo = 3.0f;
constexpr float angleIntegralRateLimitVuelo = 60*rad, kiAngleVuelo = 2.0f;
constexpr float angleIntegralRateLimit = 30*rad;   // v5: antes 10; compensa un desbalance fijo sin quedarse inclinado
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
uint32_t accelEvents = 0, accelCurrentUs = 0, accelLongestUs = 0, accelTotalUs = 0;
float accelMinG = INFINITY, accelMaxG = 0;
bool accelOutside = false;
float gyroBias[3], correction[3][3], integral[3];
float filteredAngleRate[2];
float angleIntegralRate[2];
float yawRow[8], yawNull[8], yawAuthority;
float scaleRP = 1, scaleYaw = 1;
float forceBase, forceMin, forceMax;
float sums[6], squares[6];
unsigned calCount = 0;
bool laserOK = false;
uint16_t laserMm = 0;
uint32_t laserReadPeakUs = 0, laserErrorBase = 0;
uint8_t laserStreak = 0;
char line[48];
size_t length = 0;
bool discard = false;

// Historial circular de muestras validas, sin emitir CSV rapido durante el control.
struct RawImu { int16_t accel[3], gyro[3]; };
struct LogSample {
  uint32_t tUs;
  RawImu imu;
  float attitude[3], scale;
  uint16_t pwm[8];
  uint16_t colectivo, laser;
  uint8_t phase;
};
constexpr size_t logCapacity = 1600;
LogSample flightLog[logCapacity];
size_t logNext = 0, logCount = 0, logSent = 0, logByteOffset = 0;
uint32_t runStartUs = 0, markUs = 0;
bool marked = false, exportingLog = false;
float colectivoActual = baseUs;
// v10 "modo base": con el colectivo en ralenti (<= 1400 us) el dron va apoyado sobre la base. El yaw lo impone la
// base al girar (la rotula lo arrastra), asi que no se controla el yaw ni se corta por yaw (20 deg / 35 deg/s):
// antes cualquier giro de 360 grados de la base cortaba los motores. Roll y pitch y el corte de 12 deg siguen igual.
// Al subir el colectivo por encima de 1400 us el rumbo de referencia se reinicia y todo vuelve a ser como en v9.
inline bool modoBase() { return colectivoActual <= colectivoSinHbUs + 0.5f; }
// v10b: giro de la base. La myRIO calcula la velocidad de giro de la base (odometria) y la Pi la reenvia como
// "yb<grados/s>" (~10 Hz). Con ese dato el dron CONTROLA su yaw relativo a la base: sigue el giro de la base
// en vez de pelear contra la rotula, y el corte de 20 deg mide el giro real dron-base. Sin el dato (mas de
// 0,5 s), en ralenti (<= 1400 us) el yaw lo arrastra la rotula sin control (como v10) y por encima es v9.
float baseYawRate = 0;            // rad/s, +antihorario visto desde arriba (+Z arriba)
uint32_t baseYawMs = 0;
bool baseYawVisto = false;
inline bool baseYawFresca() { return baseYawVisto && millis() - baseYawMs < 500; }
inline bool yawPasivo() { return false; }   // v11: el dron siempre controla su rumbo (ya no hay yaw libre)
// v11: en ralenti sobre la base, sin datos de la base, el dron MANTIENE SU RUMBO INICIAL, pero con una ventana
// de +-20 deg: si la base lo arrastra mas (un giro de la base), la referencia se desliza con el en vez de cortar.
// Asi no pelea contra la base en un giro de 360 deg y, al parar la base, se queda quieto (desfase <= 20 deg).
inline bool rumboConVentana() { return !baseYawFresca() && modoBase(); }
int colectivoObjetivo = baseUs;
// v11: MANTENER ALTURA. Al pulsar "mantener", si el laser da medidas validas, el dron guarda la distancia al suelo
// de ese instante y corrige el colectivo (+-60 us alrededor del colectivo de ese instante) para quedarse a esa
// altura. Si el laser deja de dar medidas validas, la correccion se congela (= el "mantener" de siempre) y a los
// 2 s se apaga el modo. Cualquier otra orden de colectivo lo apaga. Ancho de banda ~3 rad/s (el maximo razonable con
// el laser a 20 Hz); cerca del vuelo 1 us de colectivo da ~0,026 m/s2. Ganancias elegidas por simulacion (laser a
// 20 Hz con ruido de 5 mm, motor de 70 ms, friccion de la guia 0-1 m/s2): desviacion 3-11 mm y vuelve a la referencia.
// Se calcula SOLO cuando llega una medida nueva (el laser mide cada 50 ms y el colectivo se actualiza cada 20 ms).
bool alturaActiva = false;
float alturaRefMm = 0, alturaColBase = 0, alturaInt = 0, alturaFiltMm = 0, alturaVel = 0, alturaCorr = 0;
uint32_t alturaUltimaMs = 0, alturaSecuencia = 0;
// Calidad del laser: ultimas 8 medidas validas (hora y mm). Solo se usa para la altura si hay >= 6 en 1,5 s y
// su dispersion es <= 30 mm. Con el laser actual (senal debil, dispersion ~70 mm) NO se activa: queda el mantener normal.
uint32_t calT[8] = {0}; uint16_t calMm[8] = {0}; uint8_t calI = 0; uint32_t calSec = 0;
// ---- Filtro de Kalman de altura: laser + acelerometro vertical ----
// Estado: z (m, distancia del laser al suelo), v (m/s, + subiendo), b (m/s2, sesgo del acelerometro vertical).
// Prediccion a la tasa de la IMU con la aceleracion vertical (acelerometro filtrado a 10 Hz, girado al mundo con
// roll/pitch); correccion con cada medida aceptada del laser (R = 52 mm, la dispersion medida el 23/11), con puerta
// de 3 sigma. Si rechaza 5 medidas seguidas, se resincroniza con el laser. Parametros elegidos por simulacion
// (pruebas_20261123/kalman/sim_kalman_altura.py): error ~33-52 mm frente a 73 mm del laser crudo.
float kfX[3] = {0, 0, 0}, kfP[3][3] = {{0}};
bool kfInit = false; uint8_t kfRechazos = 0; uint32_t kfUltimaMs = 0;
float accVertical = 0;                      // m/s2, aceleracion vertical del dron (sin gravedad)
// Calibracion del laser con regla (23/11/2026, hoja blanca sobre la base): real 330 mm -> lectura 394 mm,
// real 450 mm -> lectura 523 mm. Distancia real = 0,930 * lectura - 36,5 mm. Vale para este montaje.
constexpr float laserGanancia = 0.930f, laserOffsetMm = -36.5f;
inline float laserCorregidoMm(float lectura) { return laserGanancia * lectura + laserOffsetMm; }
// El laser esta en el brazo 8 (+x), a 0,20 m del eje de la rotula (medido 23/11). Al cabecear, ese extremo sube o
// baja r*sin(pitch) aunque la rotula no se mueva en la guia. En este firmware pitch > 0 es la nariz (M8) hacia ABAJO
// (giro positivo alrededor de +y en FLU), asi que la altura del eje es d*cos(roll)*cos(pitch) + r*sin(pitch).
constexpr float laserBrazoM = 0.20f;
inline float alturaEjeM(float distanciaM) { return distanciaM * cosf(roll) * cosf(pitch) + laserBrazoM * sinf(pitch); }
constexpr float kfQa = 0.6f, kfQb = 0.02f, kfR = 0.052f * 0.052f;
void kfReiniciar(float z) {
  kfX[0] = z; kfX[1] = 0; kfX[2] = 0;
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) kfP[i][j] = 0;
  kfP[0][0] = kfR; kfP[1][1] = 0.1f * 0.1f; kfP[2][2] = 0.5f * 0.5f;
  kfInit = true; kfRechazos = 0;
}
void kfPredecir(float dt, float a) {
  if (!kfInit) return;
  const float u = a - kfX[2];
  kfX[0] += kfX[1] * dt + 0.5f * u * dt * dt;
  kfX[1] += u * dt;
  // P = F P F' + G G' qa^2 + diag(0,0,qb^2 dt),  F = [[1,dt,-dt2/2],[0,1,-dt],[0,0,1]]
  const float h = 0.5f * dt * dt, F[3][3] = {{1, dt, -h}, {0, 1, -dt}, {0, 0, 1}}, G[3] = {h, dt, 0};
  float FP[3][3], N[3][3];
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) { FP[i][j] = 0; for (int k = 0; k < 3; ++k) FP[i][j] += F[i][k] * kfP[k][j]; }
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) { N[i][j] = G[i] * G[j] * kfQa * kfQa; for (int k = 0; k < 3; ++k) N[i][j] += FP[i][k] * F[j][k]; }
  N[2][2] += kfQb * kfQb * dt;
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) kfP[i][j] = N[i][j];
}
void kfCorregir(float z) {
  if (!kfInit) { kfReiniciar(z); return; }
  const float S = kfP[0][0] + kfR, inn = z - kfX[0];
  if (inn * inn > 9 * S) { if (++kfRechazos >= 5) kfReiniciar(z); return; }   // puerta de 3 sigma
  kfRechazos = 0;
  float K[3]; for (int i = 0; i < 3; ++i) K[i] = kfP[i][0] / S;
  for (int i = 0; i < 3; ++i) kfX[i] += K[i] * inn;
  float P0[3]; for (int j = 0; j < 3; ++j) P0[j] = kfP[0][j];
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) kfP[i][j] -= K[i] * P0[j];
  kfUltimaMs = millis();
}
// altura confiable: Kalman inicializado, corregido hace < 0,5 s y con incertidumbre < 30 mm
bool kfConfiable() { return kfInit && millis() - kfUltimaMs < 500 && kfP[0][0] < 0.030f * 0.030f; }
bool laserEstable() {
  const uint32_t ahora = millis(); int n = 0; uint16_t mn = 65535, mx = 0;
  for (int i = 0; i < 8; ++i) if (calT[i] && ahora - calT[i] <= 1500) { ++n; mn = min(mn, calMm[i]); mx = max(mx, calMm[i]); }
  return n >= 6 && mx - mn <= 30;
}
constexpr float kpAltura = 0.25f, kdAltura = 0.14f, kiAltura = 0.12f, corrMaxAltura = 60.0f;   // (PID anterior, ya no se usa)
// v11c: CONTROL DE ALTURA EN CASCADA (sin oscilar con la friccion de la guia). Lazo externo: error de altura ->
// consigna de velocidad (1,5 1/s, maximo 40 mm/s). Lazo interno: PI de velocidad con la velocidad del Kalman
// (acelerometro + laser) -> correccion del colectivo (+-60 us). La consigna se mueve en rampa a 20 mm/s.
// Simulacion (pruebas_20261123/kalman/sim_altura_cascada.py): 4-12 mm pico a pico con friccion, frente a 13-46 mm del PID.
constexpr float kPosAltura = 1.5f, vMaxAltura = 40.0f, kvPAltura = 0.18f, kvIAltura = 0.35f, rampaRefAltura = 20.0f;
float alturaObjMm = 0, alturaEngancheMm = 0;
uint32_t ultimoHbMs = 0, altoDesdeMs = 0, potenciaAltaDesdeMs = 0, ultimoAvisoColMs = 0;
bool hbVisto = false;
uint8_t muestraPar = 0;

void saveSample(uint32_t now, const RawImu &imu) {
  LogSample &sample = flightLog[logNext];
  sample.tUs = now-runStartUs;
  sample.imu = imu;
  sample.attitude[0] = roll;
  sample.attitude[1] = pitch;
  sample.attitude[2] = yaw;
  sample.scale = scaleRP;
  for (int i = 0; i < 8; ++i) sample.pwm[i] = appliedUs[i];
  sample.colectivo = static_cast<uint16_t>(lroundf(colectivoActual));
  sample.laser = laserMm;
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
  Serial.println("t_us,fase,ax_g,ay_g,az_g,gx_sensor_dps,gy_sensor_dps,gz_sensor_dps,roll_deg,pitch_deg,yaw_rel_deg,escala_RP,M1_us,M2_us,M3_us,M4_us,M5_us,M6_us,M7_us,M8_us,colectivo_us,laser_mm");
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
    "%lu,%s,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
    static_cast<unsigned long>(s.tUs), s.phase == RAMP ? "RAMPA" : "CONTROL",
    s.imu.accel[0]/16384.0f, s.imu.accel[1]/16384.0f, s.imu.accel[2]/16384.0f,
    s.imu.gyro[0]/131.0f, s.imu.gyro[1]/131.0f, s.imu.gyro[2]/131.0f,
    s.attitude[0]/rad, s.attitude[1]/rad, s.attitude[2]/rad, s.scale,
    unsigned(s.pwm[0]), unsigned(s.pwm[1]), unsigned(s.pwm[2]), unsigned(s.pwm[3]),
    unsigned(s.pwm[4]), unsigned(s.pwm[5]), unsigned(s.pwm[6]), unsigned(s.pwm[7]),
    unsigned(s.colectivo), unsigned(s.laser));
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

float modelForce(float pulse) {
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
  alturaActiva = false;   // v11
  mode = OFF;
  calibrating = false;
  rampPulse = stopUs;
  colectivoActual = baseUs;
  colectivoObjetivo = baseUs;
  altoDesdeMs = potenciaAltaDesdeMs = 0;
  forceBase = modelForce(baseUs);
  memset(integral, 0, sizeof(integral));
  memset(angleIntegralRate, 0, sizeof(angleIntegralRate));
  memset(filteredAngleRate, 0, sizeof(filteredAngleRate));
  uniformPWM(stopUs);
  Serial.println(reason);
  if (wasDriving) Serial.printf("# PWM escritura_max_us=%lu\n", static_cast<unsigned long>(peakPwmWriteUs));
  if (wasDriving) Serial.printf("# LASER activo=%d lectura_max_us=%lu validas=%lu invalidas=%lu errores_i2c=%lu\n",
    laserOK, static_cast<unsigned long>(laserReadPeakUs), static_cast<unsigned long>(vl53::validas),
    static_cast<unsigned long>(vl53::invalidas), static_cast<unsigned long>(vl53::erroresI2c));
  if (wasDriving) Serial.printf(
    "# ACCEL OBSERVADA sin_corte=1 eventos=%lu exceso_total_ms=%.1f exceso_max_ms=%.1f min_g=%.3f max_g=%.3f\n",
    static_cast<unsigned long>(accelEvents), accelTotalUs/1000.0f,
    accelLongestUs/1000.0f, accelMinG, accelMaxG);
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
  accelNorm = hypotf(hypotf(a[0], a[1]), a[2]);   // norma cruda: solo registro (ACCEL OBSERVADA)
  // v3: el acelerometro pasa por un filtro de 10 Hz antes de corregir el angulo; la vibracion de los
  // motores (0,1..2 g a mas de 1425 us) ensuciaba el roll estimado.
  static float af[3] = {0, 0, 1};
  const float alphaA = dt / (.0159f + dt);
  for (int i = 0; i < 3; ++i) af[i] += alphaA * (a[i] - af[i]);
  const float normaFiltrada = hypotf(hypotf(af[0], af[1]), af[2]);
  const float s = sinf(roll), c = cosf(roll);
  const float side = s*rates[1] + c*rates[2];
  const float cosPitch = fmaxf(.25f, cosf(pitch));
  roll += dt * (rates[0] + sinf(pitch)/cosPitch * side);
  pitch += dt * (c*rates[1] - s*rates[2]);
  yaw += dt * side / cosPitch; // giro relativo desde iniciar, solo para corte y registro
  if (fabsf(normaFiltrada - referenceNorm) < .15f) {
    // con el colectivo alto se confia menos en el acelerometro: constante de 1 s en vez de 0,25 s
    const float tau = colectivoActual > 1400 ? 1.0f : .25f;
    const float weight = dt / (tau + dt);
    roll += weight * (atan2f(af[1], af[2]) - roll);
    pitch += weight * (atan2f(-af[0], hypotf(af[1], af[2])) - pitch);
  }
  // aceleracion vertical en el mundo (g -> m/s2), sin la gravedad (referenceNorm = 1 g calibrado)
  const float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
  accVertical = ((-sp * af[0] + sr * cp * af[1] + cr * cp * af[2]) - referenceNorm) * 9.81f;
  kfPredecir(dt, accVertical);
}

float allocationScale(const float *forces, const float *delta) {
  float scale = 1;
  for (int i = 0; i < 8; ++i) {
    if (delta[i] > 1e-7f) scale = fminf(scale, (forceMax - forces[i])/delta[i]);
    if (delta[i] < -1e-7f) scale = fminf(scale, (forceMin - forces[i])/delta[i]);
  }
  return clip(scale, 0, 1);
}

// Each group of four keeps its original total thrust. Opposite motor pairs
// contribute torque through their force difference and collective through their sum.
float groupShare(float firstDifference, float secondDifference) {
  const float span = forceMax - forceMin;
  const float groupMargin = 4*fminf(forceBase-forceMin, forceMax-forceBase);
  const float largest = fmaxf(fabsf(firstDifference), fabsf(secondDifference));
  if (largest < 1e-7f) return 1;
  return fminf(1, fminf(span/largest,
    groupMargin/(fabsf(firstDifference)+fabsf(secondDifference))));
}

void setGroupForces(float *forces, int low1, int high1, int low2, int high2,
                    float firstDifference, float secondDifference) {
  const float total = 4*forceBase;
  const float firstAbs = fabsf(firstDifference);
  const float secondAbs = fabsf(secondDifference);
  const float firstSum = clip(2*forceBase,
    fmaxf(2*forceMin+firstAbs, total-2*forceMax+secondAbs),
    fminf(2*forceMax-firstAbs, total-2*forceMin-secondAbs));
  const float secondSum = total-firstSum;
  forces[low1] = .5f*(firstSum-firstDifference);
  forces[high1] = .5f*(firstSum+firstDifference);
  forces[low2] = .5f*(secondSum-secondDifference);
  forces[high2] = .5f*(secondSum+secondDifference);
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
  if (!enableYaw && scaleRP < 1) {
    const float cardinalRoll = rollTorque/L;
    const float cardinalPitch = pitchTorque/L;
    const float diagonalPlus = (rollTorque+pitchTorque)/(2*D);
    const float diagonalMinus = (rollTorque-pitchTorque)/(2*D);
    const float cardinalMax = groupShare(cardinalRoll, cardinalPitch);
    const float diagonalMax = groupShare(diagonalPlus, diagonalMinus);
    const float combined = fminf(1, cardinalMax+diagonalMax);
    if (combined > scaleRP) {
      const float cardinal = clip(.5f*combined, fmaxf(0, combined-diagonalMax), cardinalMax);
      const float diagonal = combined-cardinal;
      setGroupForces(forces, 4, 6, 7, 5, cardinal*cardinalRoll, cardinal*cardinalPitch);
      setGroupForces(forces, 0, 2, 1, 3, diagonal*diagonalPlus, diagonal*diagonalMinus);
      scaleRP = combined;
      scaleYaw = 1;
      for (int i = 0; i < 8; ++i) commandUs[i] = pulseForForce(forces[i]);
      return;
    }
  }
  for (int i = 0; i < 8; ++i) {
    forces[i] += scaleRP*delta[i];
    delta[i] = enableYaw ? yawNull[i]*yawTorque/yawAuthority : 0;
  }
  scaleYaw = allocationScale(forces, delta);
  for (int i = 0; i < 8; ++i) commandUs[i] = pulseForForce(forces[i] + scaleYaw*delta[i]);
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
  // v9: al aterrizar (en 1650 us) volvia a los limites del apoyo estando aun en el aire y el pitch se iba a 12 grados.
  // Con histeresis: entra sobre 1650 us y no sale hasta bajar de 1450 us (ya apoyado) o salir de CONTROL.
  static bool enVuelo = false;
  if (mode != CONTROL || colectivoActual < 1450) enVuelo = false;
  else if (colectivoActual > colectivoVueloUs) enVuelo = true;
  const float limPedido = enVuelo ? rateRequestLimitVuelo : rateRequestLimit;
  const float phiDot = clip(rollRateRequest, -limPedido, limPedido);
  const float thetaDot = clip(pitchRateRequest, -limPedido, limPedido);
  const float qRef = thetaDot/fmaxf(.5f, cosRoll);
  const float pRef = phiDot - sinRoll*tanPitch*qRef;
  if (rumboConVentana()) yaw = clip(yaw, -20*rad, 20*rad);       // v11: la referencia se desliza mas alla de 20 deg
  const float giroBase = baseYawFresca() ? baseYawRate : 0.0f;
  if (baseYawFresca()) yaw -= dt*giroBase;                       // v10b: yaw relativo a la base
  const float rRef = giroBase + clip(-kYawAngle*yaw, -15*rad, 15*rad);   // sigue a la base y corrige el desfase
  const float error[] = {pRef-rates[0], qRef-rates[1], rRef-rates[2]};
  float torque[3];
  const float escala = colectivoActual > colectivoVueloUs ? escalaVuelo : 1.0f;
  for (int i = 0; i < 2; ++i) torque[i] = escala*(kpRate[i]*error[i] + kiRate[i]*integral[i]);
  torque[2] = yawPasivo() ? 0.0f : yawKpVuelo*error[2] + kiRate[2]*integral[2];
  // el par de yaw no puede cambiar de golpe (al despegar invertia el reparto en < 1 s)
  static float yawPrevio = 0;
  const float yawPedido = clip(torque[2], -yawMaxTorque, yawMaxTorque);
  yawPrevio = mode == CONTROL ? clip(yawPedido, yawPrevio - yawSlewNmPorS*dt, yawPrevio + yawSlewNmPorS*dt) : 0;
  mix(torque[0], torque[1], yawPrevio);
  if (mode != CONTROL) return;
  if (scaleRP > .999f) {
    const float limA = enVuelo ? angleIntegralRateLimitVuelo : angleIntegralRateLimit;
    const float kiA = enVuelo ? kiAngleVuelo : kiAngle;
    const float limI = enVuelo ? rateIntegralLimitVuelo : rateIntegralLimit;
    if (fabsf(rollRateRequest) < limPedido && fabsf(pitchRateRequest) < limPedido) {
      angleIntegralRate[0] = clip(angleIntegralRate[0]+dt*kiA*rollError, -limA, limA);
      angleIntegralRate[1] = clip(angleIntegralRate[1]+dt*kiA*pitchError, -limA, limA);
    }
    for (int i = 0; i < 2; ++i) integral[i] = clip(integral[i]+dt*error[i], -limI, limI);
  }
  if (enableYaw && !yawPasivo() && scaleYaw > .999f && fabsf(torque[2]) < yawMaxTorque)
    integral[2] = clip(integral[2]+dt*error[2], -.08f, .08f);
}

void avisarColectivo() {
  ultimoAvisoColMs = millis();
  Serial.printf("# COLECTIVO objetivo=%d actual=%d us\n", colectivoObjetivo, int(lroundf(colectivoActual)));
}

bool hbReciente() { return hbVisto && millis()-ultimoHbMs < hbTimeoutMs; }

void fijarObjetivo(int objetivo) {
  objetivo = constrain(objetivo, baseUs, colectivoTechoUs);
  if (objetivo > colectivoSinHbUs && !hbReciente()) {
    Serial.println("# VUELO RECHAZADO: sin enlace con el PC (hb) no se pasa de 1400 us.");
    objetivo = min(objetivo, colectivoSinHbUs);
  }
  colectivoObjetivo = objetivo;
  avisarColectivo();
}

// Rampa del colectivo y vigilancias, una vez por periodo PWM (20 ms) en CONTROL.
void actualizarColectivo(float dt) {
  const uint32_t ahora = millis();
  if (colectivoActual > colectivoSinHbUs && !hbReciente() && colectivoObjetivo > baseUs) {
    colectivoObjetivo = baseUs;
    Serial.println("# ENLACE PERDIDO: sin hb del PC, bajando a 1300 us.");
  }
  if (colectivoActual > 1500) {
    if (!altoDesdeMs) altoDesdeMs = ahora;
    if (ahora-altoDesdeMs > maxAltoMs && colectivoObjetivo > baseUs) {
      colectivoObjetivo = baseUs;
      Serial.println("# TIEMPO MAXIMO EN ALTO (90 s): bajando a 1300 us.");
    }
  } else altoDesdeMs = 0;
  if (colectivoActual > potenciaAltaUs) {
    if (!potenciaAltaDesdeMs) potenciaAltaDesdeMs = ahora;
    if (ahora-potenciaAltaDesdeMs > maxPotenciaAltaMs && colectivoObjetivo > flotarUs) {
      colectivoObjetivo = flotarUs;
      Serial.println("# 20 s A MAXIMA POTENCIA: bajando a 1800 us (corriente y fusibles).");
      avisarColectivo();
    }
  } else potenciaAltaDesdeMs = 0;
  if (alturaActiva) {
    if (kfConfiable()) {                                                  // altura y velocidad del Kalman
      const float dl = alturaUltimaMs ? constrain((ahora - alturaUltimaMs) / 1000.0f, 0.005f, 0.1f) : 0.02f;
      alturaFiltMm = 1000.0f * kfX[0];
      alturaVel = 1000.0f * kfX[1];                                                     // mm/s, + subiendo
      // la referencia sigue al objetivo en rampa (sin escalones)
      alturaRefMm += clip(alturaObjMm - alturaRefMm, -rampaRefAltura * dl, rampaRefAltura * dl);
      const float vRef = clip(kPosAltura * (alturaRefMm - alturaFiltMm), -vMaxAltura, vMaxAltura);   // lazo externo
      const float ev = vRef - alturaVel;                                                              // lazo interno
      alturaInt = clip(alturaInt + ev * dl, -corrMaxAltura / kvIAltura, corrMaxAltura / kvIAltura);   // anti-windup
      alturaCorr = clip(kvPAltura * ev + kvIAltura * alturaInt, -corrMaxAltura, corrMaxAltura);
      alturaUltimaMs = ahora;
    } else if (alturaUltimaMs && ahora - alturaUltimaMs > 2000) {
      alturaActiva = false;
      Serial.println("# ALTURA: el laser dejo de medir 2 s; queda el colectivo fijo (mantener normal).");
    }
    colectivoObjetivo = int(lroundf(constrain(alturaColBase + alturaCorr, (float)baseUs, (float)colectivoTechoUs)));
    colectivoActual = colectivoObjetivo;   // la correccion ya es suave; no pasa por la rampa
  }
  const float antes = colectivoActual;
  if (colectivoActual < colectivoObjetivo) colectivoActual = fminf(colectivoObjetivo, colectivoActual + (colectivoActual >= subidaLentaDesdeUs ? subidaLentaUsPorS : subidaUsPorS)*dt);
  else if (colectivoActual > colectivoObjetivo) colectivoActual = fmaxf(colectivoObjetivo, colectivoActual - bajadaUsPorS*dt);
  forceBase = modelForce(colectivoActual);
  if (antes != colectivoActual && ahora-ultimoAvisoColMs >= 1000) avisarColectivo();
}

void command(const char *s) {
  if (!*s) return;
  if (!strcmp(s, "hb")) { ultimoHbMs = millis(); hbVisto = true; return; }
  if (s[0] == 'y' && s[1] == 'b') {               // v10b: giro de la base en grados/s
    const float v = atof(s + 2);
    if (isfinite(v) && fabsf(v) <= 180) { baseYawRate = v*rad; baseYawMs = millis(); baseYawVisto = true; }
    return;
  }
  if (!strcmp(s, "parar")) { stop("# PARADA"); return; }
  if (!strcmp(s, "flotar") || !strcmp(s, "subir") || !strcmp(s, "bajar") ||
      !strcmp(s, "mantener") || !strcmp(s, "aterrizar") || !strcmp(s, "maximo")) {
    if (mode != CONTROL) {
      Serial.println("# VUELO: solo con los motores en CONTROL (despues de iniciar).");
      return;
    }
    // v11c: en MANTENER ALTURA, Subir/Bajar mueven la altura objetivo +-20 mm (en rampa) en vez del colectivo;
    // como mucho 100 mm por encima o 30 mm por debajo de donde se engancho (dentro de la guia de ~14 cm)
    if (alturaActiva && (!strcmp(s, "subir") || !strcmp(s, "bajar"))) {
      alturaObjMm = constrain(alturaObjMm + (!strcmp(s, "subir") ? 20.0f : -20.0f), alturaEngancheMm - 30.0f, alturaEngancheMm + 100.0f);
      Serial.printf("# ALTURA: objetivo %.0f mm (sube en rampa de 20 mm/s)\n", alturaObjMm);
      return;
    }
    if (strcmp(s, "mantener")) alturaActiva = false;   // v11: otra orden de colectivo apaga MANTENER ALTURA
    if (!strcmp(s, "flotar")) fijarObjetivo(flotarUs);
    else if (!strcmp(s, "maximo")) fijarObjetivo(colectivoTechoUs);
    else if (!strcmp(s, "subir")) fijarObjetivo(colectivoObjetivo + pasoUs);
    else if (!strcmp(s, "bajar")) fijarObjetivo(colectivoObjetivo - pasoUs);
    else if (!strcmp(s, "mantener")) {
      fijarObjetivo(int(lroundf(colectivoActual)));
      if (kfConfiable()) {          // v11: altura del Kalman confiable -> mantener tambien la altura
        alturaActiva = true; alturaRefMm = alturaObjMm = alturaEngancheMm = 1000.0f * kfX[0]; alturaColBase = colectivoActual;
        alturaInt = 0; alturaCorr = 0; alturaUltimaMs = 0;
        Serial.printf("# ALTURA: manteniendo %.0f mm sobre el suelo (Kalman; colectivo base %.0f us, +-60 us).\n", alturaRefMm, alturaColBase);
      } else {
        Serial.println("# ALTURA: el Kalman no tiene altura confiable (incertidumbre > 30 mm o laser sin medir); solo se fija el colectivo.");
      }
    }
    else fijarObjetivo(baseUs);
    return;
  }
  if (exportingLog) return;
  if (!strcmp(s, "log")) { startLogExport(); return; }
  if (!strcmp(s, "marca")) {
    if (mode == CONTROL) {
      markUs = micros()-runStartUs;
      marked = true;
      Serial.printf("# MARCA us=%lu\n", static_cast<unsigned long>(markUs));
    }
    return;
  }
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
    memset(filteredAngleRate, 0, sizeof(filteredAngleRate));
    memset(angleIntegralRate, 0, sizeof(angleIntegralRate));
    mode = RAMP;
    rampPulse = stopUs;
    peakPwmWriteUs = 0;
    accelEvents = accelCurrentUs = accelLongestUs = accelTotalUs = 0;
    accelMinG = INFINITY; accelMaxG = 0; accelOutside = false;
    scaleRP = scaleYaw = 1;
    yaw = 0;
    logNext = logCount = 0;
    marked = false;
    colectivoActual = baseUs;
    colectivoObjetivo = baseUs;
    altoDesdeMs = potenciaAltaDesdeMs = 0;
    forceBase = modelForce(baseUs);
    runStartUs = lastOutput = micros();
    Serial.println("# RAMPA a 1300 us. Enviar ! o parar para detener.");
    return;
  }
  stop("# ORDEN RECHAZADA. cal | vista | arm BANCO_FIJO | iniciar | flotar | maximo | subir | bajar | mantener | aterrizar | marca | parar | ! | log | yb");
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

void pollLaser() {
  if (!laserOK) return;
  const uint32_t t0 = micros();
  uint16_t mm;
  if (vl53::leerMmNoBloqueante(mm)) {
    mm = (uint16_t)lroundf(fmaxf(0.0f, laserCorregidoMm(mm)));   // calibrado con regla
    laserMm = mm; calT[calI] = millis(); calMm[calI] = mm; calI = (calI + 1) % 8;
    kfCorregir(alturaEjeM(laserCorregidoMm(vl53::ultimoMm) / 1000.0f));   // Kalman: medida aceptada, calibrada y llevada al eje
  }
  // v11: diagnostico del laser cada 2 s con los motores apagados (para probar blancos a mano)
  static uint32_t ultimoDiagLaserMs = 0;
  if (mode != CONTROL && mode != RAMP && millis() - ultimoDiagLaserMs >= 2000) {
    ultimoDiagLaserMs = millis();
    Serial.printf("# LASER DIAG estado=%u mm=%u filtrado=%u senal=%.2f MCPS ambiente=%.2f MCPS validas=%lu (debiles %lu) invalidas=%lu i2c=%lu estable=%d | kalman z=%.0f mm v=%.0f mm/s sigma=%.0f mm sesgo=%.2f m/s2 confiable=%d\n",
      vl53::estadoRango, vl53::ultimoMm, laserMm, vl53::senalMcps, vl53::ambienteMcps,
      (unsigned long)vl53::validas, (unsigned long)vl53::debilesAceptadas, (unsigned long)vl53::invalidas, (unsigned long)vl53::erroresI2c, laserEstable(),
      1000.0f * kfX[0], 1000.0f * kfX[1], 1000.0f * sqrtf(fmaxf(kfP[0][0], 0)), kfX[2], kfConfiable());
  }
  const uint32_t used = micros()-t0;
  if (used > laserReadPeakUs) laserReadPeakUs = used;
  if (vl53::erroresI2c != laserErrorBase) {
    laserErrorBase = vl53::erroresI2c;
    if (++laserStreak >= 20) { laserOK = false; vl53::activo = false; Serial.println("# LASER DESACTIVADO: errores I2C"); }
  } else if (used > 200) laserStreak = 0;
}

void report() {
  if (millis()-lastReport < 100 || exportingLog) return;
  lastReport = millis();
  char row[256];
  const char *labels[] = {"OFF", "VISTA", "ARMADO", "RAMPA", "CONTROL"};
  const int *pulses = mode == PREVIEW ? commandUs : appliedUs;
  const int n = snprintf(row, sizeof(row),
    "%lu,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%u,%d\n",
    static_cast<unsigned long>(lastReport), labels[mode], roll/rad, pitch/rad, yaw/rad,
    rates[0]/rad, rates[1]/rad, rates[2]/rad, scaleRP, scaleYaw,
    pulses[0], pulses[1], pulses[2], pulses[3],
    pulses[4], pulses[5], pulses[6], pulses[7], accelNorm,
    static_cast<unsigned>(vl53::ultimoMm),
    !laserOK ? 255 : (vl53::fresca() ? 0 : (vl53::estadoRango ? int(vl53::estadoRango) : 254)));
  if (n > 0 && n < int(sizeof(row)) && Serial.availableForWrite() >= n)
    Serial.write(reinterpret_cast<const uint8_t *>(row), n);
}

void checkAttitudeCutoff(uint32_t intervalUs) {
  if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw) ||
      !isfinite(accelNorm) || !isfinite(rates[0]) ||
      !isfinite(rates[1]) || !isfinite(rates[2])) {
    stop("# CORTE: ESTADO IMU NO FINITO");
    calibrated = false;
    return;
  }
  const float tilt = hypotf(roll, pitch)/rad;
  const bool excessiveTilt = tilt >= cutoffTiltDeg;
  // v10b: con el giro de la base, los limites miden el giro del dron RESPECTO a la base. Sin el dato y sobre la
  // base no hay corte por yaw relativo; queda un tope de 120 deg/s contra un giro descontrolado del dron.
  const float giroRel = rates[2] - (baseYawFresca() ? baseYawRate : 0.0f);
  const bool excessiveYaw = !rumboConVentana() && fabsf(yaw) >= 20*rad;                 // en ralenti: ventana, no corte
  const bool excessiveYawRate = rumboConVentana() ? fabsf(rates[2]) >= 120*rad : fabsf(giroRel) >= 35*rad;
  const bool excessiveAccel = accelNorm < .55f || accelNorm > 1.55f;
  accelMinG = fminf(accelMinG, accelNorm);
  accelMaxG = fmaxf(accelMaxG, accelNorm);
  if (excessiveAccel) {
    if (!accelOutside) ++accelEvents;
    accelCurrentUs += intervalUs;
    accelTotalUs += intervalUs;
    if (accelCurrentUs > accelLongestUs) accelLongestUs = accelCurrentUs;
  } else {
    accelCurrentUs = 0;
  }
  accelOutside = excessiveAccel;
  if (!(excessiveTilt || excessiveYaw || excessiveYawRate)) return;

  const char *phase = mode == RAMP ? "RAMPA" : "CONTROL";
  stop("# CORTE: LIMITE ANGULO O GIRO");
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
  Serial.print("# I2C responde en:");
  for (uint8_t dev = 8; dev < 120; ++dev) {
    Wire.beginTransmission(dev);
    if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", dev);
  }
  Serial.println();
  laserOK = vl53::iniciar(true);   // v11: largo alcance
  Wire.setTimeOut(8);
  Serial.printf("# LASER VL53L0X=%d modelo=0x%02X (solo telemetria)\n", laserOK, vl53::modelo);
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
  Serial.println("# OCTO M8 VUELO GUIA v11 + LASER: +X=M8 +Y=M7. Ki angulo=0.8. PWM 1180..2000, colectivo 1300..1950. Accel filtrado 10 Hz. Yaw controlado. Integral angulo 30 dps. Yaw suave, roll/pitch completos. En vuelo: giro 60 dps, mas integral.");
  Serial.println("# VUELO: flotar (1750) | maximo (1950, 20 s) | subir/bajar (+-25 us) | mantener | aterrizar (1300). Sube 25 us/s, 8 us/s sobre 1650. hb del PC sobre 1400 us.");
  Serial.println("# INTERLOCK: tilt estimado 12 deg, yaw 20 deg, yaw_vel 35 dps (yaw relativo a la base con yb; sin yb sobre la base: yaw libre, tope 120 dps). ACCEL SOLO REGISTRO.");
  Serial.println("# Enviar log tras parar para exportar los ultimos 16 s (100 Hz).");
  Serial.println("ms,modo,roll_deg,pitch_deg,yaw_rel_deg,p_dps,q_dps,r_dps,escala_RP,escala_yaw,M1_us,M2_us,M3_us,M4_us,M5_us,M6_us,M7_us,M8_us,accel_g,laser_mm,laser_estado");
}

void loop() {
  serialInput();
  if (mode == ARMED && millis()-armMs >= 10000) stop("# ARMADO VENCIDO");
  const uint32_t now = micros();
  const uint32_t elapsed = now-lastSample;
  if (imuOK && elapsed >= sampleUs) {
    lastSample = now;
    float a[3], g[3];
    RawImu raw;
    if (elapsed > 30000) {
      imuFault("# CORTE: CICLO TARDIO", elapsed, 0, false);
      return;
    }
    const bool readOK = readIMU(a, g, raw);
    const uint32_t readUs = micros()-now;
    if (!readOK || readUs > 15000) {
      imuFault(readOK ? "# CORTE: LECTURA IMU LENTA" : "# CORTE: ERROR I2C MPU6050",
        elapsed, readUs, true);
      return;
    }
    // Kalman de altura: sin calibracion no hay actitud ni aceleracion vertical fiable; se predice con a = 0 para que
    // la incertidumbre crezca y el filtro siga al laser (si no, se "congela" promediando medidas viejas)
    if (calibrating || !calibrated) kfPredecir(elapsed*1e-6f, 0.0f);
    if (calibrating) collectCalibration(a, g);
    else if (calibrated) {
      const float dt = elapsed*1e-6f;
      updateAttitude(a, g, dt);
      if (driving()) {
        if ((muestraPar++ & 1) == 0) saveSample(now,raw);
        checkAttitudeCutoff(elapsed);
      }
      if (mode == PREVIEW || mode == CONTROL) controlStep(dt);
    }
    pollLaser();
  }
  if (driving() && now-lastOutput >= periodUs) {
    lastOutput = now;
    if (mode == RAMP) {
      rampPulse = min(baseUs, rampPulse+5);
      if (!uniformPWM(rampPulse)) stop("# FALLO PWM");
      else if (rampPulse == baseUs) mode = CONTROL;
    } else {
      actualizarColectivo(periodUs*1e-6f);
      if (!writePWM(commandUs)) stop("# FALLO PWM");
    }
  }
  report();
  streamLog();
  delay(1);
}
