#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Usa la API ST incluida en Adafruit 1.2.5. No confundir un ACK I2C o una
// distancia dentro de rango con RangeStatus=0 (medicion realmente valida).
namespace vl53 {
constexpr uint8_t ADDR = 0x29;
static Adafruit_VL53L0X sensor;
static bool activo = false;
static uint8_t modelo = 0, estadoRango = 255;
static int errorApi = 0;
static uint16_t ultimoMm = 0;
static uint32_t secuencia = 0, validas = 0, invalidas = 0, erroresI2c = 0;
static uint32_t ultimaValidaMs = 0, ultimaConsultaMs = 0;
static float senalMcps = 0, ambienteMcps = 0;
// Modo "senal debil" (lo activa iniciar(true)): se aceptan tambien las medidas con estado 2 (senal baja) si la
// senal es >= 0,03 MCPS y la distancia es razonable, pero solo se dan por validas si concuerdan con las
// anteriores: se entrega la MEDIANA de las ultimas 3 y se descarta una medida que se aleje mas de 60 mm de ella
// (si 3 seguidas se alejan, se asume que el cambio es real y se reinicia el filtro).
static bool aceptarDebiles = false;
static uint16_t bufMm[3] = {0, 0, 0};
static uint8_t nBuf = 0, saltosSeguidos = 0;
static uint32_t debilesAceptadas = 0;

inline uint8_t rReg(uint8_t reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)ADDR, 1) != 1) {
    erroresI2c++;
    return 0;
  }
  return Wire.read();
}

inline bool iniciar(bool largoAlcance = false) {
  activo = false;
  estadoRango = 255;
  ultimaValidaMs = 0;
  ultimoMm = 0;
  modelo = rReg(0xC0);
  if (modelo != 0xEE) return false;  // No inicializar un modelo distinto.
  activo = sensor.begin(ADDR, false, &Wire);
  errorApi = sensor.Status;
  if (!activo) return false;
  // Largo alcance (ST): limite de senal 0,1 MCPS y pulsos VCSEL mas largos. Acepta retornos mas debiles
  // (superficies oscuras, en angulo o lejanas) a cambio de algo mas de ruido. El codigo 2 ("signal fail")
  // que daba el sensor en vuelo es justamente senal de retorno por debajo del limite por defecto (0,25 MCPS).
  if (largoAlcance) {
    sensor.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
    // aun mas permisivo que el perfil de ST: senal minima 0,05 MCPS, sigma hasta 60 mm y 66 ms por medida
    // (mas luz integrada por medida = mejor relacion senal/ruido con la luz ambiente alta del laboratorio)
    sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.05 * 65536));
    sensor.setLimitCheckValue(VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
    sensor.setMeasurementTimingBudgetMicroSeconds(66000);
    aceptarDebiles = true;
  }
  activo = sensor.startRangeContinuous(largoAlcance ? 70 : 50);
  errorApi = sensor.Status;
  return activo;
}

inline bool leerMmNoBloqueante(uint16_t &mm) {
  // No espera a que termine una medicion. Las transacciones I2C siguen siendo
  // sincronas y su duracion real debe medirse en el hardware.
  if (!activo || millis()-ultimaConsultaMs < 10) return false;
  ultimaConsultaMs = millis();
  bool lista = sensor.isRangeComplete();
  errorApi = sensor.Status;
  if (errorApi != VL53L0X_ERROR_NONE) {
    erroresI2c++;
    estadoRango = 255;
    return false;
  }
  if (!lista) return false;
  VL53L0X_RangingMeasurementData_t medida = {};
  errorApi = sensor.getRangingMeasurement(&medida, false);
  int errorClear = sensor.clearInterruptMask(false);
  secuencia++;
  estadoRango = medida.RangeStatus;
  ultimoMm = medida.RangeMilliMeter;
  senalMcps = medida.SignalRateRtnMegaCps / 65536.0f;
  ambienteMcps = medida.AmbientRateRtnMegaCps / 65536.0f;
  if (errorApi || errorClear) {
    if(errorClear)errorApi=errorClear;
    erroresI2c++;
    invalidas++;
    estadoRango = 255;
    return false;
  }
  const bool buena = estadoRango == 0 && ultimoMm > 0 && ultimoMm <= 2000;
  const bool debil = aceptarDebiles && estadoRango == 2 && senalMcps >= 0.03f && ultimoMm >= 30 && ultimoMm <= 1500;
  if (!buena && !debil) {
    invalidas++;
    return false;
  }
  if (!aceptarDebiles) {          // comportamiento original
    validas++; ultimaValidaMs = millis(); mm = ultimoMm; return true;
  }
  // filtro de mediana de 3 con rechazo de saltos
  if (nBuf == 3) {
    uint16_t a = bufMm[0], b = bufMm[1], c = bufMm[2];
    uint16_t med = max(min(a, b), min(max(a, b), c));
    if (abs((int)ultimoMm - (int)med) > 60 && ++saltosSeguidos < 3) { invalidas++; return false; }
    if (saltosSeguidos >= 3) nBuf = 0;   // tres saltos seguidos: el cambio es real, se reinicia
  }
  saltosSeguidos = 0;
  bufMm[nBuf < 3 ? nBuf : 0] = ultimoMm;
  if (nBuf < 3) nBuf++; else { bufMm[0] = bufMm[1]; bufMm[1] = bufMm[2]; bufMm[2] = ultimoMm; }
  if (nBuf < 3) { invalidas++; return false; }   // hasta tener 3 medidas no se entrega nada
  if (debil) debilesAceptadas++;
  validas++;
  ultimaValidaMs = millis();
  mm = max(min(bufMm[0], bufMm[1]), min(max(bufMm[0], bufMm[1]), bufMm[2]));   // mediana
  return true;
}

inline bool fresca(uint32_t maxEdadMs = 200) {
  return activo && ultimaValidaMs && millis()-ultimaValidaMs <= maxEdadMs;
}
}  // namespace vl53
