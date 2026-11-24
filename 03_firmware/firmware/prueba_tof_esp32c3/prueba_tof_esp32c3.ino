// Prueba aislada de ToF en ESP32-C3 Super Mini: SDA4, SCL5, 3V3 y GND.
// No usa motores, servos ni el firmware del octocoptero.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#error "Seleccionar ESP32C3 Dev Module; este sketch es para el C3 de prueba."
#endif

constexpr uint8_t sdaPin = 4, sclPin = 5, tofAddress = 0x29;
Adafruit_VL53L0X tof;
bool running = false;
uint32_t lastScan = 0, lastStatus = 0;
// Perfil elegido para esta prueba de banco despues de comparar n/l/p.
// Es mas lento (200 ms), no configura un controlador de altura del dron.
char profile = 'p';

void settings() {
  Serial.printf("# PERFIL=%c presupuesto_us=%lu signal_min_mcps=%.5f ignore_activo=%u ignore_mcps_spad=%.5f\n",
    profile, static_cast<unsigned long>(tof.getMeasurementTimingBudgetMicroSeconds()),
    tof.getLimitCheckValue(VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE)/65536.0,
    unsigned(tof.getLimitCheckEnable(VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD)),
    tof.getLimitCheckValue(VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD)/65536.0);
}

void setProfile(char selected) {
  tof.stopRangeContinuous();
  delay(20);
  profile=selected;
  bool ok=tof.configSensor(selected=='p' ? Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY
                                       : Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
  if (selected!='p') ok=ok && tof.setMeasurementTimingBudgetMicroSeconds(selected=='l' ? 200000 : 33000);
  running=ok && tof.startRangeContinuous(selected=='n' ? 50 : 250);
  settings();
  Serial.println("ms,distancia_mm,range_status,valida,senal_mcps,ambiente_mcps,spads,dmax_mm,error_api");
}

bool readRegister8(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(tofAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(tofAddress, uint8_t(1)) != 1)
    return false;
  value = Wire.read();
  return true;
}

void scanAndStart() {
  Serial.println("# Escaneo I2C: SDA=GPIO4 SCL=GPIO5, frecuencia=100kHz");
  bool found = false;
  unsigned count = 0;
  for (uint8_t addr = 8; addr < 120; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("# Responde 0x%02X\n", addr);
      found |= addr == tofAddress;
      ++count;
    }
  }
  if (!found) {
    Serial.printf("# SIN_TOF_0x29 dispositivos=%u. Revisar alimentacion, cables y XSHUT. No prueba que este averiado.\n", count);
    return;
  }
  uint8_t model = 0;
  if (!readRegister8(0xC0, model)) {
    Serial.println("# ACK_0x29_PERO_FALLO_LECTURA_ID");
    return;
  }
  Serial.printf("# Registro modelo 0xC0=0x%02X (VL53L0X esperado: 0xEE)\n", model);
  if (model != 0xEE) {
    // VL53L1X usa un registro de identificacion de 16 bits.
    Wire.beginTransmission(tofAddress); Wire.write(0x01); Wire.write(0x0F);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(tofAddress, uint8_t(1)) == 1)
      Serial.printf("# ID_0x010F=0x%02X (0xEA sugiere VL53L1X; requiere otro controlador)\n", Wire.read());
    Serial.println("# MODELO_DISTINTO: no se inicializa como VL53L0X.");
    return;
  }
  if (!tof.begin(tofAddress, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY)) {
    Serial.printf("# VL53L0X_IDENTIFICADO_PERO_INICIO_FALLO status=%d\n", int(tof.Status));
    return;
  }
  running = tof.startRangeContinuous(250);
  Serial.printf("# Lectura continua %s. Apuntar a una superficie a 10-30 cm.\n", running ? "ACTIVA" : "FALLO");
  settings();
  Serial.println("# Comandos: n=normal, l=normal lento 200ms, p=perfil oficial alta precision, i=ajustes");
  Serial.println("ms,distancia_mm,range_status,valida,senal_mcps,ambiente_mcps,spads,dmax_mm,error_api");
}

void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis()-start < 3000) delay(10);
  Wire.begin(sdaPin, sclPin, 100000);
  Wire.setTimeOut(20);
  delay(300);
  Serial.println("# PRUEBA_ToF_ESP32C3: solo sensor, sin conexion al carrito.");
  scanAndStart();
  lastScan = millis();
}

void loop() {
  while (Serial.available()) {
    const char cmd=Serial.read();
    if (running && (cmd=='n' || cmd=='l' || cmd=='p')) setProfile(cmd);
    else if (running && cmd=='i') settings();
  }
  if (!running) {
    if (millis()-lastScan >= 3000) { lastScan=millis(); scanAndStart(); }
    delay(5);
    return;
  }
  if (tof.isRangeComplete()) {
    VL53L0X_RangingMeasurementData_t measurement = {};
    const int status = tof.getRangingMeasurement(&measurement, false);
    const int clearStatus = tof.clearInterruptMask(false);
    const bool valid = status == 0 && clearStatus == 0 && measurement.RangeStatus == 0
      && measurement.RangeMilliMeter > 0 && measurement.RangeMilliMeter <= 2000;
    Serial.printf("%lu,%u,%u,%d,%.5f,%.5f,%.3f,%u,%d\n", static_cast<unsigned long>(millis()),
      unsigned(measurement.RangeMilliMeter), unsigned(measurement.RangeStatus), valid,
      measurement.SignalRateRtnMegaCps/65536.0,measurement.AmbientRateRtnMegaCps/65536.0,
      measurement.EffectiveSpadRtnCount/256.0,unsigned(measurement.RangeDMaxMilliMeter),status ? status : clearStatus);
    lastStatus = millis();
    if (status || clearStatus) {
      Serial.printf("# ERROR_LECTURA status=%d clear=%d\n", status, clearStatus);
      running = false; lastScan=millis();
    }
  } else if (millis()-lastStatus > 2000) {
    Serial.printf("# SIN_MEDIDA_NUEVA status=%d\n", int(tof.Status));
    lastStatus = millis();
  }
  delay(5);
}
