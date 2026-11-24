/*
 * DSM: Diagnóstico seguro de IMU (MPU-6050 / Giroscopio y Acelerómetro)
 * NodeMCU-32S (ESP32): SDA = GPIO23, SCL = GPIO22 (400 kHz)
 * MOTORES APAGADOS (salidas LOW sin PWM) para máxima seguridad en banco.
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Pines de los 8 motores: forzados a LOW
constexpr uint8_t ESC[8] = {25, 26, 27, 14, 21, 19, 5, 17};

constexpr uint8_t SDA_PIN = 23;
constexpr uint8_t SCL_PIN = 22;

uint8_t mpuAddr = 0x68;
bool mpuDetectado = false;
uint8_t whoAmI = 0x00;

float gyroBias[3] = {0.0f, 0.0f, 0.0f};  // gx, gy, gz en deg/s
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
uint32_t ultimoMuestreoUs = 0;
bool emitir = true;

bool escribirReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t leerReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(static_cast<int>(addr), 1) != 1) return 0xFF;
  return Wire.read();
}

void escanearI2C() {
  Serial.println("\n# --- ESCANEO DEL BUS I2C (SDA=23, SCL=22) ---");
  uint8_t encontrados = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("# [I2C] Dispositivo encontrado en 0x%02X", a);
      if (a == 0x68) Serial.print(" (MPU-6050 por defecto)");
      else if (a == 0x69) Serial.print(" (MPU-6050 con AD0=HIGH)");
      else if (a == 0x29) Serial.print(" (Laser ToF VL53L0X)");
      Serial.println();
      encontrados++;
      if (a == 0x68 || a == 0x69) mpuAddr = a;
    }
  }
  if (!encontrados) {
    Serial.println("# [I2C] ADVERTENCIA: No se encontro ningun dispositivo en el bus I2C.");
    Serial.println("# Revisa cables: VCC (3.3V/5V), GND, SDA->GPIO23, SCL->GPIO22.");
  }
  Serial.printf("# Total dispositivos I2C: %u\n# ---------------------------------------------\n", encontrados);
}

bool iniciarMPU() {
  whoAmI = leerReg(mpuAddr, 0x75);
  Serial.printf("# WHO_AM_I registro 0x75: 0x%02X ", whoAmI);
  if (whoAmI == 0x68) {
    Serial.println("(MPU-6050 original confirmado)");
  } else if (whoAmI == 0x70 || whoAmI == 0x71) {
    Serial.println("(MPU-6500 / MPU-9250 detectado)");
  } else if (whoAmI == 0x98) {
    Serial.println("(ICM-20600 / ICM-20602 detectado)");
  } else {
    Serial.printf("(Respuesta desconocida o fallo I2C)\n");
    if (whoAmI == 0xFF || whoAmI == 0x00) return false;
  }

  // 1. Salir de sleep, usar PLL con giróscopo eje X como referencia de reloj
  if (!escribirReg(mpuAddr, 0x6B, 0x01)) return false;
  delay(15);

  // 2. Filtro pasa-bajas digital DLPF ~44 Hz (suprime vibraciones de rotores)
  escribirReg(mpuAddr, 0x1A, 0x03);

  // 3. Giroscopio +-500 deg/s (65.5 LSB / deg/s)
  escribirReg(mpuAddr, 0x1B, 0x08);

  // 4. Acelerómetro +-4g (8192 LSB / g)
  escribirReg(mpuAddr, 0x1C, 0x08);

  delay(20);
  return true;
}

void calibrarGiro() {
  Serial.println("# Calibrando offset del giroscopio (deja el dron INMOVIL sobre la mesa)...");
  double sumGx = 0, sumGy = 0, sumGz = 0;
  constexpr int MUESTRAS = 300;
  int validas = 0;

  for (int i = 0; i < MUESTRAS; i++) {
    Wire.beginTransmission(mpuAddr);
    Wire.write(0x43); // Registros GYRO_XOUT_H
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<int>(mpuAddr), 6) == 6) {
      int16_t rawGx = (Wire.read() << 8) | Wire.read();
      int16_t rawGy = (Wire.read() << 8) | Wire.read();
      int16_t rawGz = (Wire.read() << 8) | Wire.read();
      sumGx += rawGx / 65.5;
      sumGy += rawGy / 65.5;
      sumGz += rawGz / 65.5;
      validas++;
    }
    delay(4);
  }

  if (validas > 100) {
    gyroBias[0] = static_cast<float>(sumGx / validas);
    gyroBias[1] = static_cast<float>(sumGy / validas);
    gyroBias[2] = static_cast<float>(sumGz / validas);
    Serial.printf("# Calibracion completa (%d muestras). Bias [deg/s]: Gx=%+.2f, Gy=%+.2f, Gz=%+.2f\n",
                  validas, gyroBias[0], gyroBias[1], gyroBias[2]);
  } else {
    Serial.println("# Error durante la calibracion: lecturas I2C insuficientes.");
  }
}

void setup() {
  // 1. Apagar y bloquear los 8 motores
  for (uint8_t p : ESC) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  Serial.begin(115200);
  delay(400);

  Serial.println("\n=======================================================");
  Serial.println(" DSM: DIAGNOSTICO DE GIROSCOPIO / IMU (MPU-6050)");
  Serial.println(" ESP32 NodeMCU-32S | Motores: BLOQUEADOS (LOW)");
  Serial.println("=======================================================");

  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  Wire.setTimeOut(10);
  delay(100);

  escanearI2C();

  mpuDetectado = iniciarMPU();
  if (mpuDetectado) {
    Serial.println("# [IMU] Inicializacion de MPU exitosa a 400 kHz.");
    calibrarGiro();

    // Inicializar ángulos a partir del acelerómetro inicial
    Wire.beginTransmission(mpuAddr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<int>(mpuAddr), 6) == 6) {
      int16_t rawAx = (Wire.read() << 8) | Wire.read();
      int16_t rawAy = (Wire.read() << 8) | Wire.read();
      int16_t rawAz = (Wire.read() << 8) | Wire.read();
      float ax = rawAx / 8192.0f;
      float ay = rawAy / 8192.0f;
      float az = rawAz / 8192.0f;
      roll = atan2f(ay, az) * 57.29578f;
      pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
      yaw = 0.0f;
    }
  } else {
    Serial.println("# [ERROR] No se pudo inicializar la IMU.");
    Serial.println("# Comprueba la alimentacion del GY-521 y conexion I2C (GPIO23 SDA, GPIO22 SCL).");
  }

  ultimoMuestreoUs = micros();
  Serial.println("\n# Comandos serie: 'ver' | 'calibrar' | 'scan' | 'pausa' | 'continuar'");
}

void loop() {
  // Comandos por consola
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "ver" || cmd == "estado") {
      emitir = true;
    } else if (cmd == "calibrar") {
      calibrarGiro();
    } else if (cmd == "scan") {
      escanearI2C();
      mpuDetectado = iniciarMPU();
    } else if (cmd == "pausa") {
      emitir = false;
      Serial.println("# Emision pausada.");
    } else if (cmd == "continuar" || cmd == "c") {
      emitir = true;
      Serial.println("# Emision reanudada.");
    }
  }

  if (!mpuDetectado) {
    static uint32_t tReintento = 0;
    if (millis() - tReintento >= 3000) {
      tReintento = millis();
      Serial.println("# Reintentando busqueda de MPU...");
      escanearI2C();
      mpuDetectado = iniciarMPU();
      if (mpuDetectado) calibrarGiro();
    }
    delay(20);
    return;
  }

  // Leer 14 bytes: Accel (6), Temp (2), Gyro (6)
  uint32_t ahoraUs = micros();
  float dt = (ahoraUs - ultimoMuestreoUs) * 1e-6f;
  if (dt < 0.010f) {  // ~50 Hz ciclo de integracion
    delay(1);
    return;
  }
  ultimoMuestreoUs = ahoraUs;

  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(static_cast<int>(mpuAddr), 14) != 14) {
    return;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  // Escalas: +-4g -> 8192 LSB/g, +-500 deg/s -> 65.5 LSB/(deg/s)
  float ax = rawAx / 8192.0f;
  float ay = rawAy / 8192.0f;
  float az = rawAz / 8192.0f;
  float tempC = (rawTemp / 340.0f) + 36.53f;

  float gx = (rawGx / 65.5f) - gyroBias[0];
  float gy = (rawGy / 65.5f) - gyroBias[1];
  float gz = (rawGz / 65.5f) - gyroBias[2];

  // Filtro complementario de actitud
  float rollAcc = atan2f(ay, az) * 57.29578f;
  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;

  roll = 0.98f * (roll + gx * dt) + 0.02f * rollAcc;
  pitch = 0.98f * (pitch + gy * dt) + 0.02f * pitchAcc;
  yaw += gz * dt;

  // Imprimir a ~20 Hz
  static uint32_t tPrint = 0;
  if (emitir && millis() - tPrint >= 50) {
    tPrint = millis();
    Serial.printf("{\"roll\":%+.2f,\"pitch\":%+.2f,\"yaw\":%+.2f,\"gx\":%+.1f,\"gy\":%+.1f,\"gz\":%+.1f,\"ax\":%+.2f,\"ay\":%+.2f,\"az\":%+.2f,\"temp\":%.1f}\n",
                  roll, pitch, yaw, gx, gy, gz, ax, ay, az, tempC);
  }
}
