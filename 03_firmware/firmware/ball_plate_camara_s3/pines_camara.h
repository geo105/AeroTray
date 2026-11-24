#pragma once

// Perfil de pines verificado en hardware real (ESP32-S3-CAM N16R8 RE1.3 + OV2640).
// La camara usa LEDC_CHANNEL_0 y LEDC_TIMER_0 para XCLK.
// Los dos servos usan canales LEDC independientes (canales 2 y 3, timer 1 a 50 Hz).
namespace camhw {
constexpr int pwdn = -1, reset = -1;
constexpr int xclk = 15, sda = 4, scl = 5;
constexpr int d0 = 11, d1 = 9, d2 = 8, d3 = 10;
constexpr int d4 = 12, d5 = 18, d6 = 17, d7 = 16;
constexpr int vsync = 6, href = 7, pclk = 13;
constexpr uint8_t servoXDefault = 41;
constexpr uint8_t servoYDefault = 42;
constexpr int ancho = 320;
constexpr int alto = 240;
constexpr size_t capacidadFoto = 64 * 1024;
}  // namespace camhw
