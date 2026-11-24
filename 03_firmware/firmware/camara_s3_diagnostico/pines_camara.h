#pragma once

// Perfil de referencia ESP32S3_EYE usado por el ejemplo CameraWebServer de
// Espressif/Freenove. Pendiente de confirmar en la placa HTCRAL RE1.3 real:
// la serigrafia N16R8 identifica la memoria, pero no demuestra el cableado.
// No se prueban automaticamente otros mapas de pines.
namespace camhw {
constexpr int pwdn = -1, reset = -1;
constexpr int xclk = 15, sda = 4, scl = 5;
constexpr int d0 = 11, d1 = 9, d2 = 8, d3 = 10;
constexpr int d4 = 12, d5 = 18, d6 = 17, d7 = 16;
constexpr int vsync = 6, href = 7, pclk = 13;
constexpr int futuroServoX = 41, futuroServoY = 42;
constexpr size_t capacidadFoto = 96 * 1024;
}
