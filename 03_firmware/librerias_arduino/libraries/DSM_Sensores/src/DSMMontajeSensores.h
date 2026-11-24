#pragma once
#include <stdint.h>
// Ubicación física declarada por George, 05/11/2026.
// La IMU conserva orientación: NO cambiar rotImuDeg al cambiar de brazo.
namespace montaje_dsm {
constexpr uint8_t IMU_BRAZO = 8;
constexpr uint8_t LASER_BRAZO = 1;
constexpr const char* REVISION = "imu8-laser1-20261105";
}
