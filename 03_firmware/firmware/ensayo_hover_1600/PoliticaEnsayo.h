#pragma once
#include <cmath>
#include <cstdint>
#include <cstddef>

namespace ensayo {
constexpr int stopUs = 1000, floorUs = 1180, baseUs = 1300, maxUs = 1600;
constexpr float cutoffTiltDeg = 15.0f;
constexpr uint32_t heartbeatTimeoutMs = 500, maxRunMs = 90000;
constexpr float upUsPerSecond = 5.0f, downUsPerSecond = 20.0f;
// Politica provisional de banco rev. 2. Solo filtra la PROTECCION de aceleracion;
// no modifica la IMU usada por el estimador, el PID ni el corte angular.
constexpr uint8_t accelPolicyRevision = 2;
constexpr float accelTauSeconds = .020f;
constexpr float accelSoftMinG = .55f, accelSoftMaxG = 1.55f;
constexpr float accelHardMinG = .20f, accelHardMaxG = 1.90f;
constexpr uint32_t accelPersistUs = 40000;
enum class AccelTrip : uint8_t { NONE=0, SUSTAINED=1, HARD_HIGH=2, HARD_LOW=3, INVALID=4 };

struct AccelGuard {
  float raw=1, filtered=1;
  uint32_t previousUs=0, outsideSinceUs=0, outsideUs=0;
  bool initialized=false, outside=false;
  AccelTrip lastTrip=AccelTrip::NONE;

  void reset(float seed, uint32_t now) {
    raw=filtered=std::isfinite(seed) && seed>0 ? seed : 1.0f;
    previousUs=outsideSinceUs=now; outsideUs=0;
    initialized=true; outside=false; lastTrip=AccelTrip::NONE;
  }
  AccelTrip update(float g, uint32_t now, bool armed) {
    if (!initialized) reset(g,now);
    const uint32_t elapsed=now-previousUs;
    previousUs=now; raw=g;
    if (!std::isfinite(g) || g<0) {
      if (armed) lastTrip=AccelTrip::INVALID;
      return armed ? lastTrip : AccelTrip::NONE;
    }
    const float dt=elapsed*1e-6f;
    filtered += dt/(accelTauSeconds+dt)*(g-filtered);
    if (!armed) {
      outside=false; outsideUs=0; outsideSinceUs=now;
      return AccelTrip::NONE;
    }
    if (lastTrip!=AccelTrip::NONE) return lastTrip;
    if (g>=accelHardMaxG) lastTrip=AccelTrip::HARD_HIGH;
    else if (g<=accelHardMinG) lastTrip=AccelTrip::HARD_LOW;
    if (lastTrip!=AccelTrip::NONE) return lastTrip;
    if (filtered<accelSoftMinG || filtered>accelSoftMaxG) {
      if (!outside) { outside=true; outsideSinceUs=now; }
      outsideUs=now-outsideSinceUs;
      if (outsideUs>=accelPersistUs) lastTrip=AccelTrip::SUSTAINED;
    } else {
      outside=false; outsideUs=0; outsideSinceUs=now;
    }
    return lastTrip;
  }
};
inline float clip(float v, float lo, float hi) { return fminf(hi, fmaxf(lo, v)); }
inline float tiltDegrees(float roll, float pitch) {
  if (!std::isfinite(roll) || !std::isfinite(pitch)) return INFINITY;
  return acosf(clip(cosf(roll)*cosf(pitch), -1, 1))*180.0f/3.14159265358979323846f;
}
inline bool fresh(bool seen, uint32_t now, uint32_t last) {
  return seen && uint32_t(now-last) < heartbeatTimeoutMs;
}
inline bool canArm(bool pwm, bool imu, bool calibrated, bool calibrating,
                   bool heartbeat, float tilt, float rateNorm) {
  return pwm && imu && calibrated && !calibrating && heartbeat
    && std::isfinite(tilt) && std::isfinite(rateNorm) && tilt <= 4.0f && rateNorm <= 5.0f;
}
inline float slew(float current, float target, float dt) {
  const float step = (target > current ? upUsPerSecond : downUsPerSecond)*dt;
  return clip(current + clip(target-current, -step, step), baseUs, maxUs);
}
// Preserve RP moments by moving collective within the feasible interval. Only
// reduce moments when their span cannot fit between the two per-motor limits.
inline float allocate(float desiredBase, float lo, float hi, const float delta[8],
                      float out[8], float &actualBase) {
  float low = delta[0], high = delta[0];
  for (int i=1; i<8; ++i) { low=fminf(low,delta[i]); high=fmaxf(high,delta[i]); }
  const float span = high-low;
  const float scale = span > hi-lo ? (hi-lo)/span : 1.0f;
  actualBase = clip(desiredBase, lo-scale*low, hi-scale*high);
  for (int i=0; i<8; ++i) out[i] = clip(actualBase+scale*delta[i],lo,hi);
  return scale;
}
inline uint16_t crc16(const uint8_t *p, size_t n) {
  uint16_t crc=0xffff;
  while (n--) {
    crc ^= uint16_t(*p++) << 8;
    for (int j=0;j<8;++j) crc=(crc&0x8000) ? uint16_t((crc<<1)^0x1021) : uint16_t(crc<<1);
  }
  return crc;
}
}
