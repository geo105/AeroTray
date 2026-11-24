// control_octo.h - Nucleo de estabilizacion PI-D y asignacion de 8 motores para el UAV en rotula
// Sin dependencias de Arduino: se puede compilar tanto en el ESP32 NodeMCU-32S como en pruebas de PC.
//
// Disposicion de motores segun Dron_octo_RevD_grupo.pdf (pagina 5):
//   M1 (0 deg frente,     GPIO25, rama A): Pitch +1.000, Roll  0.000, Yaw +1.0
//   M2 (90 deg derecha,   GPIO26, rama B): Pitch  0.000, Roll +1.000, Yaw +1.0
//   M3 (180 deg atras,    GPIO27, rama A): Pitch -1.000, Roll  0.000, Yaw +1.0
//   M4 (270 deg izq.,     GPIO14, rama B): Pitch  0.000, Roll -1.000, Yaw +1.0
//   M5 (45 deg fr-der,    GPIO21, rama A): Pitch +0.707, Roll +0.707, Yaw -1.0
//   M6 (135 deg at-der,   GPIO19, rama B): Pitch -0.707, Roll +0.707, Yaw -1.0
//   M7 (225 deg at-izq,   GPIO5,  rama A): Pitch -0.707, Roll -0.707, Yaw -1.0
//   M8 (315 deg fr-izq,   GPIO17, rama B): Pitch +0.707, Roll -0.707, Yaw -1.0
#pragma once
#include <math.h>

namespace octo {

inline float limitar(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

static const float MIX_PITCH[8] = {
  +1.000000f,  0.000000f, -1.000000f,  0.000000f,
  +0.707107f, -0.707107f, -0.707107f, +0.707107f
};
static const float MIX_ROLL[8] = {
   0.000000f, +1.000000f,  0.000000f, -1.000000f,
  +0.707107f, +0.707107f, -0.707107f, -0.707107f
};
static const float MIX_YAW[8] = {
  +1.0f, +1.0f, +1.0f, +1.0f,
  -1.0f, -1.0f, -1.0f, -1.0f
};

struct EjePID {
  // Ganancias en [us / deg], [us / (deg*s)], [us / (deg/s)]
  // Segun Rev D p.5: par por us sube ~3x con 8 motores a 350 mm -> iniciar con ganancias moderadas
  float kp = 4.5f;
  float ki = 1.8f;
  float kd = 1.2f;
  float iMax = 80.0f;       // limite integral [us]
  float uMax = 220.0f;      // esfuerzo maximo por eje [us]
  float tauRate = 0.010f;   // filtro pasa-bajos para velocidad angular [s]

  float integ = 0.0f;
  float rateFil = 0.0f;
  float esfuerzo = 0.0f;

  void reiniciar() {
    integ = 0.0f;
    rateFil = 0.0f;
    esfuerzo = 0.0f;
  }

  float paso(float refDeg, float medDeg, float rateDegS, float dt, bool permitirIntegrar) {
    if (!isfinite(refDeg) || !isfinite(medDeg) || !isfinite(rateDegS) || !isfinite(dt) || dt <= 0 || dt > 0.05f) {
      reiniciar();
      return 0;
    }
    float alfa = dt / (tauRate + dt);
    rateFil += (rateDegS - rateFil) * alfa;
    float err = refDeg - medDeg;
    if (permitirIntegrar && ki > 0.0f) {
      integ = limitar(integ + ki * err * dt, -iMax, iMax);
    }
    float u = kp * err - kd * rateFil + integ;
    esfuerzo = limitar(u, -uMax, uMax);
    return esfuerzo;
  }
};

struct ControlAltura {
  // Control PI-D opcional sobre el riel vertical [cm] -> correccion de base [us]
  bool activo = false;
  float kp = 12.0f;         // us / cm
  float ki = 4.0f;          // us / (cm*s)
  float kd = 8.0f;          // us / (cm/s)
  float iMax = 100.0f;
  float uMax = 160.0f;
  float zRefCm = 30.0f;     // objetivo comunicado: +30 cm sobre el cero inicial
  float zConsignaCm = 0.0f;
  float rampaCmS = 5.0f;    // rampa de referencia; no es una velocidad de vuelo garantizada
  float zFilCm = 0.0f;
  float vzFilCmS = 0.0f;
  float integ = 0.0f;
  float esfuerzo = 0.0f;
  bool inicializado = false;

  void reiniciar() {
    integ = 0.0f;
    vzFilCmS = 0.0f;
    esfuerzo = 0.0f;
    inicializado = false;
  }

  float paso(float zMedCm, float dt, bool medidaValida, bool permitirIntegrar) {
    if (!activo || !medidaValida || !isfinite(zMedCm) || !isfinite(dt) || dt <= 0.0f || dt > 0.05f) {
      reiniciar();
      return 0;
    }
    if (!inicializado) {
      zFilCm = zMedCm;
      zConsignaCm = zMedCm;
      vzFilCmS = 0.0f;
      inicializado = true;
    } else {
      float zPrev = zFilCm;
      zFilCm += (zMedCm - zFilCm) * limitar(dt / (0.04f + dt), 0.05f, 1.0f);
      float vzBruta = (zFilCm - zPrev) / dt;
      vzFilCmS += (vzBruta - vzFilCmS) * limitar(dt / (0.08f + dt), 0.05f, 1.0f);
    }
    float pasoRef = rampaCmS*dt;
    zConsignaCm += limitar(zRefCm-zConsignaCm, -pasoRef, pasoRef);
    float err = zConsignaCm - zFilCm;
    if (permitirIntegrar && ki > 0.0f) {
      integ = limitar(integ + ki * err * dt, -iMax, iMax);
    }
    esfuerzo = limitar(kp * err - kd * vzFilCmS + integ, -uMax, uMax);
    return esfuerzo;
  }
};

struct AsignadorOcto {
  float usArmado = 1000.0f;
  float usMinActivo = 1120.0f;
  float usMax = 1950.0f;
  float trim[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Mezcla con prioridad de actitud (Pitch/Roll) sobre Yaw y desplazamiento de base ante saturacion
  bool mezclar(float baseUs, float uPitch, float uRoll, float uYaw, float salUs[8]) const {
    float diffPR[8];
    float minPR = 0.0f, maxPR = 0.0f;
    for (int i = 0; i < 8; i++) {
      diffPR[i] = uPitch * MIX_PITCH[i] + uRoll * MIX_ROLL[i] + trim[i];
      if (diffPR[i] < minPR) minPR = diffPR[i];
      if (diffPR[i] > maxPR) maxPR = diffPR[i];
    }

    // 1. Si el diferencial Pitch+Roll supera el rango total disponible, escalarlo proporcionalmente
    float spanMax = (usMax - usMinActivo) * 0.85f;
    float spanActual = maxPR - minPR;
    float escalaPR = 1.0f;
    bool saturado = false;
    if (spanActual > spanMax && spanActual > 1e-3f) {
      escalaPR = spanMax / spanActual;
      saturado = true;
    }

    // 2. Ajustar base colectiva para que base + diffPR quede dentro de [usMinActivo, usMax]
    float b = limitar(baseUs, usMinActivo, usMax);
    if (b + maxPR * escalaPR > usMax) {
      b = usMax - maxPR * escalaPR;
      saturado = true;
    }
    if (b + minPR * escalaPR < usMinActivo) {
      b = usMinActivo - minPR * escalaPR;
      saturado = true;
    }

    // 3. Agregar Yaw solo dentro del margen restante
    float escalaYaw = 1.0f;
    for (int i = 0; i < 8; i++) {
      float mSinYaw = b + diffPR[i] * escalaPR;
      float dy = uYaw * MIX_YAW[i];
      if (mSinYaw + dy * escalaYaw > usMax && dy > 1e-3f) {
        escalaYaw = (usMax - mSinYaw) / dy;
      } else if (mSinYaw + dy * escalaYaw < usMinActivo && dy < -1e-3f) {
        escalaYaw = (usMinActivo - mSinYaw) / dy;
      }
    }
    if (escalaYaw < 0.0f) escalaYaw = 0.0f;
    if (escalaYaw < 0.9999f) saturado = true;

    for (int i = 0; i < 8; i++) {
      float u = b + diffPR[i] * escalaPR + uYaw * MIX_YAW[i] * escalaYaw;
      salUs[i] = limitar(u, usMinActivo, usMax);
    }
    return saturado;
  }
};

}  // namespace octo
