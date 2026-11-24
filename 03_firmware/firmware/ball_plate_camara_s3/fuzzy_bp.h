// fuzzy_bp.h - nucleo de control del ball & plate (sin dependencias de Arduino)
// Lo usan el sketch del ESP32 / ESP32-S3-CAM y la simulacion de PC, asi lo que se simula es lo que corre.
//
// Usa la lectura de posicion (pantalla o camara OV2640) y el angulo que se le manda a los servos.
//   1) Observador alfa-beta-gamma con la entrada conocida (el angulo mandado a los servos):
//        estima posicion x, velocidad v y la PERTURBACION d (aceleracion que nadie mando:
//        inclinacion del dron, centro mal nivelado, roce, viento). Hace de accion integral.
//   2) Fuzzy PD tipo Sugeno de orden cero, 7 x 7 reglas, entradas: error y velocidad.
//      Sale la aceleracion que se le pide a la bola.
//   3) a_pedida = a_ref - Amax * fuzzy(e, v) - d_estimada
//      y se pasa a grados de servo con la ganancia de la planta K [m/s2 por grado de servo].
//
// Unidades: metros, segundos, grados de servo.
#pragma once
#include <math.h>

namespace bp {

inline float limitar(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

// ======================= FUZZY PD (Sugeno 0, 7x7) =======================
static const float CENTRO_ENT[7] = {-1.0f, -0.55f, -0.2f, 0.0f, 0.2f, 0.55f, 1.0f};
static const float NIVEL_SAL[7]  = {-1.0f, -0.62f, -0.28f, 0.0f, 0.28f, 0.62f, 1.0f};

inline void fuzzificar(float x, float mu[7]) {
  for (int i = 0; i < 7; i++) mu[i] = 0;
  x = limitar(x, -1.0f, 1.0f);
  for (int i = 0; i < 6; i++) {
    if (x <= CENTRO_ENT[i + 1]) {
      float t = (x - CENTRO_ENT[i]) / (CENTRO_ENT[i + 1] - CENTRO_ENT[i]);
      mu[i] = 1.0f - t; mu[i + 1] = t;
      return;
    }
  }
  mu[6] = 1.0f;
}

inline int regla(int i, int j) { int k = (i - 3) + (j - 3); return k < -3 ? -3 : (k > 3 ? 3 : k); }

inline float fuzzyPD(float en, float vn) {
  float me[7], mv[7];
  fuzzificar(en, me); fuzzificar(vn, mv);
  float num = 0, den = 0;
  for (int i = 0; i < 7; i++) {
    if (me[i] <= 0) continue;
    for (int j = 0; j < 7; j++) {
      if (mv[j] <= 0) continue;
      float w = me[i] * mv[j];
      num += w * NIVEL_SAL[regla(i, j) + 3];
      den += w;
    }
  }
  return den > 0 ? num / den : 0;
}

// ======================= OBSERVADOR alfa-beta-gamma =======================
struct Observador {
  float x = 0, v = 0, d = 0;
  bool listo = false;
  float wxv = 11.0f, wd = 3.0f;
  float dmax = 1.0f;
  float saltoMax = 0.035f;           // 3.5 cm para camara
  float rD = 0.005f;
  unsigned char rechazos = 0;
  unsigned long nRechazos = 0;
  float zRech = 0, dtRech = 0;

  void reiniciar() { listo = false; x = v = d = 0; rechazos = 0; }

  void predecir(float a, float dt) {
    if (!listo) return;
    float at = a + d;
    x += v * dt + 0.5f * at * dt * dt;
    v += at * dt;
  }

  bool corregir(float z, float dt) {
    if (!listo) { x = z; v = 0; listo = true; rechazos = 0; return true; }
    float r = z - x;
    if (rechazos > 0) dtRech += dt;
    if (fabsf(r) > saltoMax && rechazos < 2) {
      if (rechazos == 0) { zRech = z; dtRech = 0; }
      rechazos++; nRechazos++; return false;
    }
    if (rechazos >= 2) {
      v = dtRech > 0 ? (z - zRech) / dtRech : 0;
      x = z; rechazos = 0; return true;
    }
    rechazos = 0;
    float p1 = expf(-wxv * dt), p2 = expf(-wd * dt);
    float c2 = -(2 * p1 + p2), c1 = p1 * p1 + 2 * p1 * p2, c0 = -p1 * p1 * p2;
    float al = 1 + c0, ga = (c2 + c1 + al) * 0.5f, be = (c2 + 3 - al) - ga;
    x += al * r;
    v += (be / dt) * r;
    d = limitar(d + (2 * ga / (dt * dt)) * limitar(r, -rD, rD), -dmax, dmax);
    return true;
  }
};

// ======================= UN EJE =======================
struct Escalas {
  float E = 0.05f;         // error "grande" [m]
  float V = 0.20f;         // velocidad "grande" [m/s]
  float A = 0.90f;         // aceleracion maxima [m/s2]
};

struct Eje {
  Observador ob;
  float centro = 80;
  float signo = 1;
  float K = 0.0533f;
  float smax = 22;
  float tauServo = 0.06f;
  float retardo = 0.06f;   // por defecto 60 ms (latencia camara + servo)
  bool usarObs = true;
  static const int NBUF = 32;
  float hist[NBUF];
  int ih = 0;
  float aPedida = 0;
  float aAplicada = 0;
  float servo = 80;
  float esfuerzo = 0;
  float xPred = 0, vPred = 0;

  Eje() { for (int i = 0; i < NBUF; i++) hist[i] = 0; }
  void reiniciar() {
    ob.reiniciar(); aPedida = aAplicada = 0; servo = centro; esfuerzo = 0;
    for (int i = 0; i < NBUF; i++) hist[i] = 0;
    ih = 0;
  }
  float mandado(int j) const { return hist[(ih - j + 2 * NBUF) % NBUF]; }

  float paso(const Escalas &s, bool hayMedida, float z, float dt, float sp, float vsp, float asp) {
    int D = (int)(retardo / dt + 0.5f);
    if (D > NBUF - 2) D = NBUF - 2;
    if (D < 0) D = 0;
    float aLlega = mandado(D + 1);
    float fil = dt / (tauServo + dt);
    aAplicada += (aLlega - aAplicada) * fil;
    ob.dmax = 0.8f * K * smax;
    ob.predecir(aAplicada, dt);
    if (hayMedida) ob.corregir(z, dt);

    float xf = ob.x, vf = ob.v, af = aAplicada;
    for (int i = D; i >= 1; i--) {
      af += (mandado(i) - af) * fil;
      float at = af + ob.d;
      xf += vf * dt + 0.5f * at * dt * dt;
      vf += at * dt;
    }
    xPred = xf; vPred = vf;

    esfuerzo = fuzzyPD((xf - sp) / s.E, (vf - vsp) / s.V);
    float a = asp - s.A * esfuerzo;
    if (usarObs) a -= ob.d;

    float du = limitar(a / (signo * K), -smax, smax);
    servo = centro + du;
    aPedida = signo * K * du;
    hist[ih] = aPedida; ih = (ih + 1) % NBUF;
    return servo;
  }
};

}  // namespace bp
