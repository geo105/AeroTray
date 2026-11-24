// fuzzy_bp.h - nucleo de control del ball & plate (sin dependencias de Arduino)
// Lo usan el sketch del ESP32 y la simulacion de PC, asi lo que se simula es lo que corre.
//
// Solo usa la lectura de la pantalla y el angulo que se le manda a los servos (ningun otro sensor).
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
// Etiquetas  NG  NM  NP  CE  PP  PM  PG   (negativo grande ... positivo grande)
// Funciones de pertenencia triangulares que se cruzan al 50 % (suman 1); fuera de [-1, 1] satura.
// Centros mas juntos cerca de cero: mas ganancia cerca del centro (vence el roce de la membrana)
// y menos lejos (no satura ni se pasa cuando la bola esta lejos).
static const float CENTRO_ENT[7] = {-1.0f, -0.55f, -0.2f, 0.0f, 0.2f, 0.55f, 1.0f};
// Consecuentes (singletons) de cada nivel de salida NG..PG
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

// Base de reglas (Mac Vicar-Whelan): nivel = error + velocidad, saturado.
// Filas = error (NG..PG), columnas = velocidad (NG..PG). Salida positiva = empujar la bola hacia -.
//            v: NG  NM  NP  CE  PP  PM  PG
//   e = NG   [ NG  NG  NG  NG  NM  NP  CE ]
//   e = NM   [ NG  NG  NG  NM  NP  CE  PP ]
//   e = NP   [ NG  NG  NM  NP  CE  PP  PM ]
//   e = CE   [ NG  NM  NP  CE  PP  PM  PG ]
//   e = PP   [ NM  NP  CE  PP  PM  PG  PG ]
//   e = PM   [ NP  CE  PP  PM  PG  PG  PG ]
//   e = PG   [ CE  PP  PM  PG  PG  PG  PG ]
// Lectura: bola a la derecha (PG) que ya vuelve rapido (NG) -> CE: no hacer nada, que llegue sola.
//          bola a la derecha que se sigue yendo (PG, PG) -> PG: inclinar fuerte hacia el centro.
inline int regla(int i, int j) { int k = (i - 3) + (j - 3); return k < -3 ? -3 : (k > 3 ? 3 : k); }

// en, vn: error y velocidad normalizados. Devuelve el esfuerzo en [-1, 1].
inline float fuzzyPD(float en, float vn) {
  float me[7], mv[7];
  fuzzificar(en, me); fuzzificar(vn, mv);
  float num = 0, den = 0;
  for (int i = 0; i < 7; i++) {
    if (me[i] <= 0) continue;
    for (int j = 0; j < 7; j++) {
      if (mv[j] <= 0) continue;
      float w = me[i] * mv[j];                 // AND = producto
      num += w * NIVEL_SAL[regla(i, j) + 3];
      den += w;
    }
  }
  return den > 0 ? num / den : 0;              // promedio ponderado (Sugeno)
}

// ======================= OBSERVADOR alfa-beta-gamma =======================
// Modelo: x'' = a_aplicada + d,   d = perturbacion desconocida (cambia lento)
// Ganancias por ubicacion de polos: polo doble en wxv (posicion/velocidad) y polo en wd (d).
// Se recalculan con el dt real, asi funciona igual a 20 ms que a 40 ms.
struct Observador {
  float x = 0, v = 0, d = 0;
  bool listo = false;
  float wxv = 11.0f, wd = 3.0f;      // rad/s (afinado en la simulacion solo con la pantalla: sim/sim.cpp)
  float dmax = 1.0f;                 // |d| maximo [m/s2]
  float saltoMax = 0.03f;            // lectura que salta mas que esto respecto de lo previsto: se descarta
  float rD = 0.005f;                 // para d, el residuo se recorta a +-5 mm (una lectura mala no la dispara)
  unsigned char rechazos = 0;
  unsigned long nRechazos = 0;
  float zRech = 0, dtRech = 0;       // primera lectura descartada y tiempo desde ella

  void reiniciar() { listo = false; x = v = d = 0; rechazos = 0; }

  void predecir(float a, float dt) {
    if (!listo) return;
    float at = a + d;
    x += v * dt + 0.5f * at * dt * dt;
    v += at * dt;
  }
  // devuelve false si la lectura se descarto por salto
  bool corregir(float z, float dt) {
    if (!listo) { x = z; v = 0; listo = true; rechazos = 0; return true; }
    float r = z - x;
    if (rechazos > 0) dtRech += dt;
    if (fabsf(r) > saltoMax && rechazos < 2) {
      if (rechazos == 0) { zRech = z; dtRech = 0; }
      rechazos++; nRechazos++; return false;
    }
    if (rechazos >= 2) {                     // tres lecturas seguidas lejos de lo previsto: la bola de verdad
      v = dtRech > 0 ? (z - zRech) / dtRech : 0;   // esta ahi; se reinicia con la velocidad medida entre ellas
      x = z; rechazos = 0; return true;
    }
    rechazos = 0;
    // (z - p1)^2 (z - p2) = z^3 + c2 z^2 + c1 z + c0
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
struct Escalas {           // del fuzzy
  float E = 0.05f;         // error que ya es "grande" [m]
  float V = 0.20f;         // velocidad que ya es "grande" [m/s]
  float A = 0.90f;         // aceleracion maxima que se le pide a la bola [m/s2] (~17 grados de servo)
};

struct Eje {
  Observador ob;
  // planta y servo
  float centro = 80;       // grados de servo con la placa horizontal
  float signo = 1;         // +1: subir el angulo del servo acelera la bola hacia +
  float K = 0.0533f;       // m/s2 por grado de servo (bandeja v5 0,436 grados/grado, bola maciza)
  float smax = 22;         // grados de servo alrededor del centro
  float tauServo = 0.06f;  // retardo equivalente servo + mecanismo [s] (primer orden)
  float retardo = 0;       // tiempo muerto puro entre el comando y la aceleracion de la bola [s]
                           // (medido en el banco: ~0,16 s en X y ~0,20 s en Y). Con retardo > 0 el control
                           // actua sobre el estado PREDICHO al final del retardo (predictor tipo Smith)
  bool usarObs = true;     // cancelar la perturbacion estimada (accion integral)
  // estado
  static const int NBUF = 32;   // hasta 31 pasos de retardo (0,62 s a 20 ms)
  float hist[NBUF];        // aceleraciones de control mandadas, una por paso
  int ih = 0;
  float aPedida = 0;       // aceleracion de control mandada en el ultimo paso (ya saturada)
  float aAplicada = 0;     // la que llega a la bola (con el retardo del servo)
  float servo = 80;        // ultimo comando [grados]
  float esfuerzo = 0;      // salida del fuzzy [-1, 1] (telemetria)
  float xPred = 0, vPred = 0;   // estado predicho al final del retardo (telemetria)

  Eje() { for (int i = 0; i < NBUF; i++) hist[i] = 0; }
  void reiniciar() {
    ob.reiniciar(); aPedida = aAplicada = 0; servo = centro; esfuerzo = 0;
    for (int i = 0; i < NBUF; i++) hist[i] = 0;
    ih = 0;
  }
  // comando mandado hace j pasos (j = 1: el del paso anterior)
  float mandado(int j) const { return hist[(ih - j + 2 * NBUF) % NBUF]; }

  // Un paso de control. hayMedida = false durante un corte del panel: solo se predice.
  // sp, vsp, asp: referencia de posicion, velocidad y aceleracion (trayectoria).
  float paso(const Escalas &s, bool hayMedida, float z, float dt, float sp, float vsp, float asp) {
    int D = (int)(retardo / dt + 0.5f);
    if (D > NBUF - 2) D = NBUF - 2;
    if (D < 0) D = 0;
    // en el ultimo intervalo llego a la bola el comando de hace D + 1 pasos
    float aLlega = mandado(D + 1);
    float fil = dt / (tauServo + dt);
    aAplicada += (aLlega - aAplicada) * fil;
    ob.dmax = 0.8f * K * smax;
    ob.predecir(aAplicada, dt);
    if (hayMedida) ob.corregir(z, dt);

    // prediccion hasta que llegue el comando de ahora: aplica los D comandos que ya estan en camino
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
