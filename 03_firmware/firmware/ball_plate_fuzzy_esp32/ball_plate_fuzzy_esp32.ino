/*
  BALL & PLATE - CONTROL FUZZY CON RECHAZO DE PERTURBACIONES
  ESP32 NodeMCU-32S (galleta del AeroTray) + panel resistivo de 4 hilos + 2 servos.
  Usa SOLO la lectura de la pantalla y los servos (la IMU de la base no se toca).
  Bandeja v5 (brazo 25T de 20 mm, horquilla a 45,85 mm: 0,436 grados de placa por grado de servo)

  QUE HACE (el nucleo esta en fuzzy_bp.h, el mismo que se simula en la PC)
    1. Lee la bola en el panel (doble carga) y descarta lecturas malas (sin bola, fuera de la zona
       calibrada, saltos imposibles). Si se pierde la bola un momento (vibracion), sigue con la prediccion.
    2. Observador alfa-beta-gamma: con la posicion medida y el angulo que se mando a los servos estima
       posicion, velocidad y la PERTURBACION d (aceleracion que nadie mando: dron inclinado, centro mal
       nivelado, roce). Es solo calculo, no otro sensor. Cancelar d hace de accion integral.
    3. Fuzzy PD (Sugeno, 7x7 reglas, entradas error y velocidad) -> aceleracion pedida a la bola.
    4. a_pedida = a_ref - Amax*fuzzy - d   ->   servo = centro + a_pedida / (signo * K)

  CABLEADO (igual que los otros sketches de la galleta)
    Panel: X+ GPIO32  Y+ GPIO33  X- GPIO13  Y- GPIO4      Servos: X GPIO18  Y GPIO16
    ESC 25 26 27 14 quedan en LOW (no se arman). Monitor serie 115200, "Nueva linea". QUITA LAS HELICES.

  PUESTA EN MARCHA (detalle en LEEME.md)
    1. panel                    (bola quieta en el centro) -> la doble carga tiene que verla ~40/40
    2. ver                      -> calibracion y tiempos
    3. cal ... cal fin          (solo si ver dice POR DEFECTO; es la misma zona que los otros sketches)
    4. nivel                    sostienes la bola en el centro y la sueltas 5 a 9 veces: saca solo los
                                centros (placa horizontal) y el signo de cada servo
                                (a mano: cx / cy y prueba x 6 / prueba y 6)
    6. iniciar                  -> pon la bola; arranca sola tras 5 lecturas buenas
    7. circulo 2 4 + centrar    (con el control andando) corrige cx/cy solo
    8. stop + guardar

  CSV con el control andando o esperando (p 1 = 25 Hz, p 2 = 50 Hz, p 0 = nada):
    t_ms,toque,x,y,xe,ye,vx,vy,spx,spy,servoX,servoY,dX,dY,estado
    x,y medido [cm] (repite el ultimo si toque=0) | xe,ye estimado [cm] | v [cm/s] | servo [grados]
    d = perturbacion estimada [cm/s2] | estado 0 detenido, 1 esperando, 2 activo
*/
#include <Arduino.h>
#include <EEPROM.h>
#include "fuzzy_bp.h"

// ===================== PINES =====================
const uint8_t XP = 32, YP = 33, XM = 13, YM = 4;
uint8_t PIN_SX = 18, PIN_SY = 16;
const uint8_t PIN_ESC[4] = {25, 26, 27, 14};

// ===================== PANEL =====================
// crudos 0..1023 de los bordes medibles (comando cal). Por defecto: la ultima calibracion conocida.
int X_INI = 138, X_FIN = 807, Y_INI = 155, Y_FIN = 861;
const float LX = 0.172f, LY = 0.1295f;     // distancia real entre esos bordes [m] (area activa)
const int MUESTRAS = 6;                     // promedio por lectura
bool panelCal = false;
// Metodo de lectura. 2 = DOBLE CARGA (por defecto): la capa que mide se carga a 3,3 V y luego a 0 V y
// se suelta; con la bola apoyada se va al voltaje del punto de contacto aunque el contacto sea de
// cientos de kohm; sin bola se queda en 3,3 o en 0 (fuera de la calibracion). No depende del umbral
// digital del ESP32. 1 = toque digital con pull-up (v4.3): solo ve contactos de menos de ~15 kohm.
uint8_t metodo = 2;
uint16_t asentarUs = 500;                   // espera para que la capa suelta llegue al voltaje del contacto
const float K_MAX = 0.45f;                  // fraccion sin asentar admitida (0 = contacto perfecto)
float kCont[2] = {1, 1};                    // calidad del ultimo contacto por eje
int mvHi[2] = {0, 0}, mvLo[2] = {0, 0};

// ===================== SERVOS =====================
// Misma escala que ESP32Servo con attach(pin, 500, 2500): los centros del sketch PID valen igual.
const float US_MIN = 500, US_MAX = 2500;
const int FREQ_SERVO = 50, RES_SERVO = 16;
const int CANAL[2] = {4, 5};                // solo Arduino-ESP32 2.x
int pinConectado[2] = {-1, -1};
float servoAct[2] = {80, 78};
const float SMAX_ABS = 30;                  // +-30 grados de servo = +-13 grados de placa (CAD: +-15 en los dos ejes)

// ===================== CONTROL =====================
const float GRAV = 9.81f, FACTOR_BOLA = 5.0f / 7.0f;  // esfera maciza (si fuera hueca: 3/5)
const float GANANCIA_MEC = 0.436f;                     // grados de placa por grado de servo
const float K_TEORICO = FACTOR_BOLA * GRAV * GANANCIA_MEC * PI / 180.0f;   // 0,0533 m/s2 por grado

bp::Eje ej[2];
bp::Escalas esc;
bool signoOk[2] = {false, false};
uint32_t periodoUs = 20000;
float tPerdida = 0.20f;                     // s sin ver la bola -> se detiene

enum { DETENIDO = 0, ESPERANDO = 1, ACTIVO = 2 };
uint8_t estado = DETENIDO, ejes = 3, buenas = 0;
const char *motivo = "inicio";
float tSinBola = 0;
uint32_t tUltValida = 0;

bool centrando = false; float tCentrar = 0, tCentrarObj = 4, sumD[2] = {0, 0};
float spX = 0, spY = 0, circR = 0, circT = 5;
uint32_t circT0 = 0;

// ===================== DIAGNOSTICO =====================
uint32_t usCtrlMax = 0, usServoMax = 0, usSerieMax = 0, usLecturaMax = 0;
uint32_t usLectura = 0, usPeriodo = 0, usPeriodoMax = 0, cortes = 0, invalidas = 0, nPaso = 0;
uint8_t imprimir = 1;
int rawX = -1, rawY = -1;
bool toqueAct = false, validaAct = false;
float posX = 0, posY = 0;
bool calibrando = false;
int cMinX, cMaxX, cMinY, cMaxY, cN;
uint32_t proximo = 0, ultimo = 0;

// ===================== EEPROM =====================
const int TAM_EEPROM = 1024;                // igual que los otros sketches (0..: v4.3, 900: comun, 1000: ident)
const int DIR_HW = 900; const uint16_t MAGICO_HW = 0x4857;
struct Hardware { uint16_t magico; int xi, xf, yi, yf; uint8_t px, py; };
const int DIR_FZ = 600; const uint16_t MAGICO_FZ = 0xF2B8;
struct ConfigFZ {
  uint16_t magico;
  float centro[2], signo[2]; uint8_t sOk[2];
  float K[2], smax, E, V, A, wxv, wd;
  uint8_t obs;
  uint8_t metodo; uint16_t asentar; float perdida;
};
// bloque aparte (asi no se pierde la config guardada con versiones anteriores)
const int DIR_RET = 700; const uint16_t MAGICO_RET = 0x5245;
struct ConfigRet { uint16_t magico; float retardo[2], tau; };
static_assert(DIR_FZ + sizeof(ConfigFZ) <= DIR_RET && DIR_RET + sizeof(ConfigRet) <= DIR_HW, "EEPROM: bloques solapados");

void guardarHW() {
  Hardware h = {MAGICO_HW, X_INI, X_FIN, Y_INI, Y_FIN, PIN_SX, PIN_SY};
  EEPROM.put(DIR_HW, h); EEPROM.commit();
}
void cargarHW() {
  Hardware h; EEPROM.get(DIR_HW, h);
  if (h.magico != MAGICO_HW) { Serial.println(F("# panel: sin calibracion guardada (uso la ultima conocida; mejor: cal)")); return; }
  if (h.xi >= 0 && h.xf <= 1023 && h.yi >= 0 && h.yf <= 1023 && h.xf - h.xi >= 200 && h.yf - h.yi >= 200) {
    X_INI = h.xi; X_FIN = h.xf; Y_INI = h.yi; Y_FIN = h.yf; panelCal = true;
  }
  if ((h.px == 18 && h.py == 16) || (h.px == 16 && h.py == 18)) { PIN_SX = h.px; PIN_SY = h.py; }
  Serial.printf("# panel: X %d..%d  Y %d..%d (zona comun)  servo X en GPIO%u\n", X_INI, X_FIN, Y_INI, Y_FIN, PIN_SX);
}
void guardar() {
  ConfigFZ c; c.magico = MAGICO_FZ;
  for (int k = 0; k < 2; k++) {
    c.centro[k] = ej[k].centro; c.signo[k] = ej[k].signo; c.sOk[k] = signoOk[k];
  }
  c.K[0] = ej[0].K; c.K[1] = ej[1].K; c.smax = ej[0].smax; c.E = esc.E; c.V = esc.V; c.A = esc.A;
  c.wxv = ej[0].ob.wxv; c.wd = ej[0].ob.wd; c.obs = ej[0].usarObs;
  c.metodo = metodo; c.asentar = asentarUs; c.perdida = tPerdida;
  EEPROM.put(DIR_FZ, c);
  ConfigRet r = {MAGICO_RET, {ej[0].retardo, ej[1].retardo}, ej[0].tauServo};
  EEPROM.put(DIR_RET, r); EEPROM.commit();
  guardarHW();
  Serial.println(F("# guardado"));
}
void cargar() {
  ConfigRet r = {}; EEPROM.get(DIR_RET, r);
  if (r.magico == MAGICO_RET)
    for (int k = 0; k < 2; k++) {
      ej[k].retardo = (r.retardo[k] >= 0 && r.retardo[k] <= 0.5f) ? r.retardo[k] : 0;
      ej[k].tauServo = (r.tau >= 0.005f && r.tau <= 0.2f) ? r.tau : 0.03f;
    }
  ConfigFZ c; EEPROM.get(DIR_FZ, c);
  if (c.magico != MAGICO_FZ) { Serial.println(F("# config: valores por defecto (nunca se guardo)")); return; }
  for (int k = 0; k < 2; k++) {
    ej[k].centro = constrain(c.centro[k], 30.0f, 150.0f); ej[k].signo = c.signo[k] < 0 ? -1 : 1; signoOk[k] = c.sOk[k];
    ej[k].K = constrain(c.K[k], 0.01f, 0.3f); ej[k].smax = constrain(c.smax, 3.0f, SMAX_ABS);
    ej[k].ob.wxv = constrain(c.wxv, 4.0f, 30.0f); ej[k].ob.wd = constrain(c.wd, 0.0f, 15.0f);
    ej[k].usarObs = c.obs;
  }
  esc.E = constrain(c.E, 0.01f, 0.2f); esc.V = constrain(c.V, 0.03f, 1.0f); esc.A = constrain(c.A, 0.1f, 1.5f);
  metodo = c.metodo == 1 ? 1 : 2; asentarUs = constrain(c.asentar, 50, 2000); tPerdida = constrain(c.perdida, 0.1f, 0.6f);
  Serial.println(F("# config cargada"));
}

// ===================== PANEL (rutinas probadas en ball_plate_esp32 v4.3) =====================
void soltar() { pinMode(XP, INPUT); pinMode(XM, INPUT); pinMode(YP, INPUT); pinMode(YM, INPUT); }
int leer1023(uint8_t pin) { long mv = analogReadMilliVolts(pin); return (int)min(mv * 1023L / 3300L, 1023L); }
int prom(uint8_t pin) { leer1023(pin); long s = 0; for (int i = 0; i < MUESTRAS; i++) s += leer1023(pin); return s / MUESTRAS; }
// toque: X+ y X- a GND, Y+ con pull-up; si la bola apoya, la capa Y baja. Voto 2 de 3.
bool tocaPanel() {
  soltar(); pinMode(XP, OUTPUT); digitalWrite(XP, LOW); pinMode(XM, OUTPUT); digitalWrite(XM, LOW);
  pinMode(YP, INPUT_PULLUP);
  delayMicroseconds(350);
  uint8_t bajos = 0;
  for (uint8_t i = 0; i < 3; i++) { if (digitalRead(YP) == LOW) bajos++; if (i < 2) delayMicroseconds(100); }
  soltar();
  return bajos >= 2;
}
int leerX() {
  soltar(); pinMode(XP, OUTPUT); digitalWrite(XP, HIGH); pinMode(XM, OUTPUT); digitalWrite(XM, LOW);
  delayMicroseconds(60); int v = prom(YP); soltar(); return v;
}
int leerY() {
  soltar(); pinMode(YP, OUTPUT); digitalWrite(YP, HIGH); pinMode(YM, OUTPUT); digitalWrite(YM, LOW);
  delayMicroseconds(60); int v = prom(XP); soltar(); return v;
}
// toque antes y despues: si la bola rebota a mitad de la lectura, se descarta
bool leerPanel(int &rx, int &ry) {
  if (!tocaPanel()) return false;
  rx = leerX(); ry = leerY();
  return tocaPanel();
}
// ---- metodo 2: doble carga ----
// Excita una capa (ex1 = 3,3 V, ex0 = 0 V), precarga la otra (s1, s2) a 'nivel', la suelta, espera y mide en s1.
float leerCapa(uint8_t ex1, uint8_t ex0, uint8_t s1, uint8_t s2, uint8_t nivel) {
  soltar();
  pinMode(ex1, OUTPUT); digitalWrite(ex1, HIGH); pinMode(ex0, OUTPUT); digitalWrite(ex0, LOW);
  pinMode(s1, OUTPUT); digitalWrite(s1, nivel); pinMode(s2, OUTPUT); digitalWrite(s2, nivel);
  delayMicroseconds(15);
  pinMode(s2, INPUT); pinMode(s1, INPUT);
  delayMicroseconds(asentarUs);
  analogReadMilliVolts(s1);
  long suma = 0;
  for (int i = 0; i < MUESTRAS; i++) suma += analogReadMilliVolts(s1);
  soltar();
  return (float)suma / MUESTRAS;
}
// Devuelve el crudo 0..1023 del eje, o -1 si no hay bola. Con la carga desde arriba (hi) y desde abajo
// (lo), si el contacto es lento ninguna llega del todo, pero las dos se acercan igual:
//   hi = V + (3300 - V) k,  lo = V (1 - k)   ->   k = (hi - lo) / 3300,  V = lo / (1 - k)
int leerEje(uint8_t ex1, uint8_t ex0, uint8_t s1, uint8_t s2, uint8_t k) {
  float hi = leerCapa(ex1, ex0, s1, s2, HIGH), lo = leerCapa(ex1, ex0, s1, s2, LOW);
  mvHi[k] = (int)hi; mvLo[k] = (int)lo;
  if (hi > 3000 || lo < 250) { kCont[k] = 1; return -1; }       // se quedo en la precarga: sin bola
  float kk = (hi - lo) / 3300.0f;
  kCont[k] = kk;
  if (kk > K_MAX) return -1;                                      // contacto demasiado debil
  if (kk < 0) kk = 0;
  return (int)(lo / (1 - kk) * 1023.0f / 3300.0f + 0.5f);
}
bool leerPanel2(int &rx, int &ry) {
  rx = leerEje(XP, XM, YP, YM, 0);          // X: capa X excitada, se mide en la capa Y (Y+)
  if (rx < 0) { ry = -1; return false; }
  ry = leerEje(YP, YM, XP, XM, 1);          // Y: capa Y excitada, se mide en la capa X (X+)
  return ry >= 0;
}
// dentro de la zona calibrada (con 3 % de tolerancia). Fuera = franja ciega o lectura basura.
bool dentroCal(int rx, int ry) {
  float tx = 0.03f * (X_FIN - X_INI), ty = 0.03f * (Y_FIN - Y_INI);
  return rx > X_INI - tx && rx < X_FIN + tx && ry > Y_INI - ty && ry < Y_FIN + ty;
}
float aMetros(int raw, int ini, int fin, float L) { return ((float)(raw - ini) / (float)(fin - ini) - 0.5f) * L; }

// lectura completa con validacion; deja todo en rawX/rawY/posX/posY
bool medir() {
  int rx, ry;
  toqueAct = metodo == 1 ? leerPanel(rx, ry) : leerPanel2(rx, ry);
  validaAct = false;
  if (toqueAct) {
    rawX = rx; rawY = ry;
    if (dentroCal(rx, ry)) {
      posX = aMetros(rx, X_INI, X_FIN, LX); posY = aMetros(ry, Y_INI, Y_FIN, LY);
      validaAct = true; tUltValida = millis();
    }
  }
  return validaAct;
}

// ===================== SERVOS (LEDC, Arduino-ESP32 2.x y 3.x) =====================
void servoUs(uint8_t k, float us) {
  uint32_t duty = (uint32_t)(us * ((1UL << RES_SERVO) - 1) / (1000000.0f / FREQ_SERVO));
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  if (pinConectado[k] >= 0) ledcWrite(pinConectado[k], duty);
#else
  ledcWrite(CANAL[k], duty);
#endif
}
// En Arduino-ESP32 3.3 (IDF 5.5) ledcWrite espera a que el PWM tome el valor nuevo: hasta 20 ms por
// servo a 50 Hz. Medido en el banco: 39 ms por ciclo solo en escribir los dos servos, el control
// quedaba a 40 ms. Por eso cada servo se escribe desde su propia tarea: el lazo solo deja el valor.
#if defined(ARDUINO_ARCH_ESP32)
volatile float servoObj[2] = {80, 78};
volatile bool pausaServos = true;
void tareaServo(void *p) {
  uint8_t k = (uint8_t)(uintptr_t)p;
  float ult = -1000;
  for (;;) {
    float u = servoObj[k];
    if (!pausaServos && u != ult) { servoUs(k, US_MIN + u * (US_MAX - US_MIN) / 180.0f); ult = u; }
    // SIEMPRE ceder 1 ms: ledcWrite espera girando (no duerme) hasta el inicio del proximo pulso; si el
    // control cambia el angulo en cada paso y la tarea no cede, la tarea IDLE del nucleo 0 no corre y el
    // watchdog reinicia el ESP32 a los 5 s (banco 27-oct 15:19, con el control andando suave)
    vTaskDelay(1);
  }
}
void iniciarTareasServo() {
  xTaskCreatePinnedToCore(tareaServo, "servoX", 2048, (void *)0, 1, NULL, 0);
  xTaskCreatePinnedToCore(tareaServo, "servoY", 2048, (void *)1, 1, NULL, 0);
}
#endif
void escribir(uint8_t k, float deg) {
  deg = constrain(deg, ej[k].centro - SMAX_ABS, ej[k].centro + SMAX_ABS);
  deg = constrain(deg, 0.0f, 180.0f);
  servoAct[k] = deg;
#if defined(ARDUINO_ARCH_ESP32)
  servoObj[k] = deg;
#else
  servoUs(k, US_MIN + deg * (US_MAX - US_MIN) / 180.0f);
#endif
}
void conectarServos() {
#if defined(ARDUINO_ARCH_ESP32)
  bool estaba = pausaServos;
  pausaServos = true; delay(45);                     // que ninguna tarea este escribiendo
#endif
  uint8_t pin[2] = {PIN_SX, PIN_SY};
  for (uint8_t k = 0; k < 2; k++) {
    if (pinConectado[k] < 0) continue;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcDetach(pinConectado[k]);
#else
    ledcDetachPin(pinConectado[k]);
#endif
  }
  for (uint8_t k = 0; k < 2; k++) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pin[k], FREQ_SERVO, RES_SERVO);
#else
    ledcSetup(CANAL[k], FREQ_SERVO, RES_SERVO);
    ledcAttachPin(pin[k], CANAL[k]);
#endif
    pinConectado[k] = pin[k];
  }
#if defined(ARDUINO_ARCH_ESP32)
  pausaServos = estaba;
#endif
}
void nivelar() { escribir(0, ej[0].centro); escribir(1, ej[1].centro); }

// ===================== ESTADOS =====================
void detener(const char *razon) {
  centrando = false;
  estado = DETENIDO; motivo = razon; buenas = 0; tSinBola = 0;
  ej[0].reiniciar(); ej[1].reiniciar();
  nivelar();
  Serial.print(F("# DETENIDO: ")); Serial.println(razon);
}
void arrancar(float zx, float zy, float dt) {
  float z[2] = {zx, zy};
  for (int k = 0; k < 2; k++) {
    ej[k].reiniciar();
    ej[k].ob.corregir(z[k], dt);
  }
  estado = ACTIVO; tSinBola = 0; usPeriodoMax = 0; cortes = 0; circT0 = millis();
  usCtrlMax = usServoMax = usSerieMax = usLecturaMax = 0;
  Serial.printf("# CONTROL ACTIVO (%s)%s\n", ejes == 3 ? "xy" : (ejes == 1 ? "x" : "y"),
                ej[0].usarObs ? " + observador" : "");
}
void referencia(float sp[2], float vsp[2], float asp[2]) {
  sp[0] = spX; sp[1] = spY; vsp[0] = vsp[1] = asp[0] = asp[1] = 0;
  if (circR <= 0) return;
  float t = (millis() - circT0) * 1e-3f, w = 2 * PI / circT, r = circR * min(1.0f, t / 1.0f);
  sp[0] += r * sinf(w * t);           vsp[0] = r * w * cosf(w * t); asp[0] = -r * w * w * sinf(w * t);
  sp[1] += -r * cosf(w * t);          vsp[1] = r * w * sinf(w * t); asp[1] = r * w * w * cosf(w * t);
}

// ===================== PRUEBAS (con el control detenido) =====================
// manda +g grados al servo k durante 450 ms y compara donde quedo la bola con donde
// habria quedado sin el pulso (posicion + velocidad previas): sirve aunque la bola ya ruede lento
void prueba(uint8_t k, float g) {
  const char *n = k ? "y" : "x";
  nivelar(); delay(300);
  // antes: hasta 8 lecturas en ~0,6 s -> posicion y velocidad (recta por minimos cuadrados).
  // Se descartan las que caen a mas de 1 cm de la mediana (lecturas sueltas malas).
  float tt[8], px[8], py[8]; uint8_t m = 0; uint32_t ta = millis();
  for (uint8_t i = 0; i < 30 && m < 8; i++) {
    if (medir()) { tt[m] = (millis() - ta) * 1e-3f; px[m] = posX; py[m] = posY; m++; }
    delay(20);
  }
  if (m < 5) {
    Serial.printf("# no veo la bola (%u lecturas buenas de 30). Escribe  panel  con la bola en el centro\n", m);
    return;
  }
  float ox[8], oy[8];
  for (uint8_t i = 0; i < m; i++) { ox[i] = px[i]; oy[i] = py[i]; }
  for (uint8_t i = 1; i < m; i++) for (uint8_t j = i; j > 0 && ox[j] < ox[j - 1]; j--) { float a = ox[j]; ox[j] = ox[j - 1]; ox[j - 1] = a; }
  for (uint8_t i = 1; i < m; i++) for (uint8_t j = i; j > 0 && oy[j] < oy[j - 1]; j--) { float a = oy[j]; oy[j] = oy[j - 1]; oy[j - 1] = a; }
  float medX = ox[m / 2], medY = oy[m / 2];
  float st = 0, stt = 0, sp[2] = {0, 0}, stp[2] = {0, 0}; uint8_t n2 = 0;
  for (uint8_t i = 0; i < m; i++) {
    if (fabsf(px[i] - medX) > 0.01f || fabsf(py[i] - medY) > 0.01f) continue;
    float t = tt[i]; st += t; stt += t * t; sp[0] += px[i]; sp[1] += py[i]; stp[0] += t * px[i]; stp[1] += t * py[i]; n2++;
  }
  if (n2 < 4) { Serial.println(F("# lecturas muy dispersas: escribe  panel  con la bola quieta en el centro")); return; }
  m = n2;
  float den = m * stt - st * st, v0[2], p0[2];
  for (int i = 0; i < 2; i++) { v0[i] = (m * stp[i] - st * sp[i]) / den; p0[i] = (sp[i] - v0[i] * st) / m; }
  if (fabsf(v0[0]) > 0.05f || fabsf(v0[1]) > 0.05f) { Serial.println(F("# la bola rueda sola rapido: nivela primero (cx / cy) y repite")); return; }
  float tPulso = (millis() - ta) * 1e-3f;
  escribir(k, ej[k].centro + g);
  uint32_t t0 = millis(); bool vio = false; float p1[2] = {0, 0}, t1 = 0;
  while (millis() - t0 < 450) { if (medir()) { p1[0] = posX; p1[1] = posY; t1 = (millis() - ta) * 1e-3f; vio = true; } delay(15); }
  nivelar();
  if (!vio || t1 < tPulso + 0.25f) { Serial.println(F("# perdi la bola durante la prueba: recolocala y repite")); return; }
  float d[2];
  for (int i = 0; i < 2; i++) d[i] = (p1[i] - (p0[i] + v0[i] * t1)) * 100;   // efecto del pulso [cm]
  float dk = d[k], dO = d[1 - k];
  Serial.printf("# servo %s %+.0f grados: la bola se desvio %+.1f cm en %s y %+.1f cm en el otro eje\n", n, g, dk, n, dO);
  if (fabsf(dk) < 1.0f && fabsf(dO) < 1.0f) {
    Serial.println(F("# casi no se movio. Revisa: servos con alimentacion (bateria/buck), bielas sueltas,"));
    Serial.println(F("#   placa muy desnivelada. Repite con mas grados: prueba x 10"));
  } else if (min(fabsf(dk), fabsf(dO)) > 0.6f * max(fabsf(dk), fabsf(dO))) {
    Serial.println(F("# ambiguo: la bola se fue en diagonal. Nivela mejor (cx / cy) y repite"));
  } else if (fabsf(dO) > fabsf(dk)) {
    Serial.println(F("# ese servo mueve la bola en el OTRO eje: escribe  intercambiar  y repite las dos pruebas"));
  } else {
    ej[k].signo = ((dk > 0) == (g > 0)) ? 1 : -1; signoOk[k] = true;
    Serial.printf("# signo %s = %+.0f  (listo; 'guardar' para conservarlo)\n", n, ej[k].signo);
  }
}
// ===================== NIVELADO AUTOMATICO =====================
// El usuario sostiene la bola en el centro y la suelta. Se mide su aceleracion ajustando una
// parabola x(t) = x0 + v0 t + a t^2 / 2 a las lecturas (la velocidad con que la suelta no importa).
// Se repite con cada servo corrido +-D grados: de las 4 tiradas sale como responde la bola a cada
// servo (signo, ganancia K y si estan cruzados) y donde queda la placa horizontal.
const uint8_t N_TIRO = 160;
float tTiro[N_TIRO], xTiro[N_TIRO], yTiro[N_TIRO];

// coeficiente de t^2 por minimos cuadrados (t centrado para que no pierda precision)
float aceleracionParabola(const float *t, const float *x, uint8_t n) {
  double tm = 0; for (uint8_t i = 0; i < n; i++) tm += t[i]; tm /= n;
  double S[5] = {0, 0, 0, 0, 0}, B[3] = {0, 0, 0};
  for (uint8_t i = 0; i < n; i++) {
    double u = t[i] - tm, p = 1;
    for (int k = 0; k < 5; k++) { S[k] += p; if (k < 3) B[k] += p * x[i]; p *= u; }
  }
  // [S0 S1 S2; S1 S2 S3; S2 S3 S4] c = B  (Cramer)
  double M[3][3] = {{S[0], S[1], S[2]}, {S[1], S[2], S[3]}, {S[2], S[3], S[4]}};
  auto det3 = [](double m[3][3]) {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  };
  double D = det3(M);
  if (fabs(D) < 1e-18) return 0;
  double M2[3][3] = {{S[0], S[1], B[0]}, {S[1], S[2], B[1]}, {S[2], S[3], B[2]}};
  return (float)(2 * det3(M2) / D);
}
bool cancelado() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  Serial.println(F("# cancelado"));
  return true;
}
// una tirada: espera la bola sostenida quieta, pide soltarla y mide la aceleracion [m/s2].
// Devuelve 1 = medida, 0 = repetir, -1 = cancelado.
int medirSoltada(float a[2], const char *etiqueta) {
  Serial.printf("# %s: pon la bola en el centro y SOSTENLA quieta con el dedo\n", etiqueta);
  // 1) 0,4 s sostenida quieta (+-6 mm): referencia = promedio de esas lecturas
  double sx = 0, sy = 0; uint16_t m = 0; uint32_t tq = millis(), t0 = millis();
  float ref[2] = {0, 0};
  while (true) {
    if (cancelado()) return -1;
    if (millis() - t0 > 30000) { Serial.println(F("# no vi la bola quieta en 30 s")); return -1; }
    if (medir()) {
      if (m && (fabsf(posX - sx / m) > 0.006f || fabsf(posY - sy / m) > 0.006f)) { sx = sy = 0; m = 0; tq = millis(); }
      sx += posX; sy += posY; m++;
      if (millis() - tq >= 400 && m >= 15) break;
    } else { sx = sy = 0; m = 0; tq = millis(); }
    delay(5);
  }
  ref[0] = sx / m; ref[1] = sy / m;
  Serial.println(F("# SUELTALA (sin empujarla)"));
  // 2) esperar que ruede: 2 lecturas seguidas a mas de 7 mm del punto sostenido
  t0 = millis();
  uint8_t fuera = 0; float tPrev = 0, xPrev = 0, yPrev = 0; uint32_t tIni = micros();
  while (true) {
    if (cancelado()) return -1;
    if (millis() - t0 > 3000) {
      a[0] = a[1] = 0;
      Serial.println(F("#   no rodo en 3 s: en esa posicion la placa esta casi horizontal"));
      return 1;
    }
    uint32_t us = micros();
    if (medir()) {
      bool lejos = fabsf(posX - ref[0]) > 0.007f || fabsf(posY - ref[1]) > 0.007f;
      if (lejos && ++fuera >= 2) break;
      if (!lejos) fuera = 0;
      tPrev = (us - tIni) * 1e-6f; xPrev = posX; yPrev = posY;
    }
  }
  // 3) registrar desde la ultima lectura cerca del punto sostenido, hasta 0,6 s o 5 cm
  uint8_t n = 0; uint32_t tUlt = millis();
  tTiro[n] = tPrev; xTiro[n] = xPrev; yTiro[n] = yPrev; n++;
  tTiro[n] = (micros() - tIni) * 1e-6f; xTiro[n] = posX; yTiro[n] = posY; n++;
  float tFin = tTiro[1] + 0.6f;
  while (n < N_TIRO) {
    uint32_t us = micros();
    float t = (us - tIni) * 1e-6f;
    if (t > tFin) break;
    if (medir()) {
      tTiro[n] = t; xTiro[n] = posX; yTiro[n] = posY; n++; tUlt = millis();
      if (fabsf(posX - ref[0]) > 0.05f || fabsf(posY - ref[1]) > 0.05f) break;
    } else if (millis() - tUlt > 120) break;                // se fue de la pantalla
  }
  float dur = tTiro[n - 1] - tTiro[0];
  if (n < 12 || dur < 0.12f) {
    Serial.printf("#   muy pocos datos (%u lecturas en %.2f s): otra vez\n", n, dur);
    return 0;
  }
  a[0] = aceleracionParabola(tTiro, xTiro, n);
  a[1] = aceleracionParabola(tTiro, yTiro, n);
  Serial.printf("#   %u lecturas en %.2f s: aceleracion x %+.1f  y %+.1f cm/s2\n", n, dur, a[0] * 100, a[1] * 100);
  return 1;
}
// hasta 3 intentos por tirada
bool tirada(float a[2], const char *etiqueta) {
  for (int k = 0; k < 3; k++) {
    int r = medirSoltada(a, etiqueta);
    if (r == 1) return true;
    if (r < 0) return false;
  }
  Serial.println(F("# no pude medir esa tirada: revisa que la bola se vea (panel) y repite nivel"));
  return false;
}
// 4 tiradas con cada servo corrido +-D: calcula y aplica los centros. Devuelve false si falla.
bool pasadaNivel(int pasada, float &dcMax, float Kmed[2]) {
  const float D = 6;                                        // grados de servo
  const float paso[4][2] = {{+D, 0}, {-D, 0}, {0, +D}, {0, -D}};
  float c0[2] = {ej[0].centro, ej[1].centro}, a[4][2];
  for (int i = 0; i < 4; i++) {
    escribir(0, c0[0] + paso[i][0]); escribir(1, c0[1] + paso[i][1]); delay(300);
    char et[56]; snprintf(et, sizeof et, "[pasada %d, %d/4] servo X %+.0f, servo Y %+.0f", pasada, i + 1, paso[i][0], paso[i][1]);
    if (!tirada(a[i], et)) { nivelar(); return false; }
  }
  // respuesta de la bola: a = a0 + G (c - c0), G[eje de la bola][servo]
  float G[2][2], a0[2];
  for (int i = 0; i < 2; i++) {
    G[i][0] = (a[0][i] - a[1][i]) / (2 * D); G[i][1] = (a[2][i] - a[3][i]) / (2 * D);
    a0[i] = (a[0][i] + a[1][i] + a[2][i] + a[3][i]) / 4;
  }
  Serial.printf("# respuesta [cm/s2 por grado]: servo X -> bola x %+.2f y %+.2f | servo Y -> bola x %+.2f y %+.2f\n",
                G[0][0] * 100, G[1][0] * 100, G[0][1] * 100, G[1][1] * 100);
  // Que servo mueve cada eje: solo se dan por cruzados si es claro (el doble).
  bool cruz = fabsf(G[1][0]) + fabsf(G[0][1]) > 2 * (fabsf(G[0][0]) + fabsf(G[1][1]));
  // Repetibilidad: en las dos tiradas en que se mueve el OTRO servo, el servo de este eje esta quieto,
  // asi que la bola deberia acelerar casi igual en este eje. Si no, la placa no esta sujeta.
  // Tiradas 0,1 mueven el servo X; 2,3 el servo Y.
  float r1[2], r2[2];
  if (!cruz) { r1[0] = a[2][0]; r2[0] = a[3][0]; r1[1] = a[0][1]; r2[1] = a[1][1]; }
  else       { r1[0] = a[0][0]; r2[0] = a[1][0]; r1[1] = a[2][1]; r2[1] = a[3][1]; }
  bool malo = false;
  for (int k = 0; k < 2; k++) {
    if (fabsf(r1[k] - r2[k]) > 0.30f) {
      Serial.printf("# EJE %c NO REPITE: con su servo quieto la bola acelero %+.0f y %+.0f cm/s2 en %c (deberian ser casi iguales).\n",
                    k ? 'Y' : 'X', r1[k] * 100, r2[k] * 100, k ? 'y' : 'x');
      malo = true;
    }
  }
  if (malo) {
    Serial.println(F("#   La placa se mueve sin que se mueva el servo: juego en la biela o en las rotulas, brazo flojo en el"));
    Serial.println(F("#   estriado o tornillo de la horquilla suelto. Sin bola y con los servos prendidos, empuja el borde de"));
    Serial.println(F("#   la mesa en ese eje: no deberia moverse. Arregla eso y repite nivel. No cambie nada."));
    ej[0].centro = c0[0]; ej[1].centro = c0[1]; nivelar(); return false;
  }
  if (cruz) {
    uint8_t t = PIN_SX; PIN_SX = PIN_SY; PIN_SY = t; conectarServos();
    float c = c0[0]; c0[0] = c0[1]; c0[1] = c;
    for (int i = 0; i < 2; i++) { float g = G[i][0]; G[i][0] = G[i][1]; G[i][1] = g; }
    Serial.printf("# los servos estaban cruzados: ahora servo X en GPIO%u, Y en GPIO%u\n", PIN_SX, PIN_SY);
  }
  for (int k = 0; k < 2; k++) {
    // El roce solo puede bajar la respuesta medida; si sale mucho MAYOR que la geometria, la placa
    // se inclina mas de lo que la mueve el servo: tiene juego.
    if (fabsf(G[k][k]) > 1.6f * K_TEORICO) {
      Serial.printf("# EJE %c CON JUEGO: la bola responde %.1f cm/s2 por grado de servo y la geometria da %.1f: demasiado.\n",
                    k ? 'Y' : 'X', fabsf(G[k][k]) * 100, K_TEORICO * 100);
      Serial.println(F("#   La placa se inclina mas de lo que la mueve el servo: juego en la biela o en las rotulas, brazo"));
      Serial.println(F("#   flojo en el estriado o tornillo de la horquilla suelto. Sin bola y con los servos prendidos, empuja"));
      Serial.println(F("#   el borde de la mesa en ese eje: no deberia moverse. Arregla eso y repite nivel. No cambie nada."));
      if (cruz) {                                           // deshacer el intercambio: no cambiar nada
        uint8_t t = PIN_SX; PIN_SX = PIN_SY; PIN_SY = t; conectarServos();
        ej[0].centro = c0[1]; ej[1].centro = c0[0];
      } else { ej[0].centro = c0[0]; ej[1].centro = c0[1]; }
      nivelar(); return false;
    }
    if (fabsf(G[k][k]) < 0.3f * K_TEORICO) {
      Serial.printf("# el servo %c casi no mueve la bola (%.2f cm/s2 por grado; se esperan ~%.1f).\n", k ? 'Y' : 'X', fabsf(G[k][k]) * 100, K_TEORICO * 100);
      Serial.println(F("#   Revisa alimentacion del servo, biela y que la placa no tope. No cambie nada."));
      if (cruz) {                                           // deshacer el intercambio: no cambiar nada
        uint8_t t = PIN_SX; PIN_SX = PIN_SY; PIN_SY = t; conectarServos();
        ej[0].centro = c0[1]; ej[1].centro = c0[0];
      } else { ej[0].centro = c0[0]; ej[1].centro = c0[1]; }
      nivelar(); return false;
    }
  }
  // centros donde la bola no acelera: G dc = -a0
  float det = G[0][0] * G[1][1] - G[0][1] * G[1][0], dc[2];
  if (fabsf(det) > 1e-9f) {
    dc[0] = (-a0[0] * G[1][1] + a0[1] * G[0][1]) / det;
    dc[1] = (-a0[1] * G[0][0] + a0[0] * G[1][0]) / det;
  } else { dc[0] = -a0[0] / G[0][0]; dc[1] = -a0[1] / G[1][1]; }
  dcMax = 0;
  for (int k = 0; k < 2; k++) {
    dc[k] = constrain(dc[k], -15.0f, 15.0f);
    dcMax = max(dcMax, fabsf(dc[k]));
    ej[k].centro = constrain(c0[k] + dc[k], 30.0f, 150.0f);
    ej[k].signo = G[k][k] > 0 ? 1 : -1; signoOk[k] = true;
    Kmed[k] = fabsf(G[k][k]);
  }
  nivelar(); delay(300);
  Serial.printf("# cx %.1f (%+.1f)  cy %.1f (%+.1f) | signo x %+.0f y %+.0f\n", ej[0].centro, dc[0], ej[1].centro, dc[1], ej[0].signo, ej[1].signo);
  return true;
}
void autoNivel() {
  Serial.printf("# NIVEL: centros de partida cx %.1f cy %.1f. Cualquier tecla + Enter cancela.\n", ej[0].centro, ej[1].centro);
  float dcMax = 0, Km[2];
  if (!pasadaNivel(1, dcMax, Km)) return;
  // Si se corrio mucho, una segunda pasada alrededor del centro nuevo: ahi la bola rueda para los
  // dos lados y el roce de la membrana se cancela en el promedio.
  if (dcMax > 3) {
    Serial.println(F("# se corrio bastante: segunda pasada para afinar"));
    if (!pasadaNivel(2, dcMax, Km)) return;
  }
  // K: el roce sesga la medida ~20 %, asi que el control usa el valor teorico (geometria de la
  // bandeja v5). Si la medida se aleja mucho, algo distinto hay en la mecanica: avisar.
  for (int k = 0; k < 2; k++) {
    if (Km[k] < 0.6f * K_TEORICO)
      Serial.printf("# OJO servo %c: la bola responde %.2f cm/s2 por grado y la geometria dice %.2f. Revisa el agujero del\n"
                    "#   brazo (20 mm), juego en la biela o el tipo de bola; si es real, ajusta con  kg\n", k ? 'Y' : 'X', Km[k] * 100, K_TEORICO * 100);
  }
  Serial.printf("# ganancia medida x %.2f y %.2f cm/s2 por grado (teorica %.2f). El control usa x %.2f y %.2f (kg para cambiarla)\n",
                Km[0] * 100, Km[1] * 100, K_TEORICO * 100, ej[0].K * 100, ej[1].K * 100);
  float r[2];
  if (!tirada(r, "[comprobacion] centro nuevo")) return;
  float ex = fabsf(r[0]) / K_TEORICO, ey = fabsf(r[1]) / K_TEORICO;   // grados de servo que faltan
  if (ex > 8 || ey > 8) {
    Serial.printf("# queda: %.1f y %.1f grados de servo. Es mas de lo que movio cualquier tirada: la placa no se quedo donde\n", ex, ey);
    Serial.println(F("#   la puso el servo. Revisa juego mecanico en ese eje (biela, rotulas, brazo en el estriado)."));
    return;
  }
  Serial.printf("# queda: %.1f y %.1f grados de servo de desnivel%s\n", ex, ey,
                (ex > 1.5f || ey > 1.5f) ? ". Repite  nivel  para afinar" : ". Listo: escribe  guardar");
}

// diagnostico del panel: 2 s comparando los dos metodos con la bola donde este
void diagnosticoPanel() {
  uint8_t ok1 = 0, ok2 = 0, n = 40; float kx = 0, ky = 0; uint8_t nk = 0;
  Serial.println(F("# panel: 2 s. hiX loX kX -> crudoX | hiY loY kY -> crudoY | doble carga | toque digital"));
  for (uint8_t i = 0; i < n; i++) {
    int rx = leerEje(XP, XM, YP, YM, 0), ry = leerEje(YP, YM, XP, XM, 1);
    bool d = rx >= 0 && ry >= 0;
    if (d) { ok2++; kx += kCont[0]; ky += kCont[1]; nk++; }
    int r1x, r1y; bool t = leerPanel(r1x, r1y);
    if (t) ok1++;
    if (i % 5 == 0)
      Serial.printf("#  %4d %4d %.2f -> %4d | %4d %4d %.2f -> %4d | %s | %s\n", mvHi[0], mvLo[0], kCont[0], rx,
                    mvHi[1], mvLo[1], kCont[1], ry, d ? "si" : "no", t ? "si" : "no");
    delay(50);
  }
  Serial.printf("# doble carga (metodo 2): %u/%u buenas, k medio X %.2f Y %.2f | toque digital (metodo 1): %u/%u\n",
                ok2, n, nk ? kx / nk : 1.0f, nk ? ky / nk : 1.0f, ok1, n);
  if (ok2 < n / 2 && nk && (kx / nk > 0.25f || ky / nk > 0.25f))
    Serial.println(F("#   el contacto es lento (k alto): prueba  asentar 1000"));
  if (ok2 == 0 && ok1 == 0)
    Serial.println(F("#   ninguno ve la bola: revisa los 4 cables del panel o apoya mas peso para probar"));
  if (ok2 > ok1 + 5) Serial.println(F("#   el toque digital pierde la bola: por eso decia 'no veo la bola'. Usa metodo 2"));
}

// ===================== TELEMETRIA Y AYUDA =====================
void ver() {
  const char *est[3] = {"DETENIDO", "ESPERANDO bola", "ACTIVO"};
  Serial.printf("# estado %s (%s) | ejes %s\n", est[estado], motivo, ejes == 3 ? "xy" : (ejes == 1 ? "x" : "y"));
  Serial.printf("# panel: toque %d valida %d raw %d %d -> x %.1f y %.1f cm | cal X %d..%d Y %d..%d %s\n",
                toqueAct, validaAct, rawX, rawY, posX * 100, posY * 100, X_INI, X_FIN, Y_INI, Y_FIN,
                panelCal ? "(guardada)" : "(POR DEFECTO: haz cal)");
  Serial.printf("# lectura %lu us (metodo %u, asentar %u us, k %.2f %.2f) | periodo %lu us (max %lu) | cortes %lu | saltos %lu %lu\n",
                (unsigned long)usLectura, metodo, asentarUs, kCont[0], kCont[1], (unsigned long)usPeriodo,
                (unsigned long)usPeriodoMax, (unsigned long)cortes, ej[0].ob.nRechazos, ej[1].ob.nRechazos);
  if (usLectura > periodoUs * 3 / 4) Serial.println(F("#   OJO: la lectura del panel se come casi todo el periodo"));
  Serial.printf("# tiempos maximos con el control andando [us]: lectura %lu  control %lu  servos %lu  serie %lu\n",
                (unsigned long)usLecturaMax, (unsigned long)usCtrlMax, (unsigned long)usServoMax, (unsigned long)usSerieMax);
  for (int k = 0; k < 2; k++)
    Serial.printf("# eje %c: centro %.1f  signo %+.0f %s  servo %.1f  d %.1f cm/s2\n", k ? 'Y' : 'X', ej[k].centro,
                  ej[k].signo, signoOk[k] ? "(ok)" : "(SIN PROBAR: prueba)", servoAct[k], ej[k].ob.d * 100);
  Serial.printf("# fuzzy E %.1f cm  V %.1f cm/s  A %.0f cm/s2 | K x %.2f y %.2f cm/s2 por grado | smax %.0f | obs %d (%.0f / %.1f rad/s)\n",
                esc.E * 100, esc.V * 100, esc.A * 100, ej[0].K * 100, ej[1].K * 100, ej[0].smax, ej[0].usarObs, ej[0].ob.wxv, ej[0].ob.wd);
  Serial.printf("# retardo servo->bola x %.0f y %.0f ms (prediccion %s) | tau %.0f ms\n", ej[0].retardo * 1000, ej[1].retardo * 1000,
                (ej[0].retardo > 0 || ej[1].retardo > 0) ? "activa" : "apagada", ej[0].tauServo * 1000);
  Serial.printf("# referencia %.1f %.1f cm, circulo R %.1f cm T %.1f s\n", spX * 100, spY * 100, circR * 100, circT);
}
void ayuda() {
  Serial.println(F("# --- comandos ---"));
  Serial.println(F("# ver | stop | iniciar [x|y|xy] | p 0|1|2 | guardar | fabrica"));
  Serial.println(F("# panel               diagnostico del panel con la bola puesta (2 s)"));
  Serial.println(F("# metodo 1|2 | asentar 500 (us) | perdida 200 (ms)   lectura del panel"));
  Serial.println(F("# cal / cal fin        calibrar el panel (pasea la bola apoyada por los 4 bordes)"));
  Serial.println(F("# cx 80 | cy 78        centro de cada servo (placa horizontal)"));
  Serial.println(F("# nivel                nivela solo: sostienes y sueltas la bola 5 a 9 veces (centros y signos)"));
  Serial.println(F("# prueba x 6 | prueba y 6   deduce el signo | signo x 1|-1 | intercambiar"));
  Serial.println(F("# centrar              (activo) corrige cx/cy con la perturbacion promedio (mejor con circulo 2 4)"));
  Serial.println(F("# fz 5 20 90           escalas fuzzy: error cm, velocidad cm/s, aceleracion max cm/s2"));
  Serial.println(F("# kg 5.33 | kg 10.7 4.75 (x y) | smax 22 | obs 0|1 [wd wxv] | periodo 20"));
  Serial.println(F("# retardo 160 200      tiempo muerto servo->bola en ms (x y); 0 = sin prediccion"));
  Serial.println(F("# sp 2 -1 (cm) | circulo 3 5 (R cm, T s) | centro"));
}
void telemetria(const float sp[2]) {
  if (!imprimir) return;
  if (estado == DETENIDO) {            // detenido: solo durante cal, 5 por segundo, para ver los crudos
    if (calibrando && nPaso % 10 == 0)
      Serial.printf("# cal: toque %d raw %d %d | X %d..%d  Y %d..%d (%d)\n", toqueAct, rawX, rawY, cMinX, cMaxX, cMinY, cMaxY, cN);
    return;
  }
  if (imprimir == 1 && (nPaso & 1)) return;
  Serial.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%u\n",
                (unsigned long)millis(), validaAct, posX * 100, posY * 100, ej[0].ob.x * 100, ej[1].ob.x * 100,
                ej[0].ob.v * 100, ej[1].ob.v * 100, sp[0] * 100, sp[1] * 100, servoAct[0], servoAct[1],
                ej[0].ob.d * 100, ej[1].ob.d * 100, estado);
}

// ===================== COMANDOS =====================
char linea[80]; uint8_t nL = 0;
void comando(char *s) {
  char *tok[5]; int n = 0;
  for (char *t = strtok(s, " \t"); t && n < 5; t = strtok(NULL, " \t")) tok[n++] = t;
  if (!n) return;
  const char *c = tok[0];
  float v[4] = {0, 0, 0, 0};
  for (int i = 1; i < n; i++) v[i - 1] = atof(tok[i]);
  bool esX = n > 1 && !strcmp(tok[1], "x"), esY = n > 1 && !strcmp(tok[1], "y");

  // ---- validos siempre ----
  if (!strcmp(c, "stop")) { calibrando = false; detener("orden stop"); return; }
  if (!strcmp(c, "ver")) { ver(); return; }
  if (!strcmp(c, "?") || !strcmp(c, "ayuda")) { ayuda(); return; }
  if (!strcmp(c, "p") && n == 2) { imprimir = (uint8_t)constrain((int)v[0], 0, 2); return; }
  if (!strcmp(c, "sp") && n == 3) { spX = constrain(v[0], -6.0f, 6.0f) / 100; spY = constrain(v[1], -4.5f, 4.5f) / 100; circR = 0; return; }
  if (!strcmp(c, "centro")) { spX = spY = 0; circR = 0; return; }
  if (!strcmp(c, "circulo") && n == 3 && v[0] >= 0 && v[0] <= 4 && v[1] >= 2 && v[1] <= 30) {
    circR = v[0] / 100; circT = v[1]; circT0 = millis(); spX = spY = 0; return;
  }
  if (!strcmp(c, "centrar")) {
    if (estado != ACTIVO) { Serial.println(F("# centrar: primero iniciar")); return; }
    // promedia la perturbacion estimada; con la bola girando (circulo) el roce se cancela solo
    centrando = true; tCentrar = 0; sumD[0] = sumD[1] = 0;
    tCentrarObj = circR > 0 ? 2 * circT : 4.0f;
    Serial.printf("# centrando: promedio %.0f s%s\n", tCentrarObj, circR > 0 ? " (con circulo: mejor)" : " (mejor con: circulo 2 4)");
    return;
  }
  if (!strcmp(c, "iniciar")) {
    ejes = (esX ? 1 : (esY ? 2 : 3));
    for (int k = 0; k < 2; k++)
      if ((ejes & (1 << k)) && !signoOk[k]) { Serial.printf("# falta probar el signo de %c: prueba %c 6\n", k ? 'y' : 'x', k ? 'y' : 'x'); return; }
    if (!panelCal) Serial.println(F("# aviso: panel con la calibracion por defecto"));
    calibrando = false; nivelar(); estado = ESPERANDO; buenas = 0; motivo = "esperando bola";
    Serial.println(F("# ESPERANDO: pon la bola (arranca con 5 lecturas buenas)"));
    return;
  }
  if (estado != DETENIDO) { Serial.println(F("# primero stop")); return; }

  // ---- solo detenido ----
  if (!strcmp(c, "cal")) {
    if (n > 1 && !strcmp(tok[1], "fin")) {
      calibrando = false;
      if (cN < 30 || cMaxX - cMinX < 200 || cMaxY - cMinY < 200) { Serial.printf("# cal incompleta (%d lecturas, X %d, Y %d): repite\n", cN, cMaxX - cMinX, cMaxY - cMinY); return; }
      X_INI = cMinX; X_FIN = cMaxX; Y_INI = cMinY; Y_FIN = cMaxY; panelCal = true; guardarHW();
      Serial.printf("# cal guardada: X %d..%d  Y %d..%d\n", X_INI, X_FIN, Y_INI, Y_FIN);
    } else {
      calibrando = true; cN = 0; cMinX = cMinY = 1023; cMaxX = cMaxY = 0; nivelar();
      Serial.println(F("# CALIBRANDO: pasea la bola apoyada por los 4 bordes (sin la franja ciega) y escribe cal fin"));
    }
    return;
  }
  if ((!strcmp(c, "cx") || !strcmp(c, "cy")) && n == 2 && v[0] >= 30 && v[0] <= 150) {
    int k = c[1] == 'y'; ej[k].centro = v[0]; nivelar(); Serial.printf("# %s = %.1f\n", c, v[0]); return;
  }
  if (!strcmp(c, "nivel")) { autoNivel(); return; }
  if (!strcmp(c, "prueba") && (esX || esY)) {
    float g = n > 2 ? v[1] : 6; if (fabsf(g) < 2 || fabsf(g) > 15) { Serial.println(F("# prueba x|y 2..15")); return; }
    prueba(esY, g); return;
  }
  if (!strcmp(c, "signo") && (esX || esY) && n == 3 && (v[1] == 1 || v[1] == -1)) {
    ej[esY].signo = v[1]; signoOk[esY] = true; Serial.println(F("# ok")); return;
  }
  if (!strcmp(c, "intercambiar")) {
    // los centros son de cada servo fisico: se intercambian con ellos
    uint8_t t = PIN_SX; PIN_SX = PIN_SY; PIN_SY = t;
    float c0 = ej[0].centro; ej[0].centro = ej[1].centro; ej[1].centro = c0;
    conectarServos(); nivelar(); signoOk[0] = signoOk[1] = false;
    guardarHW(); Serial.printf("# servo X ahora en GPIO%u (cx %.1f), Y en GPIO%u (cy %.1f). Repite las pruebas de signo\n",
                               PIN_SX, ej[0].centro, PIN_SY, ej[1].centro); return;
  }
  if (!strcmp(c, "fz") && n == 4 && v[0] >= 1 && v[0] <= 20 && v[1] >= 3 && v[1] <= 100 && v[2] >= 10 && v[2] <= 150) {
    esc.E = v[0] / 100; esc.V = v[1] / 100; esc.A = v[2] / 100;
    Serial.printf("# fuzzy E %.1f cm, V %.1f cm/s, A %.0f cm/s2 (~%.1f grados de servo)\n", v[0], v[1], v[2], esc.A / ej[0].K); return;
  }
  if (!strcmp(c, "kg") && n >= 2 && v[0] >= 1 && v[0] <= 30) {
    ej[0].K = v[0] / 100; ej[1].K = (n == 3 && v[1] >= 1 && v[1] <= 30 ? v[1] : v[0]) / 100;
    Serial.printf("# K x %.2f y %.2f cm/s2 por grado de servo\n", ej[0].K * 100, ej[1].K * 100); return;
  }
  if (!strcmp(c, "retardo") && n >= 2 && v[0] >= 0 && v[0] <= 500) {
    float ry = (n >= 3 && v[1] >= 0 && v[1] <= 500) ? v[1] : v[0];
    ej[0].retardo = v[0] / 1000; ej[1].retardo = ry / 1000;
    for (int k = 0; k < 2; k++) ej[k].tauServo = ej[k].retardo > 0 ? 0.03f : 0.06f;
    Serial.printf("# retardo x %.0f y %.0f ms\n", v[0], ry); return;
  }
  if (!strcmp(c, "smax") && n == 2 && v[0] >= 3 && v[0] <= SMAX_ABS) { ej[0].smax = ej[1].smax = v[0]; return; }
  if (!strcmp(c, "obs") && n >= 2) {
    ej[0].usarObs = ej[1].usarObs = v[0] != 0;
    // obs 1 6 15: polo de la perturbacion (wd) y de posicion/velocidad (wxv) [rad/s]. Mas rapido = la integral
    // alcanza antes una inclinacion nueva (dron, base movida), pero con mas ruido en los servos
    if (n >= 3 && v[1] >= 0.3f && v[1] <= 15) ej[0].ob.wd = ej[1].ob.wd = v[1];
    if (n >= 4 && v[2] >= 4 && v[2] <= 30) ej[0].ob.wxv = ej[1].ob.wxv = v[2];
    Serial.printf("# observador %s (perturbacion %.1f rad/s, posicion %.0f rad/s)\n", ej[0].usarObs ? "cancela la perturbacion" : "solo estima",
                  ej[0].ob.wd, ej[0].ob.wxv); return;
  }
  if (!strcmp(c, "periodo") && n == 2 && v[0] >= 10 && v[0] <= 40) { periodoUs = (uint32_t)(v[0] * 1000); return; }
  if (!strcmp(c, "panel")) { diagnosticoPanel(); return; }
  if (!strcmp(c, "metodo") && n == 2 && (v[0] == 1 || v[0] == 2)) { metodo = (uint8_t)v[0]; Serial.printf("# metodo %u\n", metodo); return; }
  if (!strcmp(c, "asentar") && n == 2 && v[0] >= 50 && v[0] <= 2000) { asentarUs = (uint16_t)v[0]; return; }
  if (!strcmp(c, "perdida") && n == 2 && v[0] >= 100 && v[0] <= 600) { tPerdida = v[0] / 1000; return; }
  if (!strcmp(c, "guardar")) { guardar(); return; }
  if (!strcmp(c, "fabrica")) { EEPROM.put(DIR_FZ, (uint16_t)0); EEPROM.commit(); Serial.println(F("# config borrada (la calibracion del panel se conserva)")); return; }
  Serial.println(F("# no entendi ( ? = ayuda )"));
}
void leerSerie() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { linea[nL] = 0; if (nL) comando(linea); nL = 0; }
    else if (nL < sizeof(linea) - 1) linea[nL++] = ch;
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  for (uint8_t i = 0; i < 4; i++) { pinMode(PIN_ESC[i], OUTPUT); digitalWrite(PIN_ESC[i], LOW); }
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n# BALL & PLATE FUZZY + observador (ESP32, bandeja v5, solo pantalla y servos) - version 27-oct 18:15"));
#if defined(ARDUINO_ARCH_ESP32)
  {
    esp_reset_reason_t rr = esp_reset_reason();
    if (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT || rr == ESP_RST_WDT)
      Serial.println(F("# OJO: el ESP32 se reinicio por WATCHDOG (algo no cedio el nucleo). Mandame el log"));
    else if (rr == ESP_RST_PANIC) Serial.println(F("# OJO: el ESP32 se reinicio por un error (panic). Mandame el log"));
    else if (rr == ESP_RST_BROWNOUT) Serial.println(F("# OJO: el ESP32 se reinicio por BAJA TENSION (brownout): revisa la alimentacion de los servos"));
  }
#endif
  analogReadResolution(12);
  soltar();
  EEPROM.begin(TAM_EEPROM);
  ej[0].centro = 80; ej[1].centro = 78;      // los del sketch PID; hay que nivelar con cx / cy
  for (int k = 0; k < 2; k++) ej[k].K = K_TEORICO;
  cargarHW();
  cargar();
  conectarServos();
  nivelar();
#if defined(ARDUINO_ARCH_ESP32)
  iniciarTareasServo();
  pausaServos = false;
#endif
  Serial.println(F("# DETENIDO. ? = ayuda, ver = estado"));
  proximo = ultimo = micros();
}

void loop() {
  leerSerie();
  uint32_t ahora = micros();
  if ((int32_t)(ahora - proximo) < 0) return;
  proximo += periodoUs;
  if ((int32_t)(ahora - proximo) > 0) proximo = ahora + periodoUs;   // atrasado: no acumular
  uint32_t dtUs = ahora - ultimo; ultimo = ahora;
  float dt = constrain(dtUs * 1e-6f, 0.002f, 0.2f);
  usPeriodo = dtUs; nPaso++;

  uint32_t t0 = micros();
  bool valida = medir();
  usLectura = micros() - t0;
  if (estado == ACTIVO && usLectura > usLecturaMax) usLecturaMax = usLectura;

  // calibracion: vale cualquier lectura con toque (aunque caiga fuera de la cal vieja), pero solo si
  // se repite (+-20) en la lectura siguiente: una lectura basura en la franja ciega no estira los bordes
  static int prevX = -100, prevY = -100;
  if (calibrando && toqueAct) {
    if (abs(rawX - prevX) <= 20 && abs(rawY - prevY) <= 20) {
      cMinX = min(cMinX, rawX); cMaxX = max(cMaxX, rawX); cMinY = min(cMinY, rawY); cMaxY = max(cMaxY, rawY); cN++;
    }
    prevX = rawX; prevY = rawY;
  } else { prevX = prevY = -100; }

  float sp[2], vsp[2], asp[2];
  referencia(sp, vsp, asp);

  if (estado == ESPERANDO) {
    // 5 lecturas buenas sin un hueco de mas de 0,3 s entre ellas (el contacto de la bola es intermitente)
    if (valida) { if (++buenas >= 5) arrancar(posX, posY, dt); }
    else if (millis() - tUltValida > 300) buenas = 0;
  } else if (estado == ACTIVO) {
    if (dtUs > usPeriodoMax) usPeriodoMax = dtUs;
    if (dt > 0.1f) { detener("periodo > 100 ms"); telemetria(sp); return; }
    if (valida) tSinBola = 0;
    else { cortes++; tSinBola += dt; if (tSinBola >= tPerdida) { detener("bola perdida (sin lectura valida)"); telemetria(sp); return; } }
    if (centrando) {
      for (int k = 0; k < 2; k++) sumD[k] += ej[k].ob.d * dt;
      tCentrar += dt;
      if (tCentrar >= tCentrarObj) {
        centrando = false;
        for (int k = 0; k < 2; k++) {
          if (!(ejes & (1 << k))) continue;
          float dm = sumD[k] / tCentrar, du = -dm / (ej[k].signo * ej[k].K);
          if (fabsf(du) > 8) { Serial.printf("# centrar %c: pide %.1f grados, demasiado; revisa signo o kg\n", k ? 'y' : 'x', du); continue; }
          ej[k].centro += du; ej[k].ob.d -= dm;
          Serial.printf("# c%c = %.1f (%+.1f)\n", k ? 'y' : 'x', ej[k].centro, du);
        }
        Serial.println(F("# 'guardar' (despues de stop) para conservar los centros"));
      }
    }
    float z[2] = {posX, posY}, u[2];
    uint32_t t1 = (uint32_t)micros();
    for (uint8_t k = 0; k < 2; k++)
      u[k] = (ejes & (1 << k)) ? ej[k].paso(esc, valida, z[k], dt, sp[k], vsp[k], asp[k]) : ej[k].centro;
    uint32_t t2 = (uint32_t)micros();
    escribir(0, u[0]); escribir(1, u[1]);
    uint32_t t3 = (uint32_t)micros();
    usCtrlMax = max(usCtrlMax, t2 - t1); usServoMax = max(usServoMax, t3 - t2);
  }
  uint32_t t4 = (uint32_t)micros();
  telemetria(sp);
  if (estado == ACTIVO) usSerieMax = max(usSerieMax, (uint32_t)(micros() - t4));
}
