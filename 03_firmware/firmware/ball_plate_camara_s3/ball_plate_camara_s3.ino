/*
  BALL & PLATE CON VISION POR COMPUTADORA (ESP32-S3-CAM N16R8 + OV2640 + 2 SERVOS)
  Reemplaza la lectura del panel resistivo roto por deteccion en tiempo real en QVGA (320x240)
  conservando integro el controlador Fuzzy PD (Sugeno 7x7) + Observador alfa-beta-gamma +
  Predictor de retardo tipo Smith de fuzzy_bp.h.

  ARQUITECTURA EN TIEMPO REAL:
    - Nucleo 0 (tareaCaptura): captura frames RGB565 en PSRAM (320x240), ejecuta segmentacion
      en la ROI del plato (con sustraccion opcional de fondo y ventana local de centroide) y
      publica cada medida con su numero de secuencia y timestamp en microsegundos.
    - Nucleo 0 (tareas servoX/servoY): actualizan el PWM de los servos Futaba S3003 en canales
      LEDC 2 y 3 (50 Hz, 14 bits) sin bloquear el lazo de control ni tocar el canal 0 de XCLK.
    - Nucleo 1 (loop a 50 Hz / 20 ms): consume cada frame nuevo UNA SOLA VEZ para corregir el
      observador alfa-beta-gamma; en los pasos intermedios sin frame nuevo el observador predice
      con la aceleracion mandada a los servos.

  COMANDOS PRINCIPALES (115200 baudios, Nueva linea):
    ver | ? | stop | iniciar [x|y|xy] | p 0|1|2 | guardar | fabrica
    panel / camara          diagnostico de 2 s de FPS, area y deteccion de la bola
    fondo                   (sin bola) guarda imagen de fondo del plato en PSRAM para restar reflejos
    usar_fondo 0|1          activa/desactiva resta de fondo
    aprender                (con bola en el centro) calibra automaticamente brillo/color y umbral
    modo 0|1|2              0 = bola clara, 1 = bola oscura, 2 = color aprendido
    umbral <valor>          umbral de deteccion (5..240) | area <min> <max>
    cal / cal fin           calibra bordes del plato en pixeles paseando la bola por las 4 esquinas
    roi <xi> <xf> <yi> <yf> fija manualmente el rectangulo del plato en pixeles (0..319, 0..239)
    plato <lx_cm> <ly_cm>   dimensiones utiles del plato en cm (por defecto 17.2 x 12.95)
    servos 0|1              activa/desactiva PWM en los pines de servo (GPIO 41 y 42 por defecto)
    pines_servo <px> <py>   cambia los GPIO de servo X e Y
    cx 80 | cy 78 | prueba x 6 | prueba y 6 | signo x 1|-1 | intercambiar | nivel | centrar
    fz 5 20 90 | kg 10.7 4.75 | retardo 120 140 | smax 22 | obs 0|1 [wd wxv]
    sp 2 -1 | circulo 3 5 | centro | foto (JPEG con cruz sobre la bola) | wifi (AP BolaCam-prueba)
*/
#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "pines_camara.h"
#include "fuzzy_bp.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Selecciona ESP32S3 Dev Module para compilar este firmware."
#endif

// ===================== RED Y WEB OPCIONAL =====================
constexpr char RED[] = "BolaCam-prueba";
constexpr char CLAVE[] = "ballplate1";
WebServer web(80);
bool webActiva = false;

// ===================== PINES Y SERVOS =====================
uint8_t PIN_SX = camhw::servoXDefault;
uint8_t PIN_SY = camhw::servoYDefault;
const float US_MIN = 500.0f, US_MAX = 2500.0f;
const int FREQ_SERVO = 50, RES_SERVO = 14;
const int CANAL_SERVO[2] = {2, 3};  // Canales 2 y 3 para no interferir con LEDC_CHANNEL_0 de la camara
int pinConectado[2] = {-1, -1};
float servoAct[2] = {80.0f, 78.0f};
const float SMAX_ABS = 30.0f;
volatile float servoObj[2] = {80.0f, 78.0f};
volatile bool pausaServos = false;
bool servosActivos = true;

// ===================== GEOMETRIA Y VISION =====================
// Coordenadas en decimas de pixel (0..3199 en X, 0..2399 en Y) para mantener precision subpixel
int X_INI = 300, X_FIN = 2900, Y_INI = 200, Y_FIN = 2200;
float LX = 0.172f, LY = 0.1295f;  // Area util del plato [m]
bool panelCal = false;

// Modos de vision: 0 = bola clara, 1 = bola oscura, 2 = color RGB aprendido
uint8_t modoVision = 0;
uint8_t umbralVision = 45;
uint16_t areaMin = 12, areaMax = 4500;
bool usarFondo = false;
bool fondoValido = false;
uint8_t colorRef[3] = {220, 220, 220};  // R, G, B (0..255)
bool invertirX = false, invertirY = true;  // En imagen de camara Y crece hacia abajo; invertirY pone +Y hacia arriba

// Buffers en PSRAM
SemaphoreHandle_t mutexCam = nullptr;
uint16_t *ultimoFrameRgb = nullptr;  // 320x240 RGB565
uint8_t *fondoLuma = nullptr;        // 320x240 luminancia (0..255)
volatile bool pedirFondo = false;
volatile bool pedirAprender = false;

struct MedidaVision {
  uint32_t seq = 0;
  uint32_t recibidas = 0, fallos = 0, detectadas = 0;
  uint32_t capturaUs = 0, visionUs = 0;
  int64_t tomadaUs = 0;
  float fps = 0;
  bool valida = false;
  int rawX = -1, rawY = -1;       // decimas de pixel (0..3200, 0..2400)
  int pxX = -1, pxY = -1;         // pixel entero (0..319, 0..239)
  uint16_t area = 0;
  uint8_t picoScore = 0;
  float posX = 0, posY = 0;       // metros respecto al centro de la ROI
} vis;

// ===================== CONTROL =====================
const float GRAV = 9.81f, FACTOR_BOLA = 5.0f / 7.0f;
const float GANANCIA_MEC = 0.436f;
const float K_TEORICO = FACTOR_BOLA * GRAV * GANANCIA_MEC * PI / 180.0f;  // 0.0533 m/s2 por grado

bp::Eje ej[2];
bp::Escalas esc;
bool signoOk[2] = {false, false};
uint32_t periodoUs = 20000;
float tPerdida = 0.25f;

enum { DETENIDO = 0, ESPERANDO = 1, ACTIVO = 2 };
uint8_t estado = DETENIDO, ejes = 3, buenas = 0;
const char *motivo = "inicio";
float tSinBola = 0;
uint32_t tUltValida = 0;
uint32_t ultSeqControl = 0;
uint32_t ultSeqMuestra = 0;

bool centrando = false;
float tCentrar = 0, tCentrarObj = 4, sumD[2] = {0, 0};
float spX = 0, spY = 0, circR = 0, circT = 5;
uint32_t circT0 = 0;

// ===================== DIAGNOSTICO =====================
uint32_t usCtrlMax = 0, usServoMax = 0, usSerieMax = 0, usLecturaMax = 0;
uint32_t usLectura = 0, usPeriodo = 0, usPeriodoMax = 0, cortes = 0, nPaso = 0;
uint8_t imprimir = 1;
int rawX = -1, rawY = -1;
bool toqueAct = false, validaAct = false;
float posX = 0, posY = 0;
uint16_t areaAct = 0;
bool calibrando = false;
int cMinX, cMaxX, cMinY, cMaxY, cN;
uint32_t proximo = 0, ultimo = 0;
bool camaraLista = false;

// ===================== EEPROM =====================
const int TAM_EEPROM = 1024;
const int DIR_HW = 900;
const uint16_t MAGICO_HW = 0x4356;  // 'CV'
struct HardwareCV {
  uint16_t magico;
  int xi, xf, yi, yf;
  uint8_t px, py;
  uint8_t modo, umbral;
  uint16_t areaMin, areaMax;
  uint8_t color[3];
  uint8_t flags;  // bit0: usarFondo, bit1: invX, bit2: invY, bit3: panelCal
  float lx, ly;
};
const int DIR_FZ = 600;
const uint16_t MAGICO_FZ = 0xF2B9;
struct ConfigFZ {
  uint16_t magico;
  float centro[2], signo[2];
  uint8_t sOk[2];
  float K[2], smax, E, V, A, wxv, wd;
  uint8_t obs;
  float perdida;
};
const int DIR_RET = 700;
const uint16_t MAGICO_RET = 0x5245;
struct ConfigRet {
  uint16_t magico;
  float retardo[2], tau;
};

void guardarHW() {
  HardwareCV h = {};
  h.magico = MAGICO_HW;
  h.xi = X_INI; h.xf = X_FIN; h.yi = Y_INI; h.yf = Y_FIN;
  h.px = PIN_SX; h.py = PIN_SY;
  h.modo = modoVision; h.umbral = umbralVision;
  h.areaMin = areaMin; h.areaMax = areaMax;
  h.color[0] = colorRef[0]; h.color[1] = colorRef[1]; h.color[2] = colorRef[2];
  h.flags = (usarFondo ? 1 : 0) | (invertirX ? 2 : 0) | (invertirY ? 4 : 0) | (panelCal ? 8 : 0);
  h.lx = LX; h.ly = LY;
  EEPROM.put(DIR_HW, h);
  EEPROM.commit();
}

void cargarHW() {
  HardwareCV h;
  EEPROM.get(DIR_HW, h);
  if (h.magico != MAGICO_HW) {
    Serial.println(F("# camara: sin calibracion guardada (usando ROI por defecto; usa cal o roi)"));
    return;
  }
  if (h.xi >= 0 && h.xf <= 3200 && h.yi >= 0 && h.yf <= 2400 && h.xf - h.xi >= 400 && h.yf - h.yi >= 400) {
    X_INI = h.xi; X_FIN = h.xf; Y_INI = h.yi; Y_FIN = h.yf;
  }
  if (h.px <= 48 && h.py <= 48 && h.px != h.py) { PIN_SX = h.px; PIN_SY = h.py; }
  modoVision = h.modo <= 2 ? h.modo : 0;
  umbralVision = constrain(h.umbral, 5, 245);
  areaMin = constrain(h.areaMin, 4, 2000);
  areaMax = constrain(h.areaMax, 50, 25000);
  colorRef[0] = h.color[0]; colorRef[1] = h.color[1]; colorRef[2] = h.color[2];
  usarFondo = (h.flags & 1) != 0;
  invertirX = (h.flags & 2) != 0;
  invertirY = (h.flags & 4) != 0;
  panelCal = (h.flags & 8) != 0;
  if (h.lx >= 0.05f && h.lx <= 0.50f) LX = h.lx;
  if (h.ly >= 0.05f && h.ly <= 0.50f) LY = h.ly;
  Serial.printf("# camara ROI: X %.1f..%.1f px  Y %.1f..%.1f px | servo X=GPIO%u Y=GPIO%u\n",
                X_INI * 0.1f, X_FIN * 0.1f, Y_INI * 0.1f, Y_FIN * 0.1f, PIN_SX, PIN_SY);
}

void guardar() {
  ConfigFZ c = {};
  c.magico = MAGICO_FZ;
  for (int k = 0; k < 2; k++) {
    c.centro[k] = ej[k].centro; c.signo[k] = ej[k].signo; c.sOk[k] = signoOk[k];
  }
  c.K[0] = ej[0].K; c.K[1] = ej[1].K; c.smax = ej[0].smax;
  c.E = esc.E; c.V = esc.V; c.A = esc.A;
  c.wxv = ej[0].ob.wxv; c.wd = ej[0].ob.wd; c.obs = ej[0].usarObs;
  c.perdida = tPerdida;
  EEPROM.put(DIR_FZ, c);
  ConfigRet r = {MAGICO_RET, {ej[0].retardo, ej[1].retardo}, ej[0].tauServo};
  EEPROM.put(DIR_RET, r);
  EEPROM.commit();
  guardarHW();
  Serial.println(F("# guardado en EEPROM"));
}

void cargar() {
  ConfigRet r = {};
  EEPROM.get(DIR_RET, r);
  if (r.magico == MAGICO_RET) {
    for (int k = 0; k < 2; k++) {
      ej[k].retardo = (r.retardo[k] >= 0 && r.retardo[k] <= 0.5f) ? r.retardo[k] : 0.06f;
      ej[k].tauServo = (r.tau >= 0.005f && r.tau <= 0.2f) ? r.tau : 0.04f;
    }
  }
  ConfigFZ c;
  EEPROM.get(DIR_FZ, c);
  if (c.magico != MAGICO_FZ) {
    Serial.println(F("# config control: valores por defecto"));
    return;
  }
  for (int k = 0; k < 2; k++) {
    ej[k].centro = constrain(c.centro[k], 30.0f, 150.0f);
    ej[k].signo = c.signo[k] < 0 ? -1.0f : 1.0f;
    signoOk[k] = c.sOk[k] != 0;
    ej[k].K = constrain(c.K[k], 0.01f, 0.3f);
    ej[k].smax = constrain(c.smax, 3.0f, SMAX_ABS);
    ej[k].ob.wxv = constrain(c.wxv, 4.0f, 30.0f);
    ej[k].ob.wd = constrain(c.wd, 0.0f, 15.0f);
    ej[k].usarObs = c.obs != 0;
  }
  esc.E = constrain(c.E, 0.01f, 0.2f);
  esc.V = constrain(c.V, 0.03f, 1.0f);
  esc.A = constrain(c.A, 0.1f, 1.5f);
  tPerdida = constrain(c.perdida, 0.1f, 0.8f);
  Serial.println(F("# config control cargada"));
}

// ===================== PROCESAMIENTO DE IMAGEN (RGB565 -> POSICION BOLA) =====================
// NOTA: camera_fb_t en OV2640 RGB565 entrega bytes big-endian (byte alto primero).
inline void descomponerRgb565(uint16_t pbe, uint8_t &r, uint8_t &g, uint8_t &b, uint8_t &luma) {
  uint16_t p = (pbe >> 8) | (pbe << 8);
  r = ((p >> 11) & 0x1F) << 3;
  g = ((p >> 5) & 0x3F) << 2;
  b = (p & 0x1F) << 3;
  luma = static_cast<uint8_t>((r + (g << 1) + b) >> 2);
}

inline uint16_t empaquetarRgb565BE(uint8_t r, uint8_t g, uint8_t b) {
  uint16_t p = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  return (p >> 8) | (p << 8);
}

bool dentroCal(int rx, int ry) {
  float tx = 0.05f * (X_FIN - X_INI), ty = 0.05f * (Y_FIN - Y_INI);
  return rx >= X_INI - tx && rx <= X_FIN + tx && ry >= Y_INI - ty && ry <= Y_FIN + ty;
}

float aMetros(int raw, int ini, int fin, float L, bool inv) {
  float norm = (float)(raw - ini) / (float)max(1, fin - ini) - 0.5f;
  if (inv) norm = -norm;
  return norm * L;
}

// Evalua cuanto pertenece un pixel a la bola (>0 si supera el umbral)
inline int puntajePixel(uint8_t r, uint8_t g, uint8_t b, uint8_t luma, uint8_t bgLuma) {
  int score = 0;
  if (modoVision == 0) {
    // Bola clara sobre plato oscuro o diferencia positiva respecto al fondo
    int ref = (usarFondo && fondoValido) ? (int)bgLuma + (int)umbralVision : (int)umbralVision + 100;
    score = (int)luma - ref;
  } else if (modoVision == 1) {
    // Bola oscura sobre plato claro
    int ref = (usarFondo && fondoValido) ? (int)bgLuma - (int)umbralVision : 160 - (int)umbralVision;
    score = ref - (int)luma;
  } else {
    // Color RGB aprendido (distancia L1 en cromaticidad/RGB)
    int dist = (abs((int)r - (int)colorRef[0]) + abs((int)g - (int)colorRef[1]) + abs((int)b - (int)colorRef[2])) / 3;
    if (usarFondo && fondoValido && abs((int)luma - (int)bgLuma) < 12) return 0;
    score = (int)umbralVision - dist;
  }
  return score > 0 ? score : 0;
}

void ejecutarAprendizaje(const uint16_t *buf) {
  // Compara ventana central de 16x16 (bola) contra anillo de 44..70 px alrededor (plato)
  int cx = constrain((X_INI + X_FIN) / 20, 40, camhw::ancho - 41);
  int cy = constrain((Y_INI + Y_FIN) / 20, 40, camhw::alto - 41);
  long sumR = 0, sumG = 0, sumB = 0, sumL = 0, nBola = 0;
  long sumPlatoL = 0, nPlato = 0;
  for (int y = cy - 36; y <= cy + 36; y++) {
    for (int x = cx - 36; x <= cx + 36; x++) {
      uint8_t r, g, b, l;
      descomponerRgb565(buf[y * camhw::ancho + x], r, g, b, l);
      int dx = abs(x - cx), dy = abs(y - cy);
      if (dx <= 7 && dy <= 7) {
        sumR += r; sumG += g; sumB += b; sumL += l; nBola++;
      } else if (dx >= 22 || dy >= 22) {
        sumPlatoL += l; nPlato++;
      }
    }
  }
  if (nBola > 0 && nPlato > 0) {
    int lBola = sumL / nBola;
    int lPlato = sumPlatoL / nPlato;
    colorRef[0] = sumR / nBola;
    colorRef[1] = sumG / nBola;
    colorRef[2] = sumB / nBola;
    int sat = max(colorRef[0], max(colorRef[1], colorRef[2])) - min(colorRef[0], min(colorRef[1], colorRef[2]));
    int diffL = lBola - lPlato;
    if (sat > 55 && abs(diffL) < 35) {
      modoVision = 2;
      umbralVision = 45;
    } else if (diffL >= 0) {
      modoVision = 0;
      umbralVision = constrain((usarFondo && fondoValido) ? max(18, diffL / 2) : max(15, (lBola + lPlato) / 2 - 100), 10, 140);
    } else {
      modoVision = 1;
      umbralVision = constrain((usarFondo && fondoValido) ? max(18, -diffL / 2) : max(15, 160 - (lBola + lPlato) / 2), 10, 140);
    }
    Serial.printf("# APRENDER listo: bola L=%d RGB=(%u,%u,%u) vs plato L=%d -> modo=%u umbral=%u\n",
                  lBola, colorRef[0], colorRef[1], colorRef[2], lPlato, modoVision, umbralVision);
  }
}

void detectarBolaEnFrame(const uint16_t *buf, bool &valida, int &rxOut, int &ryOut, int &pxOut, int &pyOut,
                         uint16_t &areaOut, uint8_t &picoOut) {
  int x0 = calibrando ? 8 : constrain(X_INI / 10 - 10, 4, camhw::ancho - 5);
  int x1 = calibrando ? (camhw::ancho - 9) : constrain(X_FIN / 10 + 10, 4, camhw::ancho - 5);
  int y0 = calibrando ? 8 : constrain(Y_INI / 10 - 10, 4, camhw::alto - 5);
  int y1 = calibrando ? (camhw::alto - 9) : constrain(Y_FIN / 10 + 10, 4, camhw::alto - 5);

  // Paso 1: grilla gruesa de 8x6 celdas para ubicar la concentracion principal de la bola
  const int NX = 8, NY = 6;
  int cw = max(8, (x1 - x0 + 1) / NX);
  int ch = max(8, (y1 - y0 + 1) / NY);
  long cellSum[NY][NX] = {};
  int maxScore = 0;

  for (int y = y0; y <= y1; y += 2) {
    int cy = min(NY - 1, (y - y0) / ch);
    int row = y * camhw::ancho;
    for (int x = x0; x <= x1; x += 2) {
      uint8_t r, g, b, l;
      descomponerRgb565(buf[row + x], r, g, b, l);
      uint8_t bg = (fondoValido && fondoLuma) ? fondoLuma[row + x] : 128;
      int sc = puntajePixel(r, g, b, l, bg);
      if (sc > 0) {
        int cx = min(NX - 1, (x - x0) / cw);
        cellSum[cy][cx] += sc;
        if (sc > maxScore) maxScore = sc;
      }
    }
  }

  // Busca la mejor region 2x2 celdas
  long bestReg = 0;
  int bestCx = x0 + (x1 - x0) / 2, bestCy = y0 + (y1 - y0) / 2;
  for (int iy = 0; iy < NY; iy++) {
    for (int ix = 0; ix < NX; ix++) {
      long s = cellSum[iy][ix];
      if (ix + 1 < NX) s += cellSum[iy][ix + 1];
      if (iy + 1 < NY) s += cellSum[iy + 1][ix];
      if (ix + 1 < NX && iy + 1 < NY) s += cellSum[iy + 1][ix + 1];
      if (s > bestReg) {
        bestReg = s;
        bestCx = x0 + (ix + 1) * cw;
        bestCy = y0 + (iy + 1) * ch;
      }
    }
  }

  if (bestReg < 40) {
    valida = false; areaOut = 0; picoOut = 0; rxOut = ryOut = pxOut = pyOut = -1;
    return;
  }

  // Paso 2: centroide subpixel dentro de una ventana local de +-38 px alrededor del pico
  const int RAD = 38;
  int wx0 = max(x0, bestCx - RAD), wx1 = min(x1, bestCx + RAD);
  int wy0 = max(y0, bestCy - RAD), wy1 = min(y1, bestCy + RAD);
  int64_t sumW = 0, sumWX = 0, sumWY = 0;
  uint16_t count = 0;

  for (int y = wy0; y <= wy1; y++) {
    int row = y * camhw::ancho;
    for (int x = wx0; x <= wx1; x++) {
      uint8_t r, g, b, l;
      descomponerRgb565(buf[row + x], r, g, b, l);
      uint8_t bg = (fondoValido && fondoLuma) ? fondoLuma[row + x] : 128;
      int sc = puntajePixel(r, g, b, l, bg);
      if (sc > 0) {
        sumW += sc;
        sumWX += (int64_t)sc * x;
        sumWY += (int64_t)sc * y;
        count++;
      }
    }
  }

  areaOut = count;
  picoOut = (uint8_t)min(255, maxScore);
  if (count < areaMin || count > areaMax || sumW <= 0) {
    valida = false; rxOut = ryOut = pxOut = pyOut = -1;
    return;
  }

  // Coordenadas en decimas de pixel
  rxOut = (int)((sumWX * 10 + sumW / 2) / sumW);
  ryOut = (int)((sumWY * 10 + sumW / 2) / sumW);
  pxOut = rxOut / 10;
  pyOut = ryOut / 10;
  valida = true;
}

void tareaCaptura(void *) {
  uint32_t inicioVentana = millis(), enVentana = 0;
  for (;;) {
    const int64_t t0 = esp_timer_get_time();
    camera_fb_t *fb = esp_camera_fb_get();
    const int64_t t1 = esp_timer_get_time();

    if (!fb || fb->len < (size_t)(camhw::ancho * camhw::alto * 2)) {
      if (xSemaphoreTake(mutexCam, pdMS_TO_TICKS(50)) == pdTRUE) {
        vis.fallos++;
        xSemaphoreGive(mutexCam);
      }
      if (fb) esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    const auto *pixels = reinterpret_cast<const uint16_t *>(fb->buf);
    if (pedirFondo && fondoLuma) {
      for (int i = 0; i < camhw::ancho * camhw::alto; i++) {
        uint8_t r, g, b, l;
        descomponerRgb565(pixels[i], r, g, b, l);
        fondoLuma[i] = l;
      }
      fondoValido = true;
      usarFondo = true;
      pedirFondo = false;
      Serial.println(F("# FONDO capturado y activado (usar_fondo 1)"));
    }
    if (pedirAprender) {
      ejecutarAprendizaje(pixels);
      pedirAprender = false;
    }

    bool val = false;
    int rx = -1, ry = -1, pxx = -1, pxy = -1;
    uint16_t area = 0;
    uint8_t pico = 0;
    detectarBolaEnFrame(pixels, val, rx, ry, pxx, pxy, area, pico);
    const int64_t t2 = esp_timer_get_time();

    if (xSemaphoreTake(mutexCam, pdMS_TO_TICKS(50)) == pdTRUE) {
      memcpy(ultimoFrameRgb, pixels, camhw::ancho * camhw::alto * 2);
      vis.recibidas++;
      vis.seq++;
      enVentana++;
      vis.capturaUs = static_cast<uint32_t>(t1 - t0);
      vis.visionUs = static_cast<uint32_t>(t2 - t1);
      vis.tomadaUs = t1;
      vis.rawX = rx; vis.rawY = ry;
      vis.pxX = pxx; vis.pxY = pxy;
      vis.area = area; vis.picoScore = pico;
      if (val && dentroCal(rx, ry)) {
        vis.valida = true;
        vis.detectadas++;
        vis.posX = aMetros(rx, X_INI, X_FIN, LX, invertirX);
        vis.posY = aMetros(ry, Y_INI, Y_FIN, LY, invertirY);
      } else {
        vis.valida = false;
      }
      const uint32_t transcurrido = millis() - inicioVentana;
      if (transcurrido >= 2000) {
        vis.fps = enVentana * 1000.0f / transcurrido;
        enVentana = 0;
        inicioVentana = millis();
      }
      xSemaphoreGive(mutexCam);
    }
    esp_camera_fb_return(fb);
    vTaskDelay(1);
  }
}

// Lee el ultimo estado de la camara; devuelve si hay deteccion valida y si es un frame nuevo
bool medir(bool &esFrameNuevo) {
  esFrameNuevo = false;
  if (!mutexCam || xSemaphoreTake(mutexCam, pdMS_TO_TICKS(5)) != pdTRUE) return false;
  MedidaVision m = vis;
  xSemaphoreGive(mutexCam);

  if (m.seq != ultSeqMuestra) {
    esFrameNuevo = true;
    ultSeqMuestra = m.seq;
  }
  toqueAct = (m.rawX >= 0 && m.rawY >= 0);
  rawX = m.rawX;
  rawY = m.rawY;
  areaAct = m.area;
  validaAct = m.valida;
  if (validaAct) {
    posX = m.posX;
    posY = m.posY;
    if (esFrameNuevo) tUltValida = millis();
  }
  return validaAct;
}

bool medirSimple() {
  bool nuevo = false;
  return medir(nuevo);
}

// Espera al proximo frame nuevo de camara (util en rutinas de prueba y nivelado)
bool medirEsperandoFrame(uint32_t timeoutMs = 60) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    bool nuevo = false;
    bool val = medir(nuevo);
    if (nuevo) return val;
    delay(2);
  }
  return medirSimple();
}

// ===================== GENERACION DE JPEG ANOTADO (USB / WIFI) =====================
bool generarJpegAnotado(uint8_t **jpgBuf, size_t &jpgLen) {
  *jpgBuf = nullptr;
  jpgLen = 0;
  auto *copia = static_cast<uint16_t *>(heap_caps_malloc(camhw::ancho * camhw::alto * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!copia) return false;
  MedidaVision m;
  if (xSemaphoreTake(mutexCam, pdMS_TO_TICKS(100)) != pdTRUE) {
    free(copia);
    return false;
  }
  if (vis.recibidas == 0) {
    xSemaphoreGive(mutexCam);
    free(copia);
    return false;
  }
  memcpy(copia, ultimoFrameRgb, camhw::ancho * camhw::alto * 2);
  m = vis;
  xSemaphoreGive(mutexCam);

  // Dibuja el rectangulo de la ROI del plato en cian
  int xi = constrain(X_INI / 10, 0, camhw::ancho - 1);
  int xf = constrain(X_FIN / 10, 0, camhw::ancho - 1);
  int yi = constrain(Y_INI / 10, 0, camhw::alto - 1);
  int yf = constrain(Y_FIN / 10, 0, camhw::alto - 1);
  uint16_t colRoi = empaquetarRgb565BE(0, 220, 255);
  for (int x = xi; x <= xf; x++) {
    copia[yi * camhw::ancho + x] = colRoi;
    copia[yf * camhw::ancho + x] = colRoi;
  }
  for (int y = yi; y <= yf; y++) {
    copia[y * camhw::ancho + xi] = colRoi;
    copia[y * camhw::ancho + xf] = colRoi;
  }

  // Dibuja cruz sobre la bola detectada (verde si valida, amarillo si fuera de ROI)
  if (m.pxX >= 0 && m.pxY >= 0) {
    uint16_t colBola = m.valida ? empaquetarRgb565BE(0, 255, 60) : empaquetarRgb565BE(255, 220, 0);
    for (int d = -8; d <= 8; d++) {
      int xx = constrain(m.pxX + d, 0, camhw::ancho - 1);
      int yy = constrain(m.pxY + d, 0, camhw::alto - 1);
      copia[m.pxY * camhw::ancho + xx] = colBola;
      copia[yy * camhw::ancho + m.pxX] = colBola;
    }
  }

  bool ok = fmt2jpg(reinterpret_cast<uint8_t *>(copia), camhw::ancho * camhw::alto * 2,
                    camhw::ancho, camhw::alto, PIXFORMAT_RGB565, 82, jpgBuf, &jpgLen);
  free(copia);
  return ok && *jpgBuf && jpgLen > 0;
}

String estadoJson() {
  MedidaVision m;
  if (xSemaphoreTake(mutexCam, pdMS_TO_TICKS(50)) == pdTRUE) {
    m = vis;
    xSemaphoreGive(mutexCam);
  }
  const int64_t edad = m.tomadaUs ? (esp_timer_get_time() - m.tomadaUs) / 1000 : -1;
  String s = "{\"version\":\"camara-diag-0.2\",\"modo_control\":\"ball_plate_camara_fuzzy\",\"frames\":";
  s += m.recibidas;
  s += ",\"fallos\":"; s += m.fallos;
  s += ",\"detectadas\":"; s += m.detectadas;
  s += ",\"fps_captura\":"; s += String(m.fps, 2);
  s += ",\"captura_ms\":"; s += String(m.capturaUs / 1000.0f, 2);
  s += ",\"vision_ms\":"; s += String(m.visionUs / 1000.0f, 2);
  s += ",\"edad_foto_ms\":"; s += String(static_cast<long>(edad));
  s += ",\"bola_valida\":"; s += (m.valida ? "true" : "false");
  s += ",\"bola_px\":["; s += m.pxX; s += ","; s += m.pxY; s += "]";
  s += ",\"bola_cm\":["; s += String(m.posX * 100.0f, 2); s += ","; s += String(m.posY * 100.0f, 2); s += "]";
  s += ",\"area_px\":"; s += m.area;
  s += ",\"servo_deg\":["; s += String(servoAct[0], 1); s += ","; s += String(servoAct[1], 1); s += "]";
  s += ",\"flash_bytes\":"; s += ESP.getFlashChipSize();
  s += ",\"psram_bytes\":"; s += ESP.getPsramSize();
  s += ",\"psram_libre\":"; s += ESP.getFreePsram();
  s += ",\"servos\":\""; s += (servosActivos ? "activos" : "desactivados"); s += "\"}";
  return s;
}

void enviarFotoWeb() {
  uint8_t *jpg = nullptr;
  size_t len = 0;
  if (!generarJpegAnotado(&jpg, len)) {
    web.send(503, "text/plain", "Todavia no hay una foto valida.");
    return;
  }
  web.sendHeader("Cache-Control", "no-store");
  web.sendHeader("Content-Disposition", "inline; filename=foto.jpg");
  web.send_P(200, "image/jpeg", reinterpret_cast<const char *>(jpg), len);
  free(jpg);
}

void fotoSerie() {
  uint8_t *jpg = nullptr;
  size_t len = 0;
  if (!generarJpegAnotado(&jpg, len)) {
    Serial.println("# ERROR_FOTO sin imagen valida");
    return;
  }
  Serial.printf("# FOTO_JPEG %u\n", static_cast<unsigned>(len));
  Serial.write(jpg, len);
  Serial.flush();
  Serial.println("\n# FIN_FOTO");
  free(jpg);
}

void enviarPaginaWeb() {
  static const char pagina[] PROGMEM = R"HTML(<!doctype html><html lang="es"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Ball & Plate Vision S3</title>
<style>body{font:16px system-ui;max-width:780px;margin:20px auto;padding:0 14px;background:#f8fafc;color:#0f172a}img{width:100%;border-radius:8px;border:1px solid #cbd5e1}pre{background:#fff;padding:12px;border-radius:8px;white-space:pre-wrap;font-size:13px;border:1px solid #e2e8f0}</style>
<h2>Ball & Plate — Visión + Control Fuzzy (ESP32-S3)</h2>
<p>Recuadro cian = ROI del plato | Cruz verde = bola detectada</p>
<img id="foto" alt="Cargando cámara..."><pre id="estado">Cargando estado...</pre>
<script>let busy=false;function f(){document.getElementById('foto').src='/foto.jpg?t='+Date.now()}
async function e(){if(busy)return;busy=true;try{let r=await fetch('/estado',{cache:'no-store'});document.getElementById('estado').textContent=JSON.stringify(await r.json(),null,2)}catch(_){}finally{busy=false}}
f();e();setInterval(f,600);setInterval(e,1000);</script></html>)HTML";
  web.send_P(200, "text/html; charset=utf-8", pagina);
}

void iniciarWeb() {
  if (webActiva) { Serial.println("# WiFi ya activo."); return; }
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(RED, CLAVE)) { Serial.println("# ERROR al iniciar WiFi AP."); return; }
  web.on("/", HTTP_GET, enviarPaginaWeb);
  web.on("/foto.jpg", HTTP_GET, enviarFotoWeb);
  web.on("/estado", HTTP_GET, []() { web.sendHeader("Cache-Control", "no-store"); web.send(200, "application/json", estadoJson()); });
  web.begin();
  webActiva = true;
  Serial.printf("# Red: %s | clave: %s | abre http://%s\n", RED, CLAVE, WiFi.softAPIP().toString().c_str());
}

// ===================== SERVOS (LEDC canales 2 y 3) =====================
void servoUs(uint8_t k, float us) {
  if (!servosActivos || pinConectado[k] < 0) return;
  uint32_t duty = (uint32_t)(us * ((1UL << RES_SERVO) - 1) / (1000000.0f / FREQ_SERVO));
  ledcWrite(pinConectado[k], duty);
}

void tareaServo(void *p) {
  uint8_t k = (uint8_t)(uintptr_t)p;
  float ult = -1000.0f;
  for (;;) {
    float u = servoObj[k];
    if (!pausaServos && servosActivos && u != ult) {
      servoUs(k, US_MIN + u * (US_MAX - US_MIN) / 180.0f);
      ult = u;
    }
    vTaskDelay(1);
  }
}

void iniciarTareasServo() {
  xTaskCreatePinnedToCore(tareaServo, "servoX", 2048, (void *)0, 1, NULL, 0);
  xTaskCreatePinnedToCore(tareaServo, "servoY", 2048, (void *)1, 1, NULL, 0);
}

void escribir(uint8_t k, float deg) {
  deg = constrain(deg, ej[k].centro - SMAX_ABS, ej[k].centro + SMAX_ABS);
  deg = constrain(deg, 0.0f, 180.0f);
  servoAct[k] = deg;
  servoObj[k] = deg;
}

void conectarServos() {
  bool estaba = pausaServos;
  pausaServos = true;
  delay(35);
  uint8_t pin[2] = {PIN_SX, PIN_SY};
  for (uint8_t k = 0; k < 2; k++) {
    if (pinConectado[k] >= 0) {
      ledcDetach(pinConectado[k]);
      pinConectado[k] = -1;
    }
  }
  if (servosActivos) {
    for (uint8_t k = 0; k < 2; k++) {
      if (ledcAttachChannel(pin[k], FREQ_SERVO, RES_SERVO, CANAL_SERVO[k])) {
        pinConectado[k] = pin[k];
      }
    }
  }
  pausaServos = estaba;
}

void nivelar() {
  escribir(0, ej[0].centro);
  escribir(1, ej[1].centro);
}

// ===================== ESTADOS Y CONTROL =====================
void detener(const char *razon) {
  centrando = false;
  estado = DETENIDO;
  motivo = razon;
  buenas = 0;
  tSinBola = 0;
  ej[0].reiniciar();
  ej[1].reiniciar();
  nivelar();
  Serial.print(F("# DETENIDO: "));
  Serial.println(razon);
}

void arrancar(float zx, float zy, float dt) {
  float z[2] = {zx, zy};
  for (int k = 0; k < 2; k++) {
    ej[k].reiniciar();
    ej[k].ob.corregir(z[k], dt);
  }
  estado = ACTIVO;
  tSinBola = 0;
  usPeriodoMax = 0;
  cortes = 0;
  circT0 = millis();
  usCtrlMax = usServoMax = usSerieMax = usLecturaMax = 0;
  Serial.printf("# CONTROL ACTIVO (%s)%s\n", ejes == 3 ? "xy" : (ejes == 1 ? "x" : "y"),
                ej[0].usarObs ? " + observador" : "");
}

void referencia(float sp[2], float vsp[2], float asp[2]) {
  sp[0] = spX; sp[1] = spY; vsp[0] = vsp[1] = asp[0] = asp[1] = 0;
  if (circR <= 0) return;
  float t = (millis() - circT0) * 1e-3f, w = 2 * PI / circT, r = circR * min(1.0f, t / 1.0f);
  sp[0] += r * sinf(w * t);  vsp[0] = r * w * cosf(w * t); asp[0] = -r * w * w * sinf(w * t);
  sp[1] += -r * cosf(w * t); vsp[1] = r * w * sinf(w * t); asp[1] = r * w * w * cosf(w * t);
}

// ===================== PRUEBAS Y NIVELADO AUTOMATICO =====================
void prueba(uint8_t k, float g) {
  const char *n = k ? "y" : "x";
  nivelar();
  delay(350);
  float tt[8], px[8], py[8];
  uint8_t m = 0;
  uint32_t ta = millis();
  for (uint8_t i = 0; i < 30 && m < 8; i++) {
    if (medirEsperandoFrame(45)) {
      tt[m] = (millis() - ta) * 1e-3f;
      px[m] = posX;
      py[m] = posY;
      m++;
    }
  }
  if (m < 5) {
    Serial.printf("# no veo la bola (%u lecturas buenas de 30). Usa 'aprender' o 'panel' con la bola en el centro\n", m);
    return;
  }
  float ox[8], oy[8];
  for (uint8_t i = 0; i < m; i++) { ox[i] = px[i]; oy[i] = py[i]; }
  for (uint8_t i = 1; i < m; i++) for (uint8_t j = i; j > 0 && ox[j] < ox[j - 1]; j--) { float a = ox[j]; ox[j] = ox[j - 1]; ox[j - 1] = a; }
  for (uint8_t i = 1; i < m; i++) for (uint8_t j = i; j > 0 && oy[j] < oy[j - 1]; j--) { float a = oy[j]; oy[j] = oy[j - 1]; oy[j - 1] = a; }
  float medX = ox[m / 2], medY = oy[m / 2];
  float st = 0, stt = 0, sp[2] = {0, 0}, stp[2] = {0, 0};
  uint8_t n2 = 0;
  for (uint8_t i = 0; i < m; i++) {
    if (fabsf(px[i] - medX) > 0.015f || fabsf(py[i] - medY) > 0.015f) continue;
    float t = tt[i];
    st += t; stt += t * t; sp[0] += px[i]; sp[1] += py[i]; stp[0] += t * px[i]; stp[1] += t * py[i];
    n2++;
  }
  if (n2 < 4) { Serial.println(F("# lecturas muy dispersas: revisa iluminacion/umbral con 'panel'")); return; }
  m = n2;
  float den = m * stt - st * st, v0[2], p0[2];
  for (int i = 0; i < 2; i++) {
    v0[i] = (m * stp[i] - st * sp[i]) / den;
    p0[i] = (sp[i] - v0[i] * st) / m;
  }
  if (fabsf(v0[0]) > 0.06f || fabsf(v0[1]) > 0.06f) {
    Serial.println(F("# la bola rueda sola rapido: nivela primero (cx / cy) y repite"));
    return;
  }
  float tPulso = (millis() - ta) * 1e-3f;
  escribir(k, ej[k].centro + g);
  uint32_t t0 = millis();
  bool vio = false;
  float p1[2] = {0, 0}, t1 = 0;
  while (millis() - t0 < 450) {
    if (medirEsperandoFrame(35)) {
      p1[0] = posX; p1[1] = posY; t1 = (millis() - ta) * 1e-3f; vio = true;
    }
  }
  nivelar();
  if (!vio || t1 < tPulso + 0.22f) {
    Serial.println(F("# perdi la bola durante la prueba: recolocala y repite"));
    return;
  }
  float d[2];
  for (int i = 0; i < 2; i++) d[i] = (p1[i] - (p0[i] + v0[i] * t1)) * 100.0f;
  float dk = d[k], dO = d[1 - k];
  Serial.printf("# servo %s %+.0f grados: la bola se desvio %+.1f cm en %s y %+.1f cm en el otro eje\n", n, g, dk, n, dO);
  if (fabsf(dk) < 1.0f && fabsf(dO) < 1.0f) {
    Serial.println(F("# casi no se movio. Revisa: servos con alimentacion, bielas y 'servos 1'"));
  } else if (min(fabsf(dk), fabsf(dO)) > 0.6f * max(fabsf(dk), fabsf(dO))) {
    Serial.println(F("# ambiguo: la bola se fue en diagonal. Nivela mejor (cx / cy) y repite"));
  } else if (fabsf(dO) > fabsf(dk)) {
    Serial.println(F("# ese servo mueve la bola en el OTRO eje: escribe 'intercambiar' y repite las pruebas"));
  } else {
    ej[k].signo = ((dk > 0) == (g > 0)) ? 1.0f : -1.0f;
    signoOk[k] = true;
    Serial.printf("# signo %s = %+.0f  (listo; 'guardar' para conservarlo)\n", n, ej[k].signo);
  }
}

const uint8_t N_TIRO = 120;
float tTiro[N_TIRO], xTiro[N_TIRO], yTiro[N_TIRO];

float aceleracionParabola(const float *t, const float *x, uint8_t n) {
  double tm = 0;
  for (uint8_t i = 0; i < n; i++) tm += t[i];
  tm /= n;
  double S[5] = {0, 0, 0, 0, 0}, B[3] = {0, 0, 0};
  for (uint8_t i = 0; i < n; i++) {
    double u = t[i] - tm, p = 1;
    for (int k = 0; k < 5; k++) { S[k] += p; if (k < 3) B[k] += p * x[i]; p *= u; }
  }
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

int medirSoltada(float a[2], const char *etiqueta) {
  Serial.printf("# %s: pon la bola en el centro y SOSTENLA quieta\n", etiqueta);
  double sx = 0, sy = 0;
  uint16_t m = 0;
  uint32_t tq = millis(), t0 = millis();
  float ref[2] = {0, 0};
  while (true) {
    if (cancelado()) return -1;
    if (millis() - t0 > 30000) { Serial.println(F("# no vi la bola quieta en 30 s")); return -1; }
    if (medirEsperandoFrame(50)) {
      if (m && (fabsf(posX - sx / m) > 0.008f || fabsf(posY - sy / m) > 0.008f)) {
        sx = sy = 0; m = 0; tq = millis();
      }
      sx += posX; sy += posY; m++;
      if (millis() - tq >= 450 && m >= 10) break;
    } else {
      sx = sy = 0; m = 0; tq = millis();
    }
  }
  ref[0] = sx / m; ref[1] = sy / m;
  Serial.println(F("# SUELTALA (retira la mano rapido para no tapar la camara)"));
  t0 = millis();
  uint8_t fuera = 0;
  float tPrev = 0, xPrev = 0, yPrev = 0;
  uint32_t tIni = micros();
  while (true) {
    if (cancelado()) return -1;
    if (millis() - t0 > 3000) {
      a[0] = a[1] = 0;
      Serial.println(F("#   no rodo en 3 s: placa casi horizontal"));
      return 1;
    }
    uint32_t us = micros();
    if (medirEsperandoFrame(45)) {
      bool lejos = fabsf(posX - ref[0]) > 0.008f || fabsf(posY - ref[1]) > 0.008f;
      if (lejos && ++fuera >= 2) break;
      if (!lejos) fuera = 0;
      tPrev = (us - tIni) * 1e-6f; xPrev = posX; yPrev = posY;
    }
  }
  uint8_t n = 0;
  uint32_t tUlt = millis();
  tTiro[n] = tPrev; xTiro[n] = xPrev; yTiro[n] = yPrev; n++;
  tTiro[n] = (micros() - tIni) * 1e-6f; xTiro[n] = posX; yTiro[n] = posY; n++;
  float tFin = tTiro[1] + 0.65f;
  while (n < N_TIRO) {
    uint32_t us = micros();
    float t = (us - tIni) * 1e-6f;
    if (t > tFin) break;
    if (medirEsperandoFrame(45)) {
      tTiro[n] = t; xTiro[n] = posX; yTiro[n] = posY; n++; tUlt = millis();
      if (fabsf(posX - ref[0]) > 0.055f || fabsf(posY - ref[1]) > 0.055f) break;
    } else if (millis() - tUlt > 150) break;
  }
  float dur = tTiro[n - 1] - tTiro[0];
  if (n < 8 || dur < 0.12f) {
    Serial.printf("#   pocos datos (%u lecturas en %.2f s): repite\n", n, dur);
    return 0;
  }
  a[0] = aceleracionParabola(tTiro, xTiro, n);
  a[1] = aceleracionParabola(tTiro, yTiro, n);
  Serial.printf("#   %u lecturas en %.2f s: aceleracion x %+.1f  y %+.1f cm/s2\n", n, dur, a[0] * 100, a[1] * 100);
  return 1;
}

bool tirada(float a[2], const char *etiqueta) {
  for (int k = 0; k < 3; k++) {
    int r = medirSoltada(a, etiqueta);
    if (r == 1) return true;
    if (r < 0) return false;
  }
  return false;
}

void autoNivel() {
  const float D = 6.0f;
  const float paso[4][2] = {{+D, 0}, {-D, 0}, {0, +D}, {0, -D}};
  float c0[2] = {ej[0].centro, ej[1].centro}, a[4][2];
  for (int i = 0; i < 4; i++) {
    escribir(0, c0[0] + paso[i][0]);
    escribir(1, c0[1] + paso[i][1]);
    delay(350);
    char et[56];
    snprintf(et, sizeof et, "[%d/4] servo X %+.0f, servo Y %+.0f", i + 1, paso[i][0], paso[i][1]);
    if (!tirada(a[i], et)) { nivelar(); return; }
  }
  float G[2][2], a0[2];
  for (int i = 0; i < 2; i++) {
    G[i][0] = (a[0][i] - a[1][i]) / (2 * D);
    G[i][1] = (a[2][i] - a[3][i]) / (2 * D);
    a0[i] = (a[0][i] + a[1][i] + a[2][i] + a[3][i]) / 4;
  }
  bool cruz = fabsf(G[1][0]) + fabsf(G[0][1]) > 2 * (fabsf(G[0][0]) + fabsf(G[1][1]));
  if (cruz) {
    uint8_t t = PIN_SX; PIN_SX = PIN_SY; PIN_SY = t;
    conectarServos();
    float c = c0[0]; c0[0] = c0[1]; c0[1] = c;
    for (int i = 0; i < 2; i++) { float g = G[i][0]; G[i][0] = G[i][1]; G[i][1] = g; }
    Serial.printf("# servos cruzados corregidos: X=GPIO%u, Y=GPIO%u\n", PIN_SX, PIN_SY);
  }
  float det = G[0][0] * G[1][1] - G[0][1] * G[1][0], dc[2];
  if (fabsf(det) > 1e-9f) {
    dc[0] = (-a0[0] * G[1][1] + a0[1] * G[0][1]) / det;
    dc[1] = (-a0[1] * G[0][0] + a0[0] * G[1][0]) / det;
  } else {
    dc[0] = -a0[0] / G[0][0];
    dc[1] = -a0[1] / G[1][1];
  }
  for (int k = 0; k < 2; k++) {
    dc[k] = constrain(dc[k], -15.0f, 15.0f);
    ej[k].centro = constrain(c0[k] + dc[k], 30.0f, 150.0f);
    ej[k].signo = G[k][k] > 0 ? 1.0f : -1.0f;
    signoOk[k] = true;
  }
  nivelar();
  Serial.printf("# NIVEL listo: cx %.1f (%+.1f) cy %.1f (%+.1f) | signo x %+.0f y %+.0f. Escribe 'guardar'\n",
                ej[0].centro, dc[0], ej[1].centro, dc[1], ej[0].signo, ej[1].signo);
}

void diagnosticoCamara() {
  uint8_t ok = 0, n = 40;
  Serial.println(F("# camara: 2 s de prueba (pxX pxY area -> posX posY cm | valida)"));
  for (uint8_t i = 0; i < n; i++) {
    bool v = medirEsperandoFrame(55);
    if (v) ok++;
    if (i % 5 == 0) {
      Serial.printf("#  px=(%.1f, %.1f) area=%u -> x=%+.2f y=%+.2f cm | %s\n",
                    rawX * 0.1f, rawY * 0.1f, areaAct, posX * 100.0f, posY * 100.0f, v ? "SI" : "NO");
    }
  }
  Serial.printf("# resultado vision: %u/%u frames con bola valida (FPS %.1f, modo %u, umbral %u)\n",
                ok, n, vis.fps, modoVision, umbralVision);
  if (ok == 0) {
    Serial.println(F("#   No se detecto la bola. Pon la bola en el centro y escribe 'aprender', o revisa 'foto'/'wifi'"));
  }
}

// ===================== TELEMETRIA Y AYUDA =====================
void ver() {
  const char *est[3] = {"DETENIDO", "ESPERANDO bola", "ACTIVO"};
  const char *nomModo[3] = {"0:claro", "1:oscuro", "2:color"};
  Serial.printf("# estado %s (%s) | ejes %s | servos %s (X=GPIO%u Y=GPIO%u)\n",
                est[estado], motivo, ejes == 3 ? "xy" : (ejes == 1 ? "x" : "y"),
                servosActivos ? "ACTIVOS" : "DESACTIVADOS", PIN_SX, PIN_SY);
  Serial.printf("# vision (%s, umbral %u, fondo %s, area %u..%u): valida %d px=(%.1f,%.1f) area=%u -> x %+.2f y %+.2f cm\n",
                nomModo[modoVision], umbralVision, (usarFondo && fondoValido) ? "SI" : "NO",
                areaMin, areaMax, validaAct, rawX * 0.1f, rawY * 0.1f, areaAct, posX * 100.0f, posY * 100.0f);
  Serial.printf("# ROI plato: X %.1f..%.1f px  Y %.1f..%.1f px (%.1f x %.1f cm) %s | FPS %.1f (cap %.1f ms, vis %.2f ms)\n",
                X_INI * 0.1f, X_FIN * 0.1f, Y_INI * 0.1f, Y_FIN * 0.1f, LX * 100.0f, LY * 100.0f,
                panelCal ? "(calibrada)" : "(por defecto)", vis.fps, vis.capturaUs / 1000.0f, vis.visionUs / 1000.0f);
  Serial.printf("# lazo: periodo %lu us (max %lu) | cortes %lu | rechazos obs %lu %lu\n",
                (unsigned long)usPeriodo, (unsigned long)usPeriodoMax, (unsigned long)cortes,
                ej[0].ob.nRechazos, ej[1].ob.nRechazos);
  for (int k = 0; k < 2; k++) {
    Serial.printf("# eje %c: centro %.1f  signo %+.0f %s  servo %.1f  d %+.1f cm/s2\n",
                  k ? 'Y' : 'X', ej[k].centro, ej[k].signo, signoOk[k] ? "(ok)" : "(SIN PROBAR: prueba)",
                  servoAct[k], ej[k].ob.d * 100.0f);
  }
  Serial.printf("# fuzzy E %.1f cm V %.1f cm/s A %.0f cm/s2 | K x %.2f y %.2f | smax %.0f | retardo x %.0f y %.0f ms\n",
                esc.E * 100.0f, esc.V * 100.0f, esc.A * 100.0f, ej[0].K * 100.0f, ej[1].K * 100.0f,
                ej[0].smax, ej[0].retardo * 1000.0f, ej[1].retardo * 1000.0f);
  Serial.println(estadoJson());
}

void ayuda() {
  Serial.println(F("# --- COMANDOS BALL & PLATE CON CAMARA ESP32-S3 ---"));
  Serial.println(F("# ver | stop | iniciar [x|y|xy] | p 0|1|2 | guardar | fabrica"));
  Serial.println(F("# panel / camara       diagnostico de 2 s de camara y deteccion"));
  Serial.println(F("# fondo                (sin bola) captura fondo del plato para restar reflejos"));
  Serial.println(F("# usar_fondo 0|1       activa/desactiva resta de fondo"));
  Serial.println(F("# aprender             (con bola en el centro) auto-ajusta modo y umbral"));
  Serial.println(F("# modo 0|1|2           0=bola clara, 1=bola oscura, 2=color | umbral 45 | area 12 4500"));
  Serial.println(F("# cal / cal fin        calibra bordes paseando la bola por las 4 esquinas"));
  Serial.println(F("# roi xi xf yi yf      define bordes del plato en pixeles (0..319, 0..239)"));
  Serial.println(F("# inv x|y 0|1          invierte sentido de eje X o Y de la camara"));
  Serial.println(F("# servos 0|1           habilita/deshabilita senal PWM a los servos"));
  Serial.println(F("# pines_servo 41 42    cambia GPIO de servo X e Y | intercambiar"));
  Serial.println(F("# cx 80 | cy 78 | nivel | prueba x 6 | prueba y 6 | signo x 1|-1 | centrar"));
  Serial.println(F("# fz 5 20 90 | kg 10.7 4.75 | retardo 120 140 | smax 22 | obs 0|1 [wd wxv]"));
  Serial.println(F("# sp 2 -1 (cm) | circulo 3 5 (R cm, T s) | centro | foto | wifi"));
}

void telemetria(const float sp[2]) {
  if (!imprimir) return;
  if (estado == DETENIDO) {
    if (calibrando && nPaso % 10 == 0) {
      Serial.printf("# cal: detecta %d px=(%.1f,%.1f) | X %.1f..%.1f  Y %.1f..%.1f (%d)\n",
                    toqueAct, rawX * 0.1f, rawY * 0.1f, cMinX * 0.1f, cMaxX * 0.1f, cMinY * 0.1f, cMaxY * 0.1f, cN);
    }
    return;
  }
  if (imprimir == 1 && (nPaso & 1)) return;
  Serial.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%u\n",
                (unsigned long)millis(), validaAct, posX * 100.0f, posY * 100.0f,
                ej[0].ob.x * 100.0f, ej[1].ob.x * 100.0f, ej[0].ob.v * 100.0f, ej[1].ob.v * 100.0f,
                sp[0] * 100.0f, sp[1] * 100.0f, servoAct[0], servoAct[1],
                ej[0].ob.d * 100.0f, ej[1].ob.d * 100.0f, estado);
}

// ===================== INTERPRETE DE COMANDOS =====================
char linea[96];
uint8_t nL = 0;

void comando(char *s) {
  char *tok[6];
  int n = 0;
  for (char *t = strtok(s, " \t"); t && n < 6; t = strtok(NULL, " \t")) tok[n++] = t;
  if (!n) return;
  const char *c = tok[0];
  float v[5] = {0, 0, 0, 0, 0};
  for (int i = 1; i < n; i++) v[i - 1] = atof(tok[i]);
  bool esX = n > 1 && !strcmp(tok[1], "x"), esY = n > 1 && !strcmp(tok[1], "y");

  if (!strcmp(c, "stop") || !strcmp(c, "!")) { calibrando = false; detener("orden stop"); return; }
  if (!strcmp(c, "ver")) { ver(); return; }
  if (!strcmp(c, "?") || !strcmp(c, "ayuda")) { ayuda(); return; }
  if (!strcmp(c, "foto")) { fotoSerie(); return; }
  if (!strcmp(c, "wifi")) { iniciarWeb(); return; }
  if (!strcmp(c, "p") && n == 2) { imprimir = (uint8_t)constrain((int)v[0], 0, 2); return; }
  if (!strcmp(c, "sp") && n == 3) {
    spX = constrain(v[0], -6.5f, 6.5f) / 100.0f;
    spY = constrain(v[1], -5.0f, 5.0f) / 100.0f;
    circR = 0;
    return;
  }
  if (!strcmp(c, "centro")) { spX = spY = 0; circR = 0; return; }
  if (!strcmp(c, "circulo") && n == 3 && v[0] >= 0 && v[0] <= 4.5f && v[1] >= 2 && v[1] <= 30) {
    circR = v[0] / 100.0f; circT = v[1]; circT0 = millis(); spX = spY = 0; return;
  }
  if (!strcmp(c, "centrar")) {
    if (estado != ACTIVO) { Serial.println(F("# centrar: primero iniciar")); return; }
    centrando = true; tCentrar = 0; sumD[0] = sumD[1] = 0;
    tCentrarObj = circR > 0 ? 2 * circT : 4.0f;
    Serial.printf("# centrando durante %.0f s\n", tCentrarObj);
    return;
  }
  if (!strcmp(c, "iniciar")) {
    ejes = (esX ? 1 : (esY ? 2 : 3));
    for (int k = 0; k < 2; k++) {
      if ((ejes & (1 << k)) && !signoOk[k]) {
        Serial.printf("# falta probar el signo de %c: usa 'prueba %c 6' o 'signo %c 1'\n", k ? 'y' : 'x', k ? 'y' : 'x', k ? 'y' : 'x');
        return;
      }
    }
    if (!servosActivos) { servosActivos = true; conectarServos(); }
    calibrando = false;
    nivelar();
    estado = ESPERANDO;
    buenas = 0;
    motivo = "esperando bola";
    Serial.println(F("# ESPERANDO: pon la bola en el plato (arranca tras 5 frames validos)"));
    return;
  }
  if (estado != DETENIDO) { Serial.println(F("# primero escribe 'stop'")); return; }

  // ---- Comandos de vision y calibracion (solo detenido) ----
  if (!strcmp(c, "panel") || !strcmp(c, "camara")) { diagnosticoCamara(); return; }
  if (!strcmp(c, "fondo")) { pedirFondo = true; return; }
  if (!strcmp(c, "usar_fondo") && n == 2) { usarFondo = ((int)v[0] != 0); Serial.printf("# usar_fondo = %d\n", usarFondo); return; }
  if (!strcmp(c, "aprender")) { pedirAprender = true; return; }
  if (!strcmp(c, "modo") && n == 2 && v[0] >= 0 && v[0] <= 2) {
    modoVision = (uint8_t)v[0];
    Serial.printf("# modo vision = %u\n", modoVision);
    return;
  }
  if (!strcmp(c, "umbral") && n == 2 && v[0] >= 5 && v[0] <= 245) {
    umbralVision = (uint8_t)v[0];
    Serial.printf("# umbral = %u\n", umbralVision);
    return;
  }
  if (!strcmp(c, "area") && n == 3 && v[0] >= 3 && v[1] > v[0]) {
    areaMin = (uint16_t)v[0]; areaMax = (uint16_t)v[1];
    Serial.printf("# area valida = %u..%u px\n", areaMin, areaMax);
    return;
  }
  if (!strcmp(c, "plato") && n == 3 && v[0] >= 5 && v[0] <= 50 && v[1] >= 5 && v[1] <= 50) {
    LX = v[0] / 100.0f; LY = v[1] / 100.0f;
    Serial.printf("# plato = %.2f x %.2f cm\n", v[0], v[1]);
    return;
  }
  if (!strcmp(c, "roi") && n == 5) {
    int xi = (int)(v[0] * 10), xf = (int)(v[1] * 10), yi = (int)(v[2] * 10), yf = (int)(v[3] * 10);
    if (xi >= 0 && xf <= 3200 && yi >= 0 && yf <= 2400 && xf - xi >= 400 && yf - yi >= 400) {
      X_INI = xi; X_FIN = xf; Y_INI = yi; Y_FIN = yf; panelCal = true; guardarHW();
      Serial.printf("# ROI guardada: X %.1f..%.1f px, Y %.1f..%.1f px\n", X_INI * 0.1f, X_FIN * 0.1f, Y_INI * 0.1f, Y_FIN * 0.1f);
    }
    return;
  }
  if (!strcmp(c, "inv") && (esX || esY) && n == 3) {
    if (esX) invertirX = ((int)v[1] != 0);
    else invertirY = ((int)v[1] != 0);
    Serial.printf("# invertir X=%d Y=%d\n", invertirX, invertirY);
    return;
  }
  if (!strcmp(c, "cal")) {
    if (n > 1 && !strcmp(tok[1], "fin")) {
      calibrando = false;
      if (cN < 20 || cMaxX - cMinX < 400 || cMaxY - cMinY < 400) {
        Serial.printf("# cal incompleta (%d muestras): repite\n", cN);
        return;
      }
      X_INI = cMinX; X_FIN = cMaxX; Y_INI = cMinY; Y_FIN = cMaxY; panelCal = true; guardarHW();
      Serial.printf("# cal guardada: X %.1f..%.1f px  Y %.1f..%.1f px\n", X_INI * 0.1f, X_FIN * 0.1f, Y_INI * 0.1f, Y_FIN * 0.1f);
    } else {
      calibrando = true; cN = 0; cMinX = 3200; cMinY = 2400; cMaxX = 0; cMaxY = 0; nivelar();
      Serial.println(F("# CALIBRANDO ROI: pasea la bola por los 4 bordes del plato y escribe 'cal fin'"));
    }
    return;
  }
  if (!strcmp(c, "servos") && n == 2) {
    servosActivos = ((int)v[0] != 0);
    conectarServos();
    nivelar();
    Serial.printf("# servos %s\n", servosActivos ? "ACTIVOS" : "DESACTIVADOS");
    return;
  }
  if (!strcmp(c, "pines_servo") && n == 3) {
    uint8_t px = (uint8_t)v[0], py = (uint8_t)v[1];
    if (px <= 48 && py <= 48 && px != py) {
      PIN_SX = px; PIN_SY = py; conectarServos(); nivelar(); guardarHW();
      Serial.printf("# pines servo: X=GPIO%u Y=GPIO%u\n", PIN_SX, PIN_SY);
    }
    return;
  }
  if ((!strcmp(c, "cx") || !strcmp(c, "cy")) && n == 2 && v[0] >= 30 && v[0] <= 150) {
    int k = c[1] == 'y';
    ej[k].centro = v[0]; nivelar();
    Serial.printf("# %s = %.1f\n", c, v[0]);
    return;
  }
  if (!strcmp(c, "nivel")) { autoNivel(); return; }
  if (!strcmp(c, "prueba") && (esX || esY)) {
    float g = n > 2 ? v[1] : 6.0f;
    if (fabsf(g) < 2 || fabsf(g) > 15) { Serial.println(F("# prueba x|y 2..15")); return; }
    prueba(esY, g);
    return;
  }
  if (!strcmp(c, "signo") && (esX || esY) && n == 3 && (v[1] == 1 || v[1] == -1)) {
    ej[esY].signo = v[1]; signoOk[esY] = true; Serial.println(F("# ok")); return;
  }
  if (!strcmp(c, "intercambiar")) {
    uint8_t t = PIN_SX; PIN_SX = PIN_SY; PIN_SY = t;
    float c0 = ej[0].centro; ej[0].centro = ej[1].centro; ej[1].centro = c0;
    conectarServos(); nivelar(); signoOk[0] = signoOk[1] = false; guardarHW();
    Serial.printf("# servo X en GPIO%u (cx %.1f), Y en GPIO%u (cy %.1f)\n", PIN_SX, ej[0].centro, PIN_SY, ej[1].centro);
    return;
  }
  if (!strcmp(c, "fz") && n == 4 && v[0] >= 1 && v[0] <= 20 && v[1] >= 3 && v[1] <= 100 && v[2] >= 10 && v[2] <= 150) {
    esc.E = v[0] / 100.0f; esc.V = v[1] / 100.0f; esc.A = v[2] / 100.0f;
    Serial.printf("# fuzzy E %.1f cm, V %.1f cm/s, A %.0f cm/s2\n", v[0], v[1], v[2]);
    return;
  }
  if (!strcmp(c, "kg") && n >= 2 && v[0] >= 1 && v[0] <= 30) {
    ej[0].K = v[0] / 100.0f;
    ej[1].K = (n == 3 && v[1] >= 1 && v[1] <= 30 ? v[1] : v[0]) / 100.0f;
    Serial.printf("# K x %.2f y %.2f cm/s2 por grado\n", ej[0].K * 100.0f, ej[1].K * 100.0f);
    return;
  }
  if (!strcmp(c, "retardo") && n >= 2 && v[0] >= 0 && v[0] <= 500) {
    float ry = (n >= 3 && v[1] >= 0 && v[1] <= 500) ? v[1] : v[0];
    ej[0].retardo = v[0] / 1000.0f; ej[1].retardo = ry / 1000.0f;
    for (int k = 0; k < 2; k++) ej[k].tauServo = ej[k].retardo > 0 ? 0.03f : 0.06f;
    Serial.printf("# retardo x %.0f y %.0f ms\n", v[0], ry);
    return;
  }
  if (!strcmp(c, "smax") && n == 2 && v[0] >= 3 && v[0] <= SMAX_ABS) { ej[0].smax = ej[1].smax = v[0]; return; }
  if (!strcmp(c, "obs") && n >= 2) {
    ej[0].usarObs = ej[1].usarObs = v[0] != 0;
    if (n >= 3 && v[1] >= 0.3f && v[1] <= 15) ej[0].ob.wd = ej[1].ob.wd = v[1];
    if (n >= 4 && v[2] >= 4 && v[2] <= 30) ej[0].ob.wxv = ej[1].ob.wxv = v[2];
    Serial.printf("# observador %s (wd=%.1f, wxv=%.0f rad/s)\n", ej[0].usarObs ? "activo" : "solo estima", ej[0].ob.wd, ej[0].ob.wxv);
    return;
  }
  if (!strcmp(c, "perdida") && n == 2 && v[0] >= 100 && v[0] <= 800) { tPerdida = v[0] / 1000.0f; return; }
  if (!strcmp(c, "guardar")) { guardar(); return; }
  if (!strcmp(c, "fabrica")) {
    EEPROM.put(DIR_FZ, (uint16_t)0);
    EEPROM.put(DIR_HW, (uint16_t)0);
    EEPROM.commit();
    Serial.println(F("# configuracion restaurada a valores de fabrica"));
    return;
  }
  Serial.println(F("# comando desconocido ('?' para ayuda)"));
}

void leerSerie() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      linea[nL] = 0;
      if (nL) comando(linea);
      nL = 0;
    } else if (nL < sizeof(linea) - 1) {
      linea[nL++] = ch;
    }
  }
}

// ===================== SETUP Y LOOP PRINCIPAL =====================
void setup() {
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  delay(400);
  Serial.println(F("\n# DSM BALL & PLATE VISION + FUZZY (ESP32-S3-CAM N16R8 + OV2640 + 2 SERVOS)"));

  if (!psramFound()) {
    Serial.println(F("# ERROR: PSRAM no detectada. Compila con PSRAM=opi."));
    return;
  }
  ultimoFrameRgb = static_cast<uint16_t *>(heap_caps_malloc(camhw::ancho * camhw::alto * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  fondoLuma = static_cast<uint8_t *>(heap_caps_malloc(camhw::ancho * camhw::alto, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  mutexCam = xSemaphoreCreateMutex();
  if (!ultimoFrameRgb || !fondoLuma || !mutexCam) {
    Serial.println(F("# ERROR reservando buffers en PSRAM."));
    return;
  }

  EEPROM.begin(TAM_EEPROM);
  ej[0].centro = 80.0f;
  ej[1].centro = 78.0f;
  for (int k = 0; k < 2; k++) ej[k].K = K_TEORICO;
  cargarHW();
  cargar();

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = camhw::d0; c.pin_d1 = camhw::d1; c.pin_d2 = camhw::d2; c.pin_d3 = camhw::d3;
  c.pin_d4 = camhw::d4; c.pin_d5 = camhw::d5; c.pin_d6 = camhw::d6; c.pin_d7 = camhw::d7;
  c.pin_xclk = camhw::xclk; c.pin_pclk = camhw::pclk;
  c.pin_vsync = camhw::vsync; c.pin_href = camhw::href;
  c.pin_sccb_sda = camhw::sda; c.pin_sccb_scl = camhw::scl;
  c.pin_pwdn = camhw::pwdn; c.pin_reset = camhw::reset;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_QVGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("# ERROR iniciando camara OV2640: 0x%x (%s)\n", err, esp_err_to_name(err));
    return;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    Serial.printf("# Sensor OV2640 listo (PID 0x%04x) en QVGA 320x240 RGB565\n", s->id.PID);
  }

  conectarServos();
  nivelar();
  iniciarTareasServo();
  xTaskCreatePinnedToCore(tareaCaptura, "captura_cv", 6144, nullptr, 2, nullptr, 0);

  camaraLista = true;
  Serial.println(F("# DETENIDO. Escribe '?' para ayuda, 'ver' para estado, 'aprender' con la bola en el centro."));
  proximo = ultimo = micros();
}

void loop() {
  if (!camaraLista) { delay(200); return; }
  if (webActiva) web.handleClient();
  leerSerie();

  uint32_t ahora = micros();
  if ((int32_t)(ahora - proximo) < 0) return;
  proximo += periodoUs;
  if ((int32_t)(ahora - proximo) > 0) proximo = ahora + periodoUs;
  uint32_t dtUs = ahora - ultimo;
  ultimo = ahora;
  float dt = constrain(dtUs * 1e-6f, 0.002f, 0.2f);
  usPeriodo = dtUs;
  nPaso++;

  uint32_t t0 = micros();
  bool esFrameNuevo = false;
  bool valida = medir(esFrameNuevo);
  // Cada foto nueva actualiza la correccion del observador una sola vez
  bool hayMedidaNueva = esFrameNuevo && valida;
  usLectura = micros() - t0;
  if (estado == ACTIVO && usLectura > usLecturaMax) usLecturaMax = usLectura;

  // Calibracion de bordes del plato en decimas de pixel
  static int prevX = -100, prevY = -100;
  if (calibrando && toqueAct && esFrameNuevo) {
    if (abs(rawX - prevX) <= 60 && abs(rawY - prevY) <= 60) {
      cMinX = min(cMinX, rawX); cMaxX = max(cMaxX, rawX);
      cMinY = min(cMinY, rawY); cMaxY = max(cMaxY, rawY);
      cN++;
    }
    prevX = rawX; prevY = rawY;
  } else if (!calibrando) {
    prevX = prevY = -100;
  }

  float sp[2], vsp[2], asp[2];
  referencia(sp, vsp, asp);

  if (estado == ESPERANDO) {
    if (hayMedidaNueva) {
      if (++buenas >= 5) arrancar(posX, posY, dt);
    } else if (millis() - tUltValida > 350) {
      buenas = 0;
    }
  } else if (estado == ACTIVO) {
    if (dtUs > usPeriodoMax) usPeriodoMax = dtUs;
    if (dt > 0.1f) { detener("periodo > 100 ms"); telemetria(sp); return; }
    if (valida) {
      if (esFrameNuevo) tSinBola = 0;
      else tSinBola += dt;
    } else {
      cortes++;
      tSinBola += dt;
    }
    if (tSinBola >= tPerdida) {
      detener("bola perdida (sin deteccion valida)");
      telemetria(sp);
      return;
    }
    if (centrando) {
      for (int k = 0; k < 2; k++) sumD[k] += ej[k].ob.d * dt;
      tCentrar += dt;
      if (tCentrar >= tCentrarObj) {
        centrando = false;
        for (int k = 0; k < 2; k++) {
          if (!(ejes & (1 << k))) continue;
          float dm = sumD[k] / tCentrar, du = -dm / (ej[k].signo * ej[k].K);
          if (fabsf(du) > 8.0f) {
            Serial.printf("# centrar %c: pide %.1f grados (excesivo)\n", k ? 'y' : 'x', du);
            continue;
          }
          ej[k].centro += du;
          ej[k].ob.d -= dm;
          Serial.printf("# c%c = %.1f (%+.1f)\n", k ? 'y' : 'x', ej[k].centro, du);
        }
        Serial.println(F("# escribe 'guardar' (despues de stop) para conservar los centros"));
      }
    }
    float z[2] = {posX, posY}, u[2];
    uint32_t t1 = micros();
    for (uint8_t k = 0; k < 2; k++) {
      u[k] = (ejes & (1 << k)) ? ej[k].paso(esc, hayMedidaNueva, z[k], dt, sp[k], vsp[k], asp[k]) : ej[k].centro;
    }
    uint32_t t2 = micros();
    escribir(0, u[0]);
    escribir(1, u[1]);
    uint32_t t3 = micros();
    usCtrlMax = max(usCtrlMax, t2 - t1);
    usServoMax = max(usServoMax, t3 - t2);
  }
  uint32_t t4 = micros();
  telemetria(sp);
  if (estado == ACTIVO) usSerieMax = max(usSerieMax, (uint32_t)(micros() - t4));
}
