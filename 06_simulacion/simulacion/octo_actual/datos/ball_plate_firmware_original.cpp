// Camara OV2640 del ESP32-S3 para ball and plate: captura, deteccion de la bola y visor por serie
//
// Nucleo 0: captura JPEG + descompresion (esp_new_jpeg, SIMD) a imagen de brillo
// Nucleo 1: deteccion de la bola (umbral automatico + manchas + prediccion) o posicion
//          mandada por la Raspberry Pi ("p<x>,<y>"), + PID + servos (Futaba S3003, 50 Hz)
// Envio a la laptop en una tarea de baja prioridad, para no frenar la captura ni la deteccion
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_jpeg_dec.h"
#include "driver/ledc.h"

// Pinout ESP32-S3-CAM (igual que ESP32-S3-EYE / Freenove)
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

static const uint32_t BAUDIOS = 2000000;
static const int ANCHO = 160, ALTO = 120, PIXELES = ANCHO * ALTO;
static const uint32_t ENVIO_CADA_MS = 80;

// "u<valor>\n" desde la laptop: 0 = automatico (se calcula en cada imagen), otro valor = fijo
static volatile uint8_t umbral = 0;
// zona de interes (la placa): solo se busca la bola aqui. "r<x0>,<y0>,<x1>,<y1>\n" desde la laptop
// por defecto: la parte visible de la placa con margen; al inclinarse la placa, el fondo claro
// de detras de sus bordes entra en la imagen y se confundia con la bola
static volatile int16_t roi_x0 = 53, roi_y0 = 10, roi_x1 = 119, roi_y1 = 104;

// Centro de la placa en la imagen (pixeles) y ejes de control:
//   X = derecha en la imagen (+), Y = arriba en la imagen (+), origen en el centro de la placa
//   x_placa = cx - CENTRO_X     y_placa = CENTRO_Y - cy
// Zona de control simetrica: la bola se ve entera con |x| <= ALCANCE_X y |y| <= ALCANCE_Y
static const int16_t CENTRO_X = 89, CENTRO_Y = 57;
static const int16_t ALCANCE_X = 24, ALCANCE_Y = 39;
// diferencia media con la imagen anterior por encima de la cual se considera corrupta
static const uint8_t SALTO_IMAGEN = 12;
// la bola no puede moverse mas que esto entre dos imagenes (20 ms); un salto mayor es un error
static const int16_t SALTO_BOLA = 8;
// tras tantas imagenes seguidas sin posicion valida, se acepta la bola donde aparezca
static const uint8_t PERDIDA_MAX = 5;
// tamano aceptado de la bola en pixeles (medida ~80). Fuera de esto es un reflejo de la placa
// inclinada (se midieron manchas de >1000 px) o ruido, y no se usa para el control
static const uint32_t BOLA_PX_MIN = 15, BOLA_PX_MAX = 200;
// medio lado de la ventana de seguimiento alrededor de la posicion predicha de la bola (pixeles)
static const int16_t VENTANA = 16;
// umbral automatico = fondo + UMBRAL_FRACCION * (pico - fondo): fondo = brillo mediano de la placa,
// pico = pixel mas brillante de la zona de busqueda. Asi se adapta solo si cambia la luz
// 0.3: la bola gris solo tiene un brillo pequeno; con 0.5 quedaba solo ese brillo (<15 px)
static const float UMBRAL_FRACCION = 0.3f;
// si el pico no supera al fondo en esto, no hay bola (solo ruido o brillos suaves de la placa)
static const int CONTRASTE_MIN = 40;
// forma de la mancha: la bola es compacta. Bordes de la placa, dedos y reflejos son alargados
// o huecos: lado largo <= 2 * lado corto + 1 y ocupa >= 40 % de su rectangulo (un circulo, 78 %)
static const int MAX_MANCHAS = 32;
// trozos de la misma bola: se juntan las manchas con centros a menos de RADIO_UNION pixeles
// (la bola mide ~12 px de diametro); trozos de menos de TROZO_PX_MIN pixeles son ruido
static const float RADIO_UNION = 8.0f;
static const uint32_t TROZO_PX_MIN = 3;
static volatile int exposicion = -1;   // -1 automatica; "e<valor>\n" la fija
static volatile bool cambiar_exposicion = false;
// calidad 30: medido 0 imagenes corruptas en 60 s a 50 FPS (con 15 y 25 salen cientos)
static volatile int xclk_mhz = 20, calidad_jpeg = 30;  // "c<xclk_mhz>,<calidad>\n" reinicia la camara
static volatile bool reiniciar_camara = false;

// ---- Servos (Futaba S3003: analogicos, 50 Hz, ~1000-2000 us) ----
#define SERVO_X_PIN 47
#define SERVO_Y_PIN 21
// la camara usa LEDC_TIMER_0 / CANAL_0 para su reloj; los servos van en otro timer
static const ledc_timer_t SERVO_TIMER = LEDC_TIMER_1;
static const ledc_channel_t SERVO_X_CANAL = LEDC_CHANNEL_2, SERVO_Y_CANAL = LEDC_CHANNEL_3;
static const uint32_t SERVO_HZ = 50, SERVO_BITS = 14;

// Ajustes del control (todos se cambian desde la laptop):
//   "o<0|1|2>"          0 = apagado (servos sin senal), 1 = PID, 2 = servos fijos en el neutro (placa plana)
//   "k<kp>,<ki>,<kd>"   ganancias en us de servo por pixel (kp), por pixel*s (ki), por pixel/s (kd)
//   "n<us_x>,<us_y>"    pulso con el que la placa queda horizontal
//   "l<us>"             maxima desviacion del pulso respecto al neutro
//   "d<+-1>,<+-1>"      sentido de cada servo (invertir si empuja la bola hacia afuera)
//   "f<fx>,<fy>"        factor de ganancia por eje (compensa que un lado de la placa se incline menos)
//   "v<a>"              filtro de la velocidad (0..0.95): mas alto = mas suave pero con mas retraso
//   "s<x>,<y>"          consigna: donde mantener la bola, en pixeles desde el centro de la placa
static volatile uint8_t modo_control = 0;  // arranca apagado por seguridad
static volatile float kp = 1.5f, ki = 0.2f, kd = 0.8f;
static volatile int16_t neutro_x_us = 1500, neutro_y_us = 1500, limite_us = 250;
static volatile int8_t signo_x = 1, signo_y = 1;
static volatile float factor_x = 1.0f, factor_y = 1.0f;
static volatile float filtro_v = 0.6f;
// "w<0|1>" borrar la integral al cruzar la consigna; "m<us>" empujon de compensacion de rozamiento
static volatile bool reinicio_integral = false;
static volatile float friccion_us = 0;
static const float ZONA_MUERTA_PX = 1.5f;     // mas cerca del centro que esto se considera centrada
static const float VELOCIDAD_QUIETA = 8.0f;   // px/s: por debajo, la bola se considera parada
// "i<vel>,<autoridad>": la integral solo acumula con la bola por debajo de vel px/s (0 = siempre)
// y puede aportar hasta autoridad (0..1) del recorrido
static volatile float vel_integral = 0, autoridad_i = 0.5f;
// "b<alfa>,<beta>": filtro alfa-beta de posicion/velocidad (alfa = 0 lo desactiva y usa filtro_v)
static volatile float ab_alfa = 0, ab_beta = 0;
// "z<xp>,<xn>,<yp>,<yn>": holgura del mecanismo en us, por eje y sentido (medida con pasos de servo)
static volatile float holgura_xp = 0, holgura_xn = 0, holgura_yp = 0, holgura_yn = 0;
static volatile float consigna_x = 0, consigna_y = 0;
// si se pierde la bola, se mantiene la ultima inclinacion este tiempo para traerla de vuelta
static const uint32_t PERDIDA_MANTENER_MS = 500;

// Posicion de la bola medida fuera (Raspberry Pi con la PS3 Eye): "p<x>,<y>\n" en pixeles de esta
// camara desde el centro de la placa (los mismos ejes y unidades que x_placa / y_placa), "p\n" = sin
// bola. Mientras lleguen, el PID usa estas medidas y no la camara del ESP32.
// "p<x>,<y>,<vx>,<vy>\n" ademas trae la velocidad (unidades por segundo) calculada en la Pi con todas
// sus imagenes: es mas limpia que la que saca el PID restando posiciones, y la usa en su lugar
struct MedidaExterna { float x, y, vx, vy; bool valida, con_velocidad; };
static QueueHandle_t cola_externa;
static volatile uint32_t t_ultima_externa = 0;
static volatile uint32_t n_externas = 0;
// sin medidas de fuera durante este tiempo se vuelve a la camara propia
static const uint32_t EXTERNA_VIGENTE_MS = 300;

// dos buffers de brillo: la camara escribe en uno mientras la deteccion lee el otro
static uint8_t *gris[2];
static QueueHandle_t cola_listos;   // indice del buffer recien descomprimido
static QueueHandle_t cola_libres;   // indice de buffer disponible para la camara

static uint8_t *envio;              // copia del cuadro que se manda a la laptop
static volatile bool envio_ocupado = false;
static const int CAMPOS_CABECERA = 20;
// diagnostico: ancho del pulso medido en el propio pin de cada servo ("q\n" lo mide), 0 = sin pulsos
static volatile uint16_t pulso_medido_x = 0, pulso_medido_y = 0;
static uint16_t cabecera_envio[CAMPOS_CABECERA];

// ---- Deteccion por manchas ----
// Se marcan los pixeles mas brillantes que el umbral, se agrupan en manchas conectadas y se elige
// la que parece una bola (tamano y forma) mas cercana a donde se predice que esta
struct Mancha {
  uint32_t n, swx, swy, sw;          // pixeles y sumas ponderadas por brillo
  int16_t xmin, xmax, ymin, ymax;
};
static uint8_t marcado[PIXELES];     // 1 = ya pertenece a una mancha (solo se usa dentro de la zona)
static uint16_t pila[PIXELES];

// Rellena desde (x, y) todos los pixeles conectados (4 vecinos) por encima del umbral
static void crecerMancha(const uint8_t *p, int x, int y, int x0, int y0, int x1, int y1, uint8_t u, Mancha &m) {
  m = {0, 0, 0, 0, (int16_t)x, (int16_t)x, (int16_t)y, (int16_t)y};
  int tope = 0;
  pila[tope++] = y * ANCHO + x;
  marcado[y * ANCHO + x] = 1;
  while (tope) {
    int i = pila[--tope], px = i % ANCHO, py = i / ANCHO;
    uint32_t peso = p[i] - u;
    m.n++; m.swx += peso * px; m.swy += peso * py; m.sw += peso;
    if (px < m.xmin) m.xmin = px;
    if (px > m.xmax) m.xmax = px;
    if (py < m.ymin) m.ymin = py;
    if (py > m.ymax) m.ymax = py;
    const int vecinos[4] = {i - 1, i + 1, i - ANCHO, i + ANCHO};
    const bool dentro[4] = {px > x0, px < x1 - 1, py > y0, py < y1 - 1};
    for (int k = 0; k < 4; k++) {
      int v = vecinos[k];
      if (dentro[k] && !marcado[v] && p[v] > u) { marcado[v] = 1; pila[tope++] = v; }
    }
  }
}

static bool pareceBola(const Mancha &m) {
  if (m.n < BOLA_PX_MIN || m.n > BOLA_PX_MAX) return false;
  int w = m.xmax - m.xmin + 1, h = m.ymax - m.ymin + 1;
  int largo = max(w, h), corto = min(w, h);
  return largo <= 2 * corto + 1 && m.n * 10 >= (uint32_t)(4 * w * h);
}

static volatile uint32_t t_desc_us = 0;  // tiempo de descompresion del ultimo cuadro
// contadores de diagnostico de la tarea de camara (por segundo)
static volatile uint32_t n_fb = 0, n_sin_fb = 0, n_sin_buffer = 0, n_fallo_desc = 0, max_len = 0;

static bool iniciarCamara() {
  camera_config_t c = {};
  c.pin_pwdn = -1; c.pin_reset = -1;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d0 = Y2_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.xclk_freq_hz = xclk_mhz * 1000000;
  c.ledc_timer = LEDC_TIMER_0; c.ledc_channel = LEDC_CHANNEL_0;
  // los formatos sin comprimir se cuelgan con este driver; JPEG y se descomprime aqui
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_QQVGA;  // 160x120
  c.jpeg_quality = calidad_jpeg;  // mas alto = mas comprimido
  c.fb_count = 2;
  // en RAM interna: con PSRAM la DMA de la camara compite con la descompresion y se pierden bytes
  c.fb_location = CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&c) != ESP_OK) return false;
  sensor_t *s = esp_camera_sensor_get();
  s->set_special_effect(s, 2);  // escala de grises en el sensor
  return true;
}

static void aplicarExposicion() {
  sensor_t *s = esp_camera_sensor_get();
  if (exposicion < 0) { s->set_exposure_ctrl(s, 1); s->set_gain_ctrl(s, 1); }
  else { s->set_exposure_ctrl(s, 0); s->set_aec_value(s, exposicion); }
}

static void iniciarServos() {
  ledc_timer_config_t t = {};
  t.speed_mode = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = (ledc_timer_bit_t)SERVO_BITS;
  t.timer_num = SERVO_TIMER;
  t.freq_hz = SERVO_HZ;
  t.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&t);
  const int pines[2] = {SERVO_X_PIN, SERVO_Y_PIN};
  const ledc_channel_t canales[2] = {SERVO_X_CANAL, SERVO_Y_CANAL};
  for (int i = 0; i < 2; i++) {
    ledc_channel_config_t c = {};
    c.gpio_num = pines[i];
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = canales[i];
    c.timer_sel = SERVO_TIMER;
    c.duty = 0;
    ledc_channel_config(&c);
    gpio_input_enable((gpio_num_t)pines[i]);  // para poder leer el pin y medir el pulso que sale
  }
}

// us = 0 apaga la senal: el servo analogico queda libre y casi no consume
// mide el ancho del pulso que realmente sale por el pin (espera como mucho dos periodos)
static uint16_t medirPulso(int pin) {
  uint32_t t0 = micros();
  while (gpio_get_level((gpio_num_t)pin) == 1) if (micros() - t0 > 45000) return 9999;  // siempre alto
  while (gpio_get_level((gpio_num_t)pin) == 0) if (micros() - t0 > 45000) return 0;     // sin pulsos
  uint32_t subida = micros();
  while (gpio_get_level((gpio_num_t)pin) == 1) if (micros() - subida > 5000) return 9999;
  return micros() - subida;
}

static void escribirServo(ledc_channel_t canal, int us) {
  uint32_t duty = (uint32_t)us * ((1u << SERVO_BITS) - 1) / (1000000 / SERVO_HZ);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, canal, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, canal);
}

// PID de un eje. Derivada sobre la medicion (velocidad filtrada) para no dar golpes al cambiar la consigna
struct EjePID {
  float integral = 0, velocidad = 0, pos_anterior = 0, error_anterior = 0, pos_est = 0;
  bool inicializado = false;

  void reiniciar() { integral = 0; velocidad = 0; error_anterior = 0; inicializado = false; }

  // devuelve la correccion en us de servo. Con usar_vel_ext la velocidad viene de fuera (la Pi)
  float paso(float pos, float consigna, float dt, bool usar_vel_ext = false, float vel_ext = 0) {
    if (!inicializado) { pos_anterior = pos; pos_est = pos; velocidad = 0; inicializado = true; }
    if (usar_vel_ext) {
      velocidad = vel_ext;
    } else if (ab_alfa > 0) {
      // filtro alfa-beta: predice con la velocidad y corrige con la medida; da posicion y
      // velocidad suaves con mucho menos retraso que un filtro paso bajo
      float pred = pos_est + velocidad * dt;
      float r = pos - pred;
      pos_est = pred + ab_alfa * r;
      velocidad += ab_beta / dt * r;
      pos = pos_est;
    } else {
      // velocidad con filtro paso bajo: el centroide tiene ruido de ~1 pixel
      float v = (pos - pos_anterior) / dt;
      velocidad = filtro_v * velocidad + (1 - filtro_v) * v;
    }
    pos_anterior = pos;
    float error = consigna - pos;
    // opcional: al cruzar la consigna se borra la integral para que no la pase de largo
    if (reinicio_integral && error * error_anterior < 0) integral = 0;
    error_anterior = error;
    // integral condicional: solo acumula con la bola lenta. Una base inclinada deja la bola apartada
    // y casi quieta (ahi hay que acumular); el vaiven normal la mueve rapido (ahi acumular la pasa de largo)
    if (vel_integral <= 0 || fabsf(velocidad) < vel_integral) integral += error * dt;
    // anti-windup: el termino integral puede aportar hasta esta fraccion del recorrido
    // (con la base inclinada es lo que mantiene la placa compensada)
    float max_i = ki > 0 ? autoridad_i * limite_us / ki : 0;
    integral = constrain(integral, -max_i, max_i);
    float u = kp * error + ki * integral - kd * velocidad;
    // compensacion de rozamiento: la bola quieta fuera del centro no se despega con correcciones
    // pequenas; se suma un empujon fijo hacia la consigna mientras este quieta y lejos del centro
    if (friccion_us > 0 && fabsf(error) > ZONA_MUERTA_PX && fabsf(velocidad) < VELOCIDAD_QUIETA)
      u += error > 0 ? friccion_us : -friccion_us;
    return u;
  }
};
static EjePID pid_x, pid_y;
static volatile int16_t pulso_x_us = 1500, pulso_y_us = 1500;

// Compensacion de holgura: el mecanismo tiene un juego (distinto en cada sentido) en el que el servo
// gira pero la placa no se inclina; se suma ese hueco al salir del neutro, con una rampa suave de
// holgura_rampa_us para que el servo no salte de un lado al otro alrededor del neutro
// ("z<xp>,<xn>,<yp>,<yn>[,<rampa>]"; con 10 us el paso por el centro daba golpes y la bola oscilaba)
static volatile float holgura_rampa_us = 10.0f;
static float compensarHolgura(float d, float hueco_pos, float hueco_neg) {
  if (d > 0) return d + hueco_pos * fminf(1.0f, d / holgura_rampa_us);
  if (d < 0) return d - hueco_neg * fminf(1.0f, -d / holgura_rampa_us);
  return 0;
}

// Nucleo 0: captura y descompresion
static void tareaCamara(void *) {
  // el decodificador no da GRAY para JPEG en color; CbYCrY y se copia solo Y (bytes impares)
  jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
  cfg.output_type = JPEG_PIXEL_FORMAT_CbYCrY;
  jpeg_dec_handle_t dec = nullptr;
  if (jpeg_dec_open(&cfg, &dec) != JPEG_ERR_OK) { Serial.println("ERROR decodificador"); vTaskDelete(nullptr); }
  uint8_t *yuv = (uint8_t *)jpeg_calloc_align(PIXELES * 2, 16);

  for (;;) {
    if (reiniciar_camara) {
      esp_camera_deinit();
      if (iniciarCamara()) aplicarExposicion();
      reiniciar_camara = false;
    }
    if (cambiar_exposicion) { aplicarExposicion(); cambiar_exposicion = false; }
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { n_sin_fb++; continue; }
    n_fb++;
    if (fb->len > max_len) max_len = fb->len;
    uint8_t idx;
    if (xQueueReceive(cola_libres, &idx, 0) != pdTRUE) { n_sin_buffer++; esp_camera_fb_return(fb); continue; }

    uint32_t t0 = micros();
    jpeg_dec_io_t io = {};
    jpeg_dec_header_info_t info;
    io.inbuf = fb->buf;
    io.inbuf_len = fb->len;
    // descartar JPEG incompletos: deben empezar con FFD8 y tener FFD9 al final
    // (el driver puede dejar bytes de relleno despues de FFD9, se busca en el ultimo tramo)
    bool completo = false;
    if (fb->len > 4 && fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) {
      size_t limite = fb->len > 1024 ? fb->len - 1024 : 2;
      for (size_t k = fb->len - 1; k >= limite; k--) {
        if (fb->buf[k - 1] == 0xFF && fb->buf[k] == 0xD9) { completo = true; io.inbuf_len = k + 1; break; }
      }
    }
    bool ok = completo && jpeg_dec_parse_header(dec, &io, &info) == JPEG_ERR_OK &&
              info.width == ANCHO && info.height == ALTO;
    if (ok) {
      io.outbuf = yuv;
      ok = jpeg_dec_process(dec, &io) == JPEG_ERR_OK;
    }
    if (ok) {
      // imagen completa (se manda asi a la laptop); el analisis solo usa el tablero
      uint8_t *g = gris[idx];
      for (int i = 0; i < PIXELES; i++) g[i] = yuv[2 * i + 1];
    }
    t_desc_us = micros() - t0;
    esp_camera_fb_return(fb);

    if (ok) xQueueSend(cola_listos, &idx, portMAX_DELAY);
    else { n_fallo_desc++; xQueueSend(cola_libres, &idx, portMAX_DELAY); }
  }
}

// Tarea de baja prioridad: manda la imagen y los datos a la laptop
static void tareaEnvio(void *) {
  for (;;) {
    if (envio_ocupado) {
      Serial.write((const uint8_t *)"FRM!", 4);
      Serial.write((const uint8_t *)cabecera_envio, sizeof(cabecera_envio));
      Serial.write(envio, PIXELES);
      // suma de comprobacion: el adaptador USB a 2 Mbaud a veces pierde bytes y la laptop
      // mostraria la imagen corrida; con esto descarta las imagenes que no llegaron completas
      uint16_t suma = 0;
      for (int i = 0; i < PIXELES; i++) suma += envio[i];
      Serial.write((const uint8_t *)&suma, 2);
      envio_ocupado = false;
    }
    while (Serial.available()) {
      static char buf[48]; static int n = 0;
      char ch = Serial.read();
      if (ch == '\n') {
        buf[n] = 0;
        if (buf[0] == 'p') {
          MedidaExterna m = {0, 0, 0, 0, false, false};
          int leidos = sscanf(buf + 1, "%f,%f,%f,%f", &m.x, &m.y, &m.vx, &m.vy);
          m.valida = leidos >= 2;
          m.con_velocidad = leidos == 4;
          xQueueOverwrite(cola_externa, &m);
          t_ultima_externa = millis();
          n_externas++;
        }
        if (buf[0] == 'u') umbral = atoi(buf + 1);
        if (buf[0] == 'e') { exposicion = atoi(buf + 1); cambiar_exposicion = true; }
        if (buf[0] == 'c') {
          int mhz, q;
          if (sscanf(buf + 1, "%d,%d", &mhz, &q) == 2 && mhz >= 6 && mhz <= 24 && q >= 4 && q <= 63) {
            xclk_mhz = mhz; calidad_jpeg = q; reiniciar_camara = true;
          }
        }
        if (buf[0] == 'o') { int m = atoi(buf + 1); if (m >= 0 && m <= 2) modo_control = m; }
        if (buf[0] == 'k') {
          float a, b, c;
          if (sscanf(buf + 1, "%f,%f,%f", &a, &b, &c) == 3 && a >= 0 && b >= 0 && c >= 0) { kp = a; ki = b; kd = c; }
        }
        if (buf[0] == 'n') {
          int a, b;
          if (sscanf(buf + 1, "%d,%d", &a, &b) == 2 && a >= 800 && a <= 2200 && b >= 800 && b <= 2200) {
            neutro_x_us = a; neutro_y_us = b;
          }
        }
        if (buf[0] == 'l') { int a = atoi(buf + 1); if (a >= 20 && a <= 600) limite_us = a; }
        if (buf[0] == 'd') {
          int a, b;
          if (sscanf(buf + 1, "%d,%d", &a, &b) == 2) { signo_x = a < 0 ? -1 : 1; signo_y = b < 0 ? -1 : 1; }
        }
        if (buf[0] == 'f') {
          float a, b;
          if (sscanf(buf + 1, "%f,%f", &a, &b) == 2 && a > 0 && a <= 10 && b > 0 && b <= 10) { factor_x = a; factor_y = b; }
        }
        if (buf[0] == 'q') { pulso_medido_x = medirPulso(SERVO_X_PIN); pulso_medido_y = medirPulso(SERVO_Y_PIN); }
        if (buf[0] == 'i') {
          float a, b;
          if (sscanf(buf + 1, "%f,%f", &a, &b) == 2 && a >= 0 && b > 0 && b <= 1) { vel_integral = a; autoridad_i = b; }
        }
        if (buf[0] == 'b') {
          float a, b;
          if (sscanf(buf + 1, "%f,%f", &a, &b) == 2 && a >= 0 && a <= 1 && b >= 0 && b <= 1) { ab_alfa = a; ab_beta = b; }
        }
        if (buf[0] == 'z') {
          float a, b, c, d, r;
          int n = sscanf(buf + 1, "%f,%f,%f,%f,%f", &a, &b, &c, &d, &r);
          if (n >= 4 && a >= 0 && b >= 0 && c >= 0 && d >= 0 && a <= 150 && b <= 150 && c <= 150 && d <= 150) {
            holgura_xp = a; holgura_xn = b; holgura_yp = c; holgura_yn = d;
            if (n == 5 && r >= 1 && r <= 200) holgura_rampa_us = r;
          }
        }
        if (buf[0] == 'w') reinicio_integral = atoi(buf + 1) != 0;
        if (buf[0] == 'm') { float a = atof(buf + 1); if (a >= 0 && a <= 150) friccion_us = a; }
        if (buf[0] == 'v') { float a = atof(buf + 1); if (a >= 0 && a <= 0.95f) filtro_v = a; }
        if (buf[0] == 's') {
          float a, b;
          if (sscanf(buf + 1, "%f,%f", &a, &b) == 2 && fabsf(a) <= ALCANCE_X && fabsf(b) <= ALCANCE_Y) {
            consigna_x = a; consigna_y = b;
          }
        }
        if (buf[0] == 'r') {
          int x0, y0, x1, y1;
          if (sscanf(buf + 1, "%d,%d,%d,%d", &x0, &y0, &x1, &y1) == 4 &&
              x0 >= 0 && y0 >= 0 && x1 <= ANCHO && y1 <= ALTO && x0 < x1 && y0 < y1) {
            roi_x0 = x0; roi_y0 = y0; roi_x1 = x1; roi_y1 = y1;
          }
        }
        n = 0;
      } else if (n < 47) buf[n++] = ch;
    }
    vTaskDelay(1);
  }
}

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.begin(BAUDIOS);
  if (!iniciarCamara()) { for (;;) { Serial.println("ERROR camara"); delay(1000); } }
  aplicarExposicion();
  iniciarServos();  // sin pulsos hasta encender el control: si los servos comparten la fuente, no la tumban al arrancar

  for (int i = 0; i < 2; i++) gris[i] = (uint8_t *)jpeg_calloc_align(PIXELES, 16);
  envio = (uint8_t *)heap_caps_malloc(PIXELES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  cola_listos = xQueueCreate(2, sizeof(uint8_t));
  cola_libres = xQueueCreate(2, sizeof(uint8_t));
  for (uint8_t i = 0; i < 2; i++) xQueueSend(cola_libres, &i, 0);
  cola_externa = xQueueCreate(1, sizeof(MedidaExterna));

  xTaskCreatePinnedToCore(tareaCamara, "camara", 8192, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(tareaEnvio, "envio", 4096, nullptr, 1, nullptr, 1);
}

// ---- PID y servos ---- (con la posicion de la camara propia o la que manda la Raspberry Pi)
static void controlar(bool hay_bola, float x_placa, float y_placa,
                      bool con_velocidad = false, float vx = 0, float vy = 0) {
  static uint32_t t_ultimo_pid = 0, t_ultima_bola = 0;
  uint32_t t_ahora_us = micros();
  if (modo_control != 1) {
    pid_x.reiniciar(); pid_y.reiniciar();
    pulso_x_us = neutro_x_us; pulso_y_us = neutro_y_us;
  } else if (hay_bola) {
    float dt = t_ultimo_pid ? (t_ahora_us - t_ultimo_pid) / 1e6f : 0.02f;
    dt = constrain(dt, 0.002f, 0.1f);
    float ux = factor_x * pid_x.paso(x_placa, consigna_x, dt, con_velocidad, vx);
    float uy = factor_y * pid_y.paso(y_placa, consigna_y, dt, con_velocidad, vy);
    ux = constrain(ux, (float)-limite_us, (float)limite_us);
    uy = constrain(uy, (float)-limite_us, (float)limite_us);
    pulso_x_us = neutro_x_us + (int)compensarHolgura(signo_x * ux, holgura_xp, holgura_xn);
    pulso_y_us = neutro_y_us + (int)compensarHolgura(signo_y * uy, holgura_yp, holgura_yn);
    t_ultima_bola = millis();
  } else if (millis() - t_ultima_bola > PERDIDA_MANTENER_MS) {
    // bola perdida mucho tiempo: placa horizontal y PID desde cero
    pid_x.reiniciar(); pid_y.reiniciar();
    pulso_x_us = neutro_x_us; pulso_y_us = neutro_y_us;
  }  // bola perdida poco tiempo: se mantiene la ultima inclinacion para traerla de vuelta
  t_ultimo_pid = hay_bola ? t_ahora_us : 0;
  escribirServo(SERVO_X_CANAL, modo_control ? pulso_x_us : 0);
  escribirServo(SERVO_Y_CANAL, modo_control ? pulso_y_us : 0);
}

// Con la Raspberry Pi mandando la posicion: PID en cuanto llega cada medida (100+ por segundo).
// La camara propia sigue capturando pero sus imagenes se devuelven sin analizar ni mandar.
// Cada ESTADO_EXTERNO_MS se responde una linea de texto para la Pi:
//   "S<pulso_x>,<pulso_y>,<estado>,<medidas por segundo>,<integral_x us>,<integral_y us>"
static const uint32_t ESTADO_EXTERNO_MS = 50;
static void bucleExterno() {
  uint8_t idx;
  while (xQueueReceive(cola_listos, &idx, 0) == pdTRUE) xQueueSend(cola_libres, &idx, portMAX_DELAY);
  static uint32_t t_estado = 0, t_tasa = 0, n_tasa = 0, tasa = 0;
  static bool hay_bola = false;
  MedidaExterna m;
  if (xQueueReceive(cola_externa, &m, pdMS_TO_TICKS(20)) == pdTRUE) {
    controlar(m.valida, m.x, m.y, m.con_velocidad, m.vx, m.vy);
    hay_bola = m.valida;
  }
  uint32_t ahora = millis();
  if (ahora - t_tasa >= 1000) { tasa = n_externas - n_tasa; n_tasa = n_externas; t_tasa = ahora; }
  if (!envio_ocupado && ahora - t_estado >= ESTADO_EXTERNO_MS) {
    t_estado = ahora;
    uint16_t estado = (modo_control == 1 ? 1 : 0) | (hay_bola ? 2 : 0) | (modo_control == 2 ? 4 : 0);
    Serial.printf("S%d,%d,%u,%lu,%d,%d\n", pulso_x_us, pulso_y_us, estado, (unsigned long)tasa,
                  (int)(ki * pid_x.integral), (int)(ki * pid_y.integral));
  }
}

// Nucleo 1 (loop de Arduino): deteccion de la bola, PID y servos
void loop() {
  if (millis() - t_ultima_externa < EXTERNA_VIGENTE_MS) { bucleExterno(); return; }
  static uint32_t cuadros = 0, t_ref = millis(), ultimo_envio = 0;
  static float fps = 0;
  static uint16_t diag[4] = {0, 0, 0, 0};

  uint8_t idx;
  if (xQueueReceive(cola_listos, &idx, pdMS_TO_TICKS(100)) != pdTRUE) return;
  const uint8_t *p = gris[idx];

  uint32_t t0 = micros();
  // ventana de seguimiento: con la bola localizada hace poco, solo se busca alrededor de donde se
  // predice que esta (ultima posicion + velocidad); a 50 FPS se mueve pocos pixeles por imagen y
  // el fondo, los reflejos y la mano fuera de la ventana no la confunden. Si se pierde, se busca
  // en toda la zona de interes
  static float ult_xf = 0, ult_yf = 0;
  static float vel_x = 0, vel_y = 0;  // pixeles por imagen, solo para predecir
  static uint8_t sin_valida = PERDIDA_MAX;
  bool con_ventana = sin_valida < PERDIDA_MAX;
  int16_t pred_x = (int16_t)(ult_xf + vel_x * (sin_valida + 1) + 0.5f);
  int16_t pred_y = (int16_t)(ult_yf + vel_y * (sin_valida + 1) + 0.5f);
  int x0 = roi_x0, y0 = roi_y0, x1 = roi_x1, y1 = roi_y1;
  if (con_ventana) {
    x0 = max(x0, pred_x - VENTANA); x1 = min(x1, pred_x + VENTANA + 1);
    y0 = max(y0, pred_y - VENTANA); y1 = min(y1, pred_y + VENTANA + 1);
  }

  // umbral: fijo si lo manda la laptop, si no a medio camino entre el fondo de la placa y el pixel
  // mas brillante de la zona de busqueda
  uint8_t u = umbral;
  if (u == 0) {
    uint16_t hist[256] = {0};
    uint32_t muestras_h = 0;
    for (int y = roi_y0; y < roi_y1; y += 2)
      for (int x = roi_x0; x < roi_x1; x += 2) { hist[p[y * ANCHO + x]]++; muestras_h++; }
    int fondo = 0;
    for (uint32_t acum = 0; fondo < 255 && (acum += hist[fondo]) < muestras_h / 2; fondo++) {}
    int pico = 0;
    for (int y = y0; y < y1; y++)
      for (int x = x0; x < x1; x++) pico = max(pico, (int)p[y * ANCHO + x]);
    u = pico - fondo >= CONTRASTE_MIN ? fondo + (int)(UMBRAL_FRACCION * (pico - fondo)) : 255;
  }

  // manchas: una bola gris sale partida en dos o tres trozos (el brillo de arriba y la parte
  // iluminada), asi que cada mancha se junta con los trozos cercanos antes de mirar si parece bola.
  // Entre los grupos que parecen bola, el mas cercano a la prediccion; sin prediccion, el de mas
  // brillo acumulado (la bola gana a los puntitos de ruido)
  for (int y = y0; y < y1; y++) memset(marcado + y * ANCHO + x0, 0, x1 - x0);
  static Mancha lista[MAX_MANCHAS];
  int manchas = 0;
  for (int y = y0; y < y1 && manchas < MAX_MANCHAS; y++) {
    const uint8_t *fila = p + y * ANCHO;
    for (int x = x0; x < x1 && manchas < MAX_MANCHAS; x++) {
      if (fila[x] <= u || marcado[y * ANCHO + x]) continue;
      Mancha m;
      crecerMancha(p, x, y, x0, y0, x1, y1, u, m);
      if (m.n >= TROZO_PX_MIN && m.sw > 0) lista[manchas++] = m;
    }
  }
  Mancha elegida = {};
  bool hay_mancha = false;
  float mejor = 1e30f;
  for (int i = 0; i < manchas; i++) {
    float ix = (float)lista[i].swx / lista[i].sw, iy = (float)lista[i].swy / lista[i].sw;
    Mancha g = {0, 0, 0, 0, lista[i].xmin, lista[i].xmax, lista[i].ymin, lista[i].ymax};
    for (int j = 0; j < manchas; j++) {
      const Mancha &m = lista[j];
      float dx = (float)m.swx / m.sw - ix, dy = (float)m.swy / m.sw - iy;
      if (dx * dx + dy * dy > RADIO_UNION * RADIO_UNION) continue;
      g.n += m.n; g.swx += m.swx; g.swy += m.swy; g.sw += m.sw;
      g.xmin = min(g.xmin, m.xmin); g.xmax = max(g.xmax, m.xmax);
      g.ymin = min(g.ymin, m.ymin); g.ymax = max(g.ymax, m.ymax);
    }
    if (!pareceBola(g)) continue;
    float gx = (float)g.swx / g.sw, gy = (float)g.swy / g.sw;
    float puntaje = con_ventana ? (gx - pred_x) * (gx - pred_x) + (gy - pred_y) * (gy - pred_y) : -(float)g.sw;
    if (puntaje < mejor) { mejor = puntaje; elegida = g; hay_mancha = true; }
  }
  // centroide ponderado por brillo: los pixeles del centro de la bola (mas brillantes) pesan mas
  // que los del borde, asi la posicion con decimales "baila" mucho menos
  uint32_t n = hay_mancha ? elegida.n : 0;
  float cxf = hay_mancha ? (float)elegida.swx / elegida.sw : -1;  // con decimales, para el PID
  float cyf = hay_mancha ? (float)elegida.swy / elegida.sw : -1;
  int16_t cx = hay_mancha ? (int16_t)(cxf + 0.5f) : -1, cy = hay_mancha ? (int16_t)(cyf + 0.5f) : -1;

  // solo el tablero cuenta para el brillo y para detectar imagenes corruptas: asi el fondo,
  // la mano o los reflejos de fuera no dan falsos positivos (1 de cada 2 pixeles en cada eje)
  const int tx0 = roi_x0, ty0 = roi_y0, tx1 = roi_x1, ty1 = roi_y1;
  uint32_t suma = 0, muestras = 0;
  for (int y = ty0; y < ty1; y += 2)
    for (int x = tx0; x < tx1; x += 2) { suma += p[y * ANCHO + x]; muestras++; }
  uint8_t brillo = muestras ? suma / muestras : 0;
  static uint8_t brillo_min = 255, brillo_max = 0;
  if (brillo < brillo_min) brillo_min = brillo;
  if (brillo > brillo_max) brillo_max = brillo;
  // imagen corrupta: el tablero entero cambia de golpe respecto a la imagen anterior
  // (diferencia media; con escena quieta es ~1-3)
  static uint8_t anterior[PIXELES / 4];
  static bool hay_anterior = false;
  uint32_t dif = 0;
  int j = 0;
  for (int y = ty0; y < ty1; y += 2)
    for (int x = tx0; x < tx1; x += 2, j++) {
      int d = (int)p[y * ANCHO + x] - anterior[j];
      dif += d < 0 ? -d : d;
      anterior[j] = p[y * ANCHO + x];
    }
  uint8_t dif_media = hay_anterior && muestras ? min(dif / muestras, (uint32_t)255) : 0;
  hay_anterior = true;
  static uint8_t n_saltos = 0, dif_max = 0;
  if (dif_media > dif_max) dif_max = dif_media;
  if (dif_media > SALTO_IMAGEN && n_saltos < 255) n_saltos++;
  uint32_t t_det = micros() - t0;

  // posicion valida para el control: se descartan imagenes corruptas y saltos imposibles
  bool valida = n >= BOLA_PX_MIN && n <= BOLA_PX_MAX && dif_media <= SALTO_IMAGEN;
  if (valida && sin_valida < PERDIDA_MAX &&
      (abs(cx - pred_x) > SALTO_BOLA || abs(cy - pred_y) > SALTO_BOLA)) valida = false;
  if (valida) {
    if (sin_valida < PERDIDA_MAX) {
      float k = 1.0f / (sin_valida + 1);
      vel_x = 0.5f * vel_x + 0.5f * (cxf - ult_xf) * k;
      vel_y = 0.5f * vel_y + 0.5f * (cyf - ult_yf) * k;
    } else {
      vel_x = vel_y = 0;
    }
    ult_xf = cxf; ult_yf = cyf; sin_valida = 0;
  }
  else if (sin_valida < 255) sin_valida++;

  // posicion de la bola en los ejes de la placa (pixeles desde el centro)
  bool hay_bola = sin_valida == 0;
  float x_placa = ult_xf - CENTRO_X;
  float y_placa = CENTRO_Y - ult_yf;

  controlar(hay_bola, x_placa, y_placa);

  cuadros++;
  uint32_t ahora = millis();
  if (ahora - t_ref >= 1000) {
    fps = cuadros * 1000.0f / (ahora - t_ref); cuadros = 0; t_ref = ahora;
    // autorreparacion: tras un bajon de tension el sensor puede quedarse mandando imagenes rotas
    // hasta reiniciarlo; con 2 s seguidos de muchas imagenes malas se reinicia la camara
    static uint8_t segundos_malos = 0;
    if (n_saltos > 10 || n_fallo_desc > 5) segundos_malos++;
    else segundos_malos = 0;
    if (segundos_malos >= 2) { reiniciar_camara = true; segundos_malos = 0; }
    diag[0] = (n_saltos << 8) | dif_max; n_saltos = 0; dif_max = 0; diag[1] = (uint16_t)min((uint32_t)max_len, (uint32_t)65535); diag[2] = (brillo_max << 8) | brillo_min; brillo_min = 255; brillo_max = 0; diag[3] = n_fallo_desc;
    n_fb = n_sin_fb = n_sin_buffer = n_fallo_desc = max_len = 0;
  }

  if (!envio_ocupado && ahora - ultimo_envio >= ENVIO_CADA_MS) {
    ultimo_envio = ahora;
    memcpy(envio, p, PIXELES);
    uint16_t estado = (modo_control == 1 ? 1 : 0) | (hay_bola ? 2 : 0) | (modo_control == 2 ? 4 : 0);
    uint16_t c[CAMPOS_CABECERA] = {(uint16_t)ANCHO, (uint16_t)ALTO, (uint16_t)cx, (uint16_t)cy,
                     (uint16_t)min(n, (uint32_t)65535), (uint16_t)(fps * 10), u,
                     (uint16_t)min((uint32_t)t_desc_us, (uint32_t)65535), (uint16_t)min(t_det, (uint32_t)65535),
                      diag[0], diag[1], diag[2], diag[3],
                      (uint16_t)pulso_x_us, (uint16_t)pulso_y_us, estado,
                      pulso_medido_x, pulso_medido_y,
                      (uint16_t)(32768 + (int)(ki * pid_x.integral)), (uint16_t)(32768 + (int)(ki * pid_y.integral))};
    memcpy(cabecera_envio, c, sizeof(c));
    envio_ocupado = true;
  }
  xQueueSend(cola_libres, &idx, portMAX_DELAY);
}
