/*
 * DSM: primera prueba ESP32-S3-CAM N16R8 + OV2640.
 * Camara + prueba limitada de servos GPIO41/42. No controla motores del dron.
 * Boot sin PWM. Centros confirmados: 90 grados. Pruebas de hasta +-2 grados.
 * Perfil de pines provisional documentado en pines_camara.h y LEEME.md.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "pines_camara.h"
#include "servos_bandeja.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Selecciona ESP32S3 Dev Module para este programa."
#endif

constexpr char RED[] = "BolaCam-prueba";
constexpr char CLAVE[] = "ballplate1";  // Red local temporal de pruebas.
WebServer web(80);
SemaphoreHandle_t mutexFoto = nullptr;
uint8_t *ultimaFoto = nullptr;
bool listo = false, webActiva = false;
uint32_t baudActual = 115200;

struct Estadisticas {
  uint32_t recibidas = 0, fallos = 0, descartadas = 0;
  size_t bytes = 0;
  uint32_t capturaUs = 0;
  int64_t tomadaUs = 0;
  float fps = 0;
} datos;

String estadoJson() {
  Estadisticas e;
  if (xSemaphoreTake(mutexFoto, pdMS_TO_TICKS(100)) != pdTRUE) {
    return "{\"error\":\"foto ocupada\"}";
  }
  e = datos;
  xSemaphoreGive(mutexFoto);
  const int64_t edad = e.tomadaUs ? (esp_timer_get_time() - e.tomadaUs) / 1000 : -1;
  String s = "{\"version\":\"camara-diag-0.5\",\"frames\":";
  s += e.recibidas;
  s += ",\"fallos\":"; s += e.fallos;
  s += ",\"descartadas\":"; s += e.descartadas;
  s += ",\"bytes\":"; s += static_cast<unsigned>(e.bytes);
  s += ",\"fps_captura\":"; s += String(e.fps, 2);
  s += ",\"captura_ms\":"; s += String(e.capturaUs / 1000.0f, 2);
  s += ",\"edad_foto_ms\":"; s += String(static_cast<long>(edad));
  s += ",\"flash_bytes\":"; s += ESP.getFlashChipSize();
  s += ",\"psram_bytes\":"; s += ESP.getPsramSize();
  s += ",\"psram_libre\":"; s += ESP.getFreePsram();
  s += ",\"heap_libre\":"; s += ESP.getFreeHeap();
  s += ",\"baudrate\":"; s += baudActual;
  s += ",\"baud_max\":2000000";
  s += ",\"servos\":";s += bandeja::estadoJson();s += "}";
  return s;
}

void tareaCaptura(void *) {
  uint32_t inicioVentana = millis(), enVentana = 0;
  for (;;) {
    const int64_t t0 = esp_timer_get_time();
    camera_fb_t *fb = esp_camera_fb_get();
    const uint32_t duracion = static_cast<uint32_t>(esp_timer_get_time() - t0);
    if (xSemaphoreTake(mutexFoto, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (!fb) {
        datos.fallos++;
      } else {
        datos.recibidas++;
        enVentana++;
        datos.capturaUs = duracion;
        if (fb->format == PIXFORMAT_JPEG && fb->len > 0 && fb->len <= camhw::capacidadFoto) {
          memcpy(ultimaFoto, fb->buf, fb->len);
          datos.bytes = fb->len;
          datos.tomadaUs = static_cast<int64_t>(fb->timestamp.tv_sec) * 1000000 + fb->timestamp.tv_usec;
        } else {
          datos.descartadas++;
        }
      }
      const uint32_t transcurrido = millis() - inicioVentana;
      if (transcurrido >= 2000) {
        datos.fps = enVentana * 1000.0f / transcurrido;
        enVentana = 0;
        inicioVentana = millis();
      }
      xSemaphoreGive(mutexFoto);
    }
    if (fb) esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(fb ? 1 : 100));
  }
}

void enviarFoto() {
  // Copia independiente: una conexion Wi-Fi lenta no retiene el mutex de captura.
  auto *copia = static_cast<uint8_t *>(heap_caps_malloc(camhw::capacidadFoto, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!copia) { web.send(503, "text/plain", "No hay memoria para la foto"); return; }
  size_t n = 0;
  if (xSemaphoreTake(mutexFoto, pdMS_TO_TICKS(100)) == pdTRUE) {
    n = datos.bytes;
    if (n) memcpy(copia, ultimaFoto, n);
    xSemaphoreGive(mutexFoto);
  }
  if (!n) {
    free(copia);
    web.send(503, "text/plain", "Todavia no hay una foto valida. Revisa /estado y el puerto serie.");
    return;
  }
  web.sendHeader("Cache-Control", "no-store");
  web.sendHeader("Content-Disposition", "inline; filename=foto.jpg");
  web.send_P(200, "image/jpeg", reinterpret_cast<const char *>(copia), n);
  free(copia);
}

void fotoSerie() {
  auto *copia = static_cast<uint8_t *>(heap_caps_malloc(camhw::capacidadFoto, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!copia) { Serial.println("# ERROR_FOTO memoria"); return; }
  size_t n = 0;
  if (xSemaphoreTake(mutexFoto, pdMS_TO_TICKS(100)) == pdTRUE) {
    n = datos.bytes;
    if (n) memcpy(copia, ultimaFoto, n);
    xSemaphoreGive(mutexFoto);
  }
  if (!n) {
    free(copia);
    Serial.println("# ERROR_FOTO sin imagen valida");
    return;
  }
  Serial.printf("# FOTO_JPEG %u\n", static_cast<unsigned>(n));
  Serial.write(copia, n);
  Serial.flush();
  Serial.println("\n# FIN_FOTO");
  free(copia);
}

void enviarPagina() {
  static const char pagina[] PROGMEM = R"HTML(<!doctype html><html lang="es"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Prueba de camara</title>
<style>body{font:18px system-ui;max-width:760px;margin:24px auto;padding:0 16px;background:#fafafa;color:#18212b}img{width:100%;image-rendering:auto}pre{white-space:pre-wrap;font-size:14px}button{font:inherit;padding:10px;margin:8px 0}</style>
<h1>Camara del ball and plate</h1><p>Prueba de imagen y memoria. Servos desactivados.</p>
<img id="foto" alt="Esperando imagen"><p><button onclick="foto()">Tomar foto</button> <a href="/foto.jpg" target="_blank">Abrir foto</a></p>
<p>Las fotos de esta pagina se actualizan cada segundo. El indicador de FPS mide la captura interna.</p><pre id="estado">Esperando estado</pre>
<script>let ocupada=false;function foto(){document.getElementById('foto').src='/foto.jpg?t='+Date.now()}
async function estado(){if(ocupada)return;ocupada=true;try{let r=await fetch('/estado',{cache:'no-store'});document.getElementById('estado').textContent=JSON.stringify(await r.json(),null,2)}catch(e){document.getElementById('estado').textContent='Sin conexion'}finally{ocupada=false}}
foto();estado();setInterval(foto,1000);setInterval(estado,2000);</script></html>)HTML";
  web.send_P(200, "text/html; charset=utf-8", pagina);
}

void iniciarWeb() {
  if (webActiva) { Serial.println("# WiFi de prueba ya activo."); return; }
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(RED, CLAVE)) { Serial.println("# ERROR al iniciar la red de prueba."); return; }
  web.on("/", HTTP_GET, enviarPagina);
  web.on("/foto.jpg", HTTP_GET, enviarFoto);
  web.on("/estado", HTTP_GET, []() { web.sendHeader("Cache-Control", "no-store"); web.send(200, "application/json", estadoJson()); });
  web.begin();
  webActiva = true;
  Serial.printf("# Red: %s | clave: %s | abre http://%s\n", RED, CLAVE, WiFi.softAPIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n# DSM camara-diag-0.5: camara + prueba servos 41/42. Arranque SIN PWM.");
  pinMode(camhw::futuroServoX, INPUT);
  pinMode(camhw::futuroServoY, INPUT);
  Serial.printf("# Flash: %u bytes. PSRAM: %u bytes.\n", ESP.getFlashChipSize(), ESP.getPsramSize());
  if (!psramFound()) {
    Serial.println("# ERROR: no se detecta PSRAM. Selecciona OPI PSRAM y revisa que sea la placa S3 N16R8.");
    return;
  }
  ultimaFoto = static_cast<uint8_t *>(heap_caps_malloc(camhw::capacidadFoto, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  mutexFoto = xSemaphoreCreateMutex();
  if (!ultimaFoto || !mutexFoto) { Serial.println("# ERROR: no se pudieron reservar los recursos."); return; }

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = camhw::d0; c.pin_d1 = camhw::d1; c.pin_d2 = camhw::d2; c.pin_d3 = camhw::d3;
  c.pin_d4 = camhw::d4; c.pin_d5 = camhw::d5; c.pin_d6 = camhw::d6; c.pin_d7 = camhw::d7;
  c.pin_xclk = camhw::xclk; c.pin_pclk = camhw::pclk;
  c.pin_vsync = camhw::vsync; c.pin_href = camhw::href;
  c.pin_sccb_sda = camhw::sda; c.pin_sccb_scl = camhw::scl;
  c.pin_pwdn = camhw::pwdn; c.pin_reset = camhw::reset;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_QVGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  Serial.println("# Probando perfil de pines ESP32S3_EYE. Compatibilidad RE1.3 pendiente de esta prueba.");
  const esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("# ERROR camara 0x%x (%s). No se cambia de mapa automaticamente.\n", err, esp_err_to_name(err));
    return;
  }
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor) Serial.printf("# Sensor detectado: PID 0x%04x (OV2640 esperado 0x0026).\n", sensor->id.PID);

  if (xTaskCreatePinnedToCore(tareaCaptura, "captura", 4096, nullptr, 1, nullptr, 0) != pdPASS) {
    Serial.println("# ERROR al iniciar captura."); return;
  }
  if(!bandeja::iniciar()){Serial.println("# ERROR: tarea de servos no disponible");return;}
  listo = true;
  Serial.println("# Comandos: ver | foto | wifi | baud 115200|460800|921600|1500000|2000000 (reset vuelve a 115200).");
  Serial.println("# Servos: hb | servo_arm 90 90 | servo_test 41|42 delta ms | servo_centro | servo_off | !");
}

void loop() {
  if (!listo) { delay(250); return; }
  if (webActiva) web.handleClient();
  static char linea[64]; static uint8_t n = 0;static bool desbordada=false;
  static uint32_t anterior = 0;
  while (Serial.available()) {
    anterior = millis();
    const char ch = Serial.read();
    if(ch=='!'){bandeja::comando("!");n=0;desbordada=false;continue;}
    if (ch == '\n') {
      linea[n] = '\0';
      if(desbordada){n=0;desbordada=false;continue;}
      if (!strcmp(linea, "ver")) Serial.println(estadoJson());
      else if (!strcmp(linea, "foto")) fotoSerie();
      else if (!strcmp(linea, "wifi")) iniciarWeb();
      else if(bandeja::comando(linea)){}
      else if (!strncmp(linea, "baud ", 5)) {
        const unsigned long baud = strtoul(linea + 5, nullptr, 10);
        if (baud == 115200 || baud == 460800 || baud == 921600 || baud == 1500000 || baud == 2000000) {
          // Confirmar en la velocidad vieja antes de cambiar ambos extremos.
          Serial.printf("# BAUD %lu\n", baud);
          Serial.flush();
          delay(50);
          Serial.updateBaudRate(baud);
          baudActual = baud;
        } else Serial.println("# ERROR_BAUD usa 115200, 460800, 921600, 1500000 o 2000000");
      }
      n = 0;
      anterior = millis();
    } else if (ch != '\r') {if(static_cast<size_t>(n)+1<sizeof linea)linea[n++]=ch;else desbordada=true;}
  }
  if (baudActual == 115200 && millis() - anterior >= 5000) { anterior = millis(); Serial.println(estadoJson()); }
  delay(1);
}
