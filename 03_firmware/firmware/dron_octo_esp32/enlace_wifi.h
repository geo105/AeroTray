// Enlace WiFi (UDP) del dron octo: mismo monitor de comandos y telemetria que el puerto serie,
// pero sin cable. El ESP32 crea su propia red; la laptop se conecta a ella y el puente ROS 2
// usa --wifi. Nada de esto bloquea el lazo de 250 Hz: el lazo solo copia bytes a un buffer y
// una tarea de baja prioridad en el nucleo 0 los envia por UDP (si se llena, se descartan).
#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

namespace enlace {
constexpr char RED[] = "DSM-Octo";
constexpr char CLAVE[] = "octo2026";
constexpr uint16_t PUERTO = 4210;

StreamBufferHandle_t bufTx = nullptr;
StreamBufferHandle_t bufRx = nullptr;
WiFiUDP udp;
IPAddress ipPc;
volatile uint16_t puertoPc = 0;
volatile bool activo = false;

void tarea(void *) {
  static uint8_t pkt[1200];
  static uint8_t rx[256];
  for (;;) {
    int n = udp.parsePacket();
    while (n > 0) {
      ipPc = udp.remoteIP();
      puertoPc = udp.remotePort();
      int k = udp.read(rx, sizeof(rx));
      if (k > 0) xStreamBufferSend(bufRx, rx, (size_t)k, 0);
      n = udp.parsePacket();
    }
    size_t m = xStreamBufferReceive(bufTx, pkt, sizeof(pkt), pdMS_TO_TICKS(4));
    if (m > 0 && puertoPc) {
      udp.beginPacket(ipPc, puertoPc);
      udp.write(pkt, m);
      udp.endPacket();
    }
  }
}

bool iniciar() {
  bufTx = xStreamBufferCreate(4096, 1);
  bufRx = xStreamBufferCreate(512, 1);
  if (!bufTx || !bufRx) return false;
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(RED, CLAVE)) return false;
  WiFi.setSleep(false);
  udp.begin(PUERTO);
  activo = true;
  xTaskCreatePinnedToCore(tarea, "enlace_udp", 4096, nullptr, 1, nullptr, 0);
  return true;
}

// Byte recibido por WiFi o -1 si no hay
inline int leerByte() {
  if (!activo) return -1;
  uint8_t c;
  return (xStreamBufferReceive(bufRx, &c, 1, 0) == 1) ? (int)c : -1;
}
}  // namespace enlace

// Salida doble: todo lo que antes iba a Serial va tambien al PC por WiFi (sin bloquear)
class SalidaDoble : public Print {
 public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    Serial.write(b, n);
    if (enlace::activo && enlace::puertoPc) xStreamBufferSend(enlace::bufTx, b, n, 0);
    return n;
  }
};
SalidaDoble Out;

// Byte de entrada desde USB o WiFi (-1 si no hay)
inline int leerByteEntrada() {
  if (Serial.available()) return Serial.read();
  return enlace::leerByte();
}
