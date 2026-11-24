/*
  ESP32-C3 SuperMini: simulador UART del NI myRIO-1900

  USB (portatil) <--> Serial  <--> ESP32-C3 <--> Serial1 <--> Raspberry Pi 3

  UART Raspberry: 9600 baudios, 8 bits, sin paridad, 1 bit de parada (8N1).
  Solo se reenvia al Raspberry una linea USB valida de DOS campos CSV:
      4,6<LF>
  Las lineas que la Raspberry devuelve se muestran por USB. Se espera una
  telemetria de SEIS campos CSV:
      posX,posY,roll,pitch,uav_activo,bp_activo<LF>

  Cableado UART TTL 3.3 V (NO usar 5 V en las senales):
    ESP32-C3 GPIO 7 (TX) --> Raspberry GPIO 15 / pin fisico 10 (RXD)
    ESP32-C3 GPIO 6 (RX) <-- Raspberry GPIO 14 / pin fisico  8 (TXD)
    ESP32-C3 GND          --- Raspberry GND       / pin fisico  6

  La USB queda conectada al portatil para flashear y usar el Monitor Serie.
  GPIO18 y GPIO19 se dejan libres: son D-/D+ de USB nativo en ESP32-C3.

  Arduino IDE:
    Board: ESP32C3 Dev Module (o ESP32-C3 SuperMini si aparece)
    USB CDC On Boot: Enabled, si la placa usa USB nativo
    Monitor serie: 115200 baudios, Line ending: Newline o Both NL & CR
*/

#include <Arduino.h>
#include <math.h>
#include <string.h>

constexpr uint32_t BAUDIOS_USB = 115200;
constexpr uint32_t BAUDIOS_RASPBERRY = 9600;
constexpr int PIN_UART_RX = 6;  // ESP recibe: conectado al TX de Raspberry
constexpr int PIN_UART_TX = 7;  // ESP transmite: conectado al RX de Raspberry
constexpr size_t MAX_LINEA = 96;

HardwareSerial uartRaspberry(1);

char bufferUsb[MAX_LINEA + 1];
size_t longitudUsb = 0;
bool descartandoUsb = false;
char bufferRaspberry[MAX_LINEA + 1];
size_t longitudRaspberry = 0;
bool descartandoRaspberry = false;

// Devuelve true solamente si "linea" tiene exactamente cantidadEsperada
// numeros finitos separados por comas. No modifica el texto que se reenviara.
bool esCsvNumerico(char *linea, uint8_t cantidadEsperada) {
  uint8_t cantidad = 0;
  char *inicio = linea;

  while (true) {
    char *coma = strchr(inicio, ',');
    if (coma != nullptr) *coma = '\0';

    if (*inicio == '\0') return false;
    char *fin = nullptr;
    const float valor = strtof(inicio, &fin);
    if (fin == inicio || *fin != '\0' || !isfinite(valor)) return false;
    cantidad++;

    if (coma == nullptr) break;
    inicio = coma + 1;
  }
  return cantidad == cantidadEsperada;
}

void procesarComandoUsb(char *linea) {
  // Copia para validar sin cambiar la trama original que se entrega al Raspberry.
  char copia[MAX_LINEA + 1];
  strncpy(copia, linea, MAX_LINEA);
  copia[MAX_LINEA] = '\0';

  if (!esCsvNumerico(copia, 2)) {
    Serial.print(F("[ERROR] Comando descartado. Use exactamente dos numeros: 4,6\\n. Recibido: "));
    Serial.println(linea);
    return;
  }

  uartRaspberry.print(linea);
  uartRaspberry.write('\n');  // El protocolo Raspberry usa LF como delimitador.
  Serial.print(F("[USB -> RPi] "));
  Serial.println(linea);
}

void procesarTelemetriaRaspberry(char *linea) {
  char copia[MAX_LINEA + 1];
  strncpy(copia, linea, MAX_LINEA);
  copia[MAX_LINEA] = '\0';

  Serial.print(F("[RPi -> USB] "));
  Serial.print(linea);
  if (esCsvNumerico(copia, 6)) {
    Serial.println(F("  [telemetria valida: X,Y,roll,pitch,UAV,BP]"));
  } else {
    Serial.println(F("  [aviso: no tiene seis numeros CSV]"));
  }
}

// Lee caracteres de una interfaz, acumula hasta LF y entrega una linea completa.
// CR se ignora para aceptar el modo CRLF del Monitor Serie.
template <typename T>
void leerLineas(T &puerto, char *buffer, size_t &longitud, bool &descartando,
                void (*alCompletar)(char *)) {
  while (puerto.available() > 0) {
    const char caracter = static_cast<char>(puerto.read());
    if (caracter == '\r') continue;

    if (caracter == '\n') {
      if (!descartando && longitud > 0) {
        buffer[longitud] = '\0';
        alCompletar(buffer);
      }
      longitud = 0;
      descartando = false;
      continue;
    }

    if (descartando) continue;

    if (longitud < MAX_LINEA) {
      buffer[longitud++] = caracter;
    } else {
      // Se descarta la linea sobredimensionada hasta el siguiente LF.
      Serial.println(F("[ERROR] Linea mayor a 96 caracteres; descartada."));
      longitud = 0;
      descartando = true;
    }
  }
}

void setup() {
  Serial.begin(BAUDIOS_USB);
  // No se bloquea esperando USB: permite que la UART con la Raspberry opere sola.
  const uint32_t inicio = millis();
  while (!Serial && millis() - inicio < 1500) delay(10);
  Serial.println();
  Serial.println(F("=== ESP32-C3 simulador myRIO ==="));
  Serial.printf("USB: %lu; UART Raspberry: %lu 8N1 (RX GPIO%d, TX GPIO%d)\n",
                BAUDIOS_USB, BAUDIOS_RASPBERRY, PIN_UART_RX, PIN_UART_TX);
  Serial.println(F("Escriba, por ejemplo: 4,6 y envie LF."));
  uartRaspberry.begin(BAUDIOS_RASPBERRY, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
}

void loop() {
  leerLineas(Serial, bufferUsb, longitudUsb, descartandoUsb, procesarComandoUsb);
  leerLineas(uartRaspberry, bufferRaspberry, longitudRaspberry, descartandoRaspberry,
             procesarTelemetriaRaspberry);
}
