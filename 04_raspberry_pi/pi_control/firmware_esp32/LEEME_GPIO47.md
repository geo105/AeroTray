# Firmware: servo X en GPIO47, servo Y en GPIO21

Actualización del 7 de noviembre de 2026. Compilación correcta con PlatformIO.
El único cambio de código es `SERVO_X_PIN`: 14 → 47. Se conservan GPIO21,
PID, neutros, límites, cámara y protocolo de la Raspberry Pi.

Con la alimentación desconectada, mueve únicamente la señal del servo X
del GPIO14 al GPIO47. La señal del servo Y permanece en GPIO21.
Conserva la alimentación externa de servos y el GND común con el ESP32.

Estos binarios requieren cargarse en el ESP32 para aplicar el cambio.
La compilación y copia de archivos no actualizan por sí solas la placa.
En esta intervención no se grabó: no había un ESP32 conectado a la laptop
ni a la Raspberry Pi. No se probó físicamente la salida GPIO47.

Para cargar desde la Pi, con los servos sin alimentación y verificando
que `/dev/ttyUSB0` sea este ESP32-S3, detén temporalmente el supervisor
de `seguidor.py` para que libere el puerto. Desde `~/firmware_esp32`:

```bash
~/venv-esp/bin/python -m esptool --chip esp32s3 --port /dev/ttyUSB0 --baud 921600 write-flash -z \
  --flash-mode dio --flash-freq 80m --flash-size 16MB \
  0x0 bootloader.bin 0x8000 partitions.bin 0xe000 boot_app0.bin 0x10000 firmware.bin
```

Después de la carga, vuelve a iniciar `~/pi_control/arrancar.sh`.
Las versiones históricas del firmware conservan sus pines originales.
