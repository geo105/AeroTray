#!/usr/bin/env bash
set -euo pipefail

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLI="$RAIZ/herramientas/arduino-cli/arduino-cli"
CFG="$RAIZ/herramientas/arduino-cli.yaml"
SKETCH="$RAIZ/firmware/ball_plate_camara_s3"
BIN="$RAIZ/firmware/bin/ball_plate_camara_s3"
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=huge_app,CDCOnBoot=default"

PUERTO="${1:-}"
if [[ -z "$PUERTO" ]]; then
  for p in /dev/ttyACM* /dev/ttyUSB*; do
    if [[ -e "$p" ]]; then
      PUERTO="$p"
      break
    fi
  done
fi

if [[ -z "$PUERTO" ]]; then
  echo "ERROR: No se detecto puerto USB (/dev/ttyACM* ni /dev/ttyUSB*)."
  echo "Conecta el ESP32-S3-CAM N16R8 por el puerto USB-C UART (izquierdo) o OTG y ejecuta:"
  echo "  bash firmware/cargar_ball_plate_s3.sh"
  exit 1
fi

echo "==> Compilando y subiendo ball_plate_camara_s3 a $PUERTO ($FQBN)..."
"$RAIZ/firmware/compilar_ball_plate_s3.sh"
"$CLI" --config-file "$CFG" upload \
  --fqbn "$FQBN" \
  --port "$PUERTO" \
  --input-dir "$BIN" \
  "$SKETCH"

echo "==> Listo. Abre el monitor serie a 115200 baudios o manda 'wifi' y conecta al WiFi 'BolaCam-prueba' (clave ballplate1)"
