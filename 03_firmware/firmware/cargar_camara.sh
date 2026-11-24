#!/usr/bin/env bash
set -euo pipefail

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLI="$RAIZ/herramientas/arduino-cli/arduino-cli"
CFG="$RAIZ/herramientas/arduino-cli.yaml"
SKETCH="$RAIZ/firmware/camara_s3_diagnostico"
BIN="$RAIZ/firmware/bin/camara_s3_diagnostico"
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
  echo "Conecta el ESP32-S3-CAM por USB y vuelve a ejecutar:"
  echo "  bash /home/george/DSM/firmware/cargar_camara.sh"
  exit 1
fi

echo "==> Compilando y subiendo camara_s3_diagnostico (v0.5 optimizada) a $PUERTO..."
bash "$RAIZ/firmware/compilar_camara.sh"
"$CLI" --config-file "$CFG" upload \
  --fqbn "$FQBN" \
  --port "$PUERTO" \
  --input-dir "$BIN" \
  "$SKETCH"

echo "==> Listo. Ahora abre el panel con: bash /home/george/DSM/panel_ball_plate.sh"
