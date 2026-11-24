#!/usr/bin/env bash
set -Eeuo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -ne 2 || ( "$1" != laser && "$1" != motores && "$1" != imu ) ]]; then
  echo "Uso: bash $0 imu|laser|motores /dev/PUERTO_DEL_NODEMCU";exit 1
fi
SKETCH="${1}_diagnostico_esp32"
# Siempre compilar el fuente actual; nunca reutilizar un binario desactualizado.
"$BASE_DIR/herramientas/arduino-cli/arduino-cli" --config-file "$BASE_DIR/herramientas/arduino-cli.yaml" compile \
  --fqbn esp32:esp32:nodemcu-32s --build-path "$BASE_DIR/tmp/build_$SKETCH" \
  --output-dir "$BASE_DIR/firmware/bin/$SKETCH" "$BASE_DIR/firmware/$SKETCH"
# La configuracion NodeMCU exige chip ESP32: no seleccionar el puerto S3 de camara.
"$BASE_DIR/herramientas/arduino-cli/arduino-cli" --config-file "$BASE_DIR/herramientas/arduino-cli.yaml" upload \
  --fqbn esp32:esp32:nodemcu-32s:UploadSpeed=115200 --upload-property upload.flags=--no-stub \
  --port "$2" --input-dir "$BASE_DIR/firmware/bin/$SKETCH" "$BASE_DIR/firmware/$SKETCH"
