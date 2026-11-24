#!/usr/bin/env bash
set -Eeuo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
for SKETCH in imu_diagnostico_esp32 laser_diagnostico_esp32 motores_diagnostico_esp32; do
  "$BASE_DIR/herramientas/arduino-cli/arduino-cli" \
    --config-file "$BASE_DIR/herramientas/arduino-cli.yaml" compile \
    --fqbn esp32:esp32:nodemcu-32s \
    --build-path "$BASE_DIR/tmp/build_$SKETCH" \
    --output-dir "$BASE_DIR/firmware/bin/$SKETCH" "$BASE_DIR/firmware/$SKETCH"
done
