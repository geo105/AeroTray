#!/usr/bin/env bash
set -Eeuo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
"$BASE_DIR/herramientas/arduino-cli/arduino-cli" \
  --config-file "$BASE_DIR/herramientas/arduino-cli.yaml" compile \
  --fqbn 'esp32:esp32:nodemcu-32s' \
  --build-path "$BASE_DIR/tmp/build_dron_octo_esp32" \
  --output-dir "$BASE_DIR/firmware/bin/dron_octo_esp32" \
  "$BASE_DIR/firmware/dron_octo_esp32"
