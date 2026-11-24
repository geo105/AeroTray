#!/usr/bin/env bash
set -Eeuo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
"$BASE_DIR/herramientas/arduino-cli/arduino-cli" \
  --config-file "$BASE_DIR/herramientas/arduino-cli.yaml" compile \
  --fqbn 'esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=huge_app,CDCOnBoot=default' \
  --build-path "$BASE_DIR/tmp/build_camara_diag" \
  --output-dir "$BASE_DIR/firmware/bin/camara_s3_diagnostico" \
  "$BASE_DIR/firmware/camara_s3_diagnostico"
