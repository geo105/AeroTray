#!/usr/bin/env bash
set -Eeuo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
"$BASE_DIR/herramientas/arduino-cli/arduino-cli" \
  --config-file "$BASE_DIR/herramientas/arduino-cli.yaml" compile \
  --fqbn 'esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=huge_app,CDCOnBoot=default' \
  --build-path "$BASE_DIR/tmp/build_ball_plate_camara_s3" \
  --output-dir "$BASE_DIR/firmware/bin/ball_plate_camara_s3" \
  "$BASE_DIR/firmware/ball_plate_camara_s3"
