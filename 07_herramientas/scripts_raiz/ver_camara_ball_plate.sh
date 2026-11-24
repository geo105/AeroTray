#!/usr/bin/env bash
set -Eeuo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -eq 0 ]]; then
  CAMERA_PORT="${DSM_CAMARA_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
  if [[ ! -e "$CAMERA_PORT" ]]; then
    echo "No se encuentra la camara. Indica el puerto:"
    echo "bash $BASE_DIR/ver_camara_ball_plate.sh --usb /dev/ttyUSB1 --calibracion $BASE_DIR/estado_camara/calibracion_panel.json"
    exit 1
  fi
  exec python3 "$BASE_DIR/herramientas/seguimiento_aruco_camara.py" \
    --usb "$CAMERA_PORT" --baudrate 921600 \
    --calibracion "$BASE_DIR/estado_camara/calibracion_panel.json"
fi
exec python3 "$BASE_DIR/herramientas/seguimiento_aruco_camara.py" "$@"
