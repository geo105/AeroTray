#!/usr/bin/env bash
set -Eeuo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

PUERTO="${1:-}"
if [[ -z "$PUERTO" ]]; then
  for p in /dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM*; do
    if [[ -e "$p" ]]; then
      PUERTO="$p"
      break
    fi
  done
fi

if [[ -z "$PUERTO" ]]; then
  echo "ERROR: No se detecto puerto serie (/dev/ttyUSB* ni /dev/ttyACM*)."
  echo "Conecta el ESP32 NodeMCU-32S del dron por USB y vuelve a ejecutar:"
  echo "  bash firmware/cargar_diagnostico_imu.sh /dev/ttyUSB0"
  exit 1
fi

echo "==> Subiendo diagnostico de IMU/Giroscopio a $PUERTO..."
bash "$BASE_DIR/firmware/cargar_diagnostico_dron.sh" imu "$PUERTO"
echo ""
echo "==> CARGA LISTA. Para probar el giro en vivo en consola ejecuta:"
echo "    python3 herramientas/probar_imu.py $PUERTO"
