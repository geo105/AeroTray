#!/usr/bin/env bash
# Comprueba 3 veces que el NodeMCU-32S lee su memoria flash ANTES de cargar firmware.
# Bien: "Manufacturer: 5e  Device: 4016  4MB" y cristal 40 MHz en los 3 intentos.
# Uso: bash ~/DSM/firmware/probar_flash_esp32.sh [/dev/ttyUSB0]
set -uo pipefail
PUERTO="${1:-/dev/ttyUSB0}"
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ESPTOOL="$RAIZ/herramientas/arduino-data/packages/esp32/tools/esptool_py/5.3.1/esptool"
[[ -x "$ESPTOOL" ]] || { echo "No encuentro esptool en $ESPTOOL"; exit 1; }
[[ -e "$PUERTO" ]] || { echo "No existe $PUERTO. Revisa: ls /dev/ttyUSB* /dev/ttyACM*"; exit 1; }
ok=0
for i in 1 2 3; do
  echo "== Intento $i/3 =="
  salida="$("$ESPTOOL" --port "$PUERTO" --baud 115200 --no-stub flash-id 2>&1)"
  echo "$salida" | grep -E "Crystal|MAC|Manufacturer|Device:|flash size|rror|arning" || echo "$salida" | tail -3
  if grep -q "Detected flash size: 4MB" <<<"$salida" && grep -q "Crystal frequency: *40MHz" <<<"$salida"; then
    ok=$((ok+1))
  fi
  sleep 1
done
echo
if [[ $ok -eq 3 ]]; then
  echo "FLASH OK (3/3). Ya se puede cargar: bash $RAIZ/firmware/cargar_dron_octo.sh $PUERTO"
else
  echo "FLASH INESTABLE ($ok/3). NO cargar todavia: revisar alimentacion, cable y la placa fuera de la galleta."
  exit 2
fi
