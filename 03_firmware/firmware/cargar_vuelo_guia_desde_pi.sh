#!/usr/bin/env bash
# Graba en el NodeMCU del dron el firmware de vuelo (o vuelve al de banco) DESDE LA RASPBERRY PI,
# y actualiza el programa de la Pi (comunicacion.py y pagina_dron.py) para las ordenes de vuelo.
#
#   ./cargar_vuelo_guia_desde_pi.sh            -> graba captura_octo_m8_vuelo_guia
#   ./cargar_vuelo_guia_desde_pi.sh --volver   -> vuelve a captura_octo_m8_banco_angulo_laser (el de siempre)
#
# Antes: LiPo de los motores DESCONECTADAS. Detiene seguidor.py (libera el USB del dron), graba y lo vuelve
# a arrancar. Los pasos remotos van en un archivo: si fueran en la linea de ssh, pkill mataria su propia sesion.
set -Eeuo pipefail
PI=${PI:-pi@raspberrypi.local}
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == "--volver" ]]; then
  BIN="$BASE/firmware/bin/captura_octo_m8_banco_angulo_laser/captura_octo_m8_banco_angulo_laser.ino.merged.bin"
else
  BIN="$BASE/firmware/bin/captura_octo_m8_vuelo_guia/captura_octo_m8_vuelo_guia.ino.merged.bin"
fi
echo "Pi: $PI"; echo "Firmware: $(basename "$BIN")"
REMOTO=$(mktemp)
cat > "$REMOTO" <<'EOF'
set -e
pkill -f "seguidor.py" || true          # el lazo de arranque y el programa (todos llevan seguidor.py)
sleep 2
PUERTO=$(ls /dev/serial/by-id/*CP2102* 2>/dev/null | head -1)
[ -n "$PUERTO" ] || { echo "No encuentro el USB del dron (CP2102)"; exit 1; }
echo "Puerto del dron: $PUERTO"
~/venv/bin/python -m esptool --chip esp32 --port "$PUERTO" --baud 460800 write-flash 0x0 /tmp/dron.merged.bin
cd ~/pi_control
setsid nohup sh ./arrancar.sh >/dev/null 2>&1 < /dev/null &
sleep 10
echo "--- arranque del dron:"
grep -a "OCTO M8\|VUELO:" salida.log | tail -2
EOF
scp -q "$BIN" "$PI:/tmp/dron.merged.bin"
scp -q "$REMOTO" "$PI:/tmp/grabar_dron.sh"
rm -f "$REMOTO"
scp -q "$BASE/pi_control/comunicacion.py" "$BASE/pi_control/pagina_dron.py" "$BASE/pi_control/seguidor.py" "$PI:pi_control/"
ssh "$PI" bash /tmp/grabar_dron.sh
echo "Listo. Abre http://${PI#*@}:8080/dron : con el firmware de vuelo aparece el panel «Vuelo en la guía» activo."
