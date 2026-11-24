#!/usr/bin/env bash
# Graba en el ESP32-S3 de la bandeja el firmware con MODO SENO (ensayo E2) y sube seguidor.py con la ruta /seno.
# Solo graba la aplicacion (0x10000); bootloader y particiones quedan como estan.
#   ./cargar_bandeja_seno_desde_pi.sh            -> firmware con seno
#   ./cargar_bandeja_seno_desde_pi.sh --volver   -> vuelve al firmware.bin anterior (07/11)
# Antes: servos sin alimentacion (o bandeja sin la bola) y LiPo del dron desconectadas.
set -Eeuo pipefail
PI=${PI:-pi@raspberrypi.local}
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$BASE/pi_control/firmware_esp32/firmware_seno.bin"
[[ "${1:-}" == "--volver" ]] && BIN="$BASE/pi_control/firmware_esp32/firmware.bin"
echo "Pi: $PI  ->  $(basename "$BIN")"
REMOTO=$(mktemp)
cat > "$REMOTO" <<'EOS'
set -e
pkill -f "seguidor.py" || true
sleep 2
PUERTO=$(ls /dev/serial/by-id/*1a86* 2>/dev/null | head -1)
[ -n "$PUERTO" ] || { echo "No encuentro el USB del ESP32-S3 (CH340, 1a86)"; exit 1; }
echo "Puerto de la bandeja: $PUERTO"
~/venv/bin/python -m esptool --chip esp32s3 --port "$PUERTO" --baud 460800 write-flash -z 0x10000 /tmp/bandeja.bin
cd ~/pi_control
cp -f seguidor.py seguidor.py.antes_seno 2>/dev/null || true
cp /tmp/seguidor_nuevo.py seguidor.py
setsid nohup sh ./arrancar.sh >/dev/null 2>&1 < /dev/null &
sleep 8
grep -a "\[pi\] ESP32 conectado\|\[myrio\]" salida.log | tail -2
EOS
scp -q "$BIN" "$PI:/tmp/bandeja.bin"
scp -q "$BASE/pi_control/seguidor.py" "$PI:/tmp/seguidor_nuevo.py"
scp -q "$REMOTO" "$PI:/tmp/grabar_bandeja.sh"; rm -f "$REMOTO"
ssh "$PI" bash /tmp/grabar_bandeja.sh
echo "Listo. Seno: http://${PI#*@}:8080/seno?eje=0&amp=100&barrido=1   (detener: /seno?amp=0)"
