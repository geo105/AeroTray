#!/usr/bin/env bash
# Sube a la Pi la comunicacion por UART con el myRIO y reinicia el programa.
# Los pasos remotos van en un archivo: si fueran en la linea de ssh, pkill mataria su propia sesion.
set -Eeuo pipefail
PI=${PI:-pi@raspberrypi.local}
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REMOTO=$(mktemp)
cat > "$REMOTO" <<'EOF'
set -e
cd ~/pi_control
for f in comunicacion.py seguidor.py config_pi.json; do cp -n "$f" "$f.antes_uart" 2>/dev/null || true; done
cp /tmp/uart_nuevo/* .
pkill -f "seguidor.py" || true
sleep 2
setsid nohup sh ./arrancar.sh >/dev/null 2>&1 < /dev/null &
sleep 8
grep -a "\[myrio\]" salida.log | tail -2
EOF
ssh "$PI" 'rm -rf /tmp/uart_nuevo && mkdir -p /tmp/uart_nuevo'
scp -q "$DIR/myrio_uart.py" "$DIR/comunicacion.py" "$DIR/seguidor.py" "$DIR/config_pi.json" "$PI:/tmp/uart_nuevo/"
scp -q "$REMOTO" "$PI:/tmp/subir_uart.sh"; rm -f "$REMOTO"
ssh "$PI" bash /tmp/subir_uart.sh
