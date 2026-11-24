#!/bin/sh
# Arranca seguidor.py y lo vuelve a arrancar si se cierra. Lo lanza el crontab al encender la Pi:
#   @reboot /home/george/pi_control/arrancar.sh
# flock evita tener dos copias a la vez (pelearian por la camara y el puerto del ESP32).
# La salida queda en salida.log (ver con: tail -f ~/pi_control/salida.log).
cd "$(dirname "$0")" || exit 1
exec flock -n /tmp/seguidor.lock sh -c '
  while true; do
    echo "[arrancar] $(date) inicio" >> salida.log
    "$HOME/venv/bin/python" seguidor.py >> salida.log 2>&1
    echo "[arrancar] $(date) se cerro (codigo $?), reinicio en 3 s" >> salida.log
    sleep 3
  done
'
