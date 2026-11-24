#!/usr/bin/env bash
# RViz + graficas + puente del dron octo con el firmware captura_octo_pid_m8_suave (con o sin laser).
# El puente SOLO ESCUCHA; las ordenes del dron se escriben en esta terminal.
#   bash ~/DSM/ver_octo_m8_rviz.sh                          # dron real por USB
#   bash ~/DSM/ver_octo_m8_rviz.sh --puerto /dev/ttyUSB0
#   bash ~/DSM/ver_octo_m8_rviz.sh --repetir ARCHIVO.txt    # reproducir un ensayo grabado
#   bash ~/DSM/ver_octo_m8_rviz.sh --demo                   # reproducir el ensayo de 49 s del 10/11
set -eo pipefail
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
args=("$@")
if [[ "${1:-}" == --demo ]]; then
  args=(--repetir "$RAIZ/octo_entrega_20261110/Octo_DSM/Octo_DSM_Monitor_Serie_Windows/registros/Octo_20261110_165557_745.txt" --bucle "${@:2}")
fi
rviz2 -d "$RAIZ/simulacion/rviz/octo_m8.rviz" >/tmp/dsm_rviz_octo_m8.log 2>&1 &
PIDS="$!"
if ros2 pkg prefix rqt_plot >/dev/null 2>&1; then
  ros2 run rqt_plot rqt_plot /dron/actitud/vector/x /dron/actitud/vector/y /dron/accel_g/data /dron/altura_cm/data >/tmp/dsm_rqt_octo_m8.log 2>&1 &
  PIDS="$PIDS $!"
fi
trap 'kill $PIDS 2>/dev/null || true' EXIT
python3 "$RAIZ/herramientas/puente_octo_m8_ros2.py" "${args[@]}"
