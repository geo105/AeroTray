#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" != "--consola" ]]; then
  exec gnome-terminal --title="Octocoptero: ! o ESPACIO = APAGAR" -- bash "$task_root/probar_hover_1600.sh" --consola "$@"
fi
shift
set +u
source /opt/ros/jazzy/setup.bash
set -u
rviz2 -d "$task_root/simulacion/rviz/octo_m8.rviz" > /tmp/rviz_ensayo_hover.log 2>&1 &
rviz_pid=$!
trap 'kill "$rviz_pid" 2>/dev/null || true' EXIT
python3 "$task_root/herramientas/ensayo_hover/consola.py" --rviz "$@"
printf '\nConsola cerrada. Pulsa Enter para cerrar esta ventana.\n'
read -r _
