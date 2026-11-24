#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$task_dir:${PYTHONPATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$task_dir:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_PARTITION="dsm_octo_actual_${USER}"
if [[ "${1:-}" == --probar ]]; then
  shift
  if [[ ! -f "$task_dir/banco.sdf" ]]; then python3 "$task_dir/preparar.py"; fi
  exec python3 "$task_dir/probar_gazebo.py" "$@"
fi
python3 "$task_dir/preparar.py"
export DSM_SIM_MODO="${1:-banco}"
case "$DSM_SIM_MODO" in banco|altura|altura_ideal) ;; *) echo 'Uso: gazebo.sh [banco|altura|altura_ideal] o --probar [opciones]'; exit 2 ;; esac
exec gz sim -r --gui-config "$task_dir/gui.config" "$task_dir/$DSM_SIM_MODO.sdf"
