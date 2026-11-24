#!/usr/bin/env bash
set -eo pipefail
task_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION="dsm_octo_actual_${USER}"
rviz2 -d "$task_dir/octo.rviz" >"$task_dir/resultados/rviz.log" 2>&1 &
rviz_pid=$!
trap 'kill "$rviz_pid" 2>/dev/null || true' EXIT
python3 "$task_dir/ver_rviz.py" "$@"
