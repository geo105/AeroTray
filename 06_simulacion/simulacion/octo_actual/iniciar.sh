#!/usr/bin/env bash
# Ambos procesos usan exclusivamente archivos y tópicos de simulación.
set -eo pipefail
task_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-banco}"
if [[ "$mode" == real ]]; then
  exec bash "$task_dir/rviz.sh" --fuente real
fi
case "$mode" in banco|altura|altura_ideal) ;; *) echo 'Uso: iniciar.sh banco|altura|altura_ideal|real'; exit 2 ;; esac
if [[ "${2:-}" == candidato ]]; then
  export DSM_SIM_BP_GAINS="$(python3 -c 'import json,sys; print(json.dumps(json.load(open(sys.argv[1]))["perfil_simulacion"]))' "$task_dir/resultados/ajuste_ball_plate.json")"
fi
bash "$task_dir/gazebo.sh" "$mode" >"$task_dir/resultados/gazebo_visor.log" 2>&1 &
gazebo_pid=$!
trap 'kill "$gazebo_pid" 2>/dev/null || true' EXIT
bash "$task_dir/rviz.sh" --fuente gazebo
