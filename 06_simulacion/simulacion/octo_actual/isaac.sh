#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
lab_dir=/home/george/DSM/herramientas/IsaacLab
export UV_CACHE_DIR=/home/george/DSM/herramientas/uv-cache
export XDG_CACHE_HOME="$task_dir/cache"
export MPLCONFIGDIR="$task_dir/cache/matplotlib"
export PYTHONUNBUFFERED=1
cd "$lab_dir"
command="${1:-train}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$command" == preparar ]]; then
  exec /home/george/DSM/herramientas/uv-env/bin/uv run --frozen --no-sync python "$task_dir/preparar.py" --isaac
fi
if [[ "$command" == evaluar ]]; then
  exec /home/george/DSM/herramientas/uv-env/bin/uv run --frozen --no-sync python "$task_dir/evaluar_isaac.py" "$@"
fi
case "$command" in train) extra=(--max_iterations 100 --logger tensorboard --run_name controles_probados) ;;
  play) extra=(--checkpoint "$task_dir/modelos/residual_100_iteraciones.pt" --viz viser --real-time) ;;
  *) echo 'Uso: isaac.sh preparar|train|play|evaluar [opciones]'; exit 2 ;;
esac
exec /home/george/DSM/herramientas/uv-env/bin/uv run --frozen --no-sync python "$task_dir/isaac_cli.py" "$command" \
  --rl_library rsl_rl --task DSM-Octo-Actual-Residual-v0 --num_envs 8 --viz none \
  "${extra[@]}" physics=newton_mjwarp "$@"
