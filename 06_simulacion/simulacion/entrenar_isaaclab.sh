#!/usr/bin/env bash
set -euo pipefail
project=/home/george/DSM
export UV_CACHE_DIR="$project/herramientas/uv-cache"
export XDG_CACHE_HOME="$project/simulacion/cache"
export MPLCONFIGDIR="$project/simulacion/cache/matplotlib"
export PYTHONUNBUFFERED=1
cd "$project/herramientas/IsaacLab"
exec "$project/herramientas/uv-env/bin/uv" run --frozen --no-sync python \
  "$project/simulacion/ejecutar_isaaclab.py" train \
  --rl_library rsl_rl --task DSM-AeroTray-Direct-v0 --num_envs 64 \
  --max_iterations 1000 --viz none --logger tensorboard \
  --run_name entrenamiento_manual physics=newton_mjwarp "$@"
