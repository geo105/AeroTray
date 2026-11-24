#!/usr/bin/env bash
set -euo pipefail
project=/home/george/DSM
export UV_CACHE_DIR="$project/herramientas/uv-cache"
export XDG_CACHE_HOME="$project/simulacion/cache"
export MPLCONFIGDIR="$project/simulacion/cache/matplotlib"
export PYTHONUNBUFFERED=1
cd "$project/herramientas/IsaacLab"
echo 'Isaac Lab + Newton: 16 drones con cambios de altura y perturbaciones. Visor: http://127.0.0.1:8080'
exec "$project/herramientas/uv-env/bin/uv" run --frozen --no-sync python \
  "$project/simulacion/ejecutar_isaaclab.py" play \
  --rl_library rsl_rl --task DSM-AeroTray-Direct-v0 --num_envs 16 \
  --checkpoint latest --viz viser --real-time physics=newton_mjwarp "$@"
