#!/usr/bin/env bash
set -euo pipefail
base=/home/george/DSM
export UV_CACHE_DIR="$base/herramientas/uv-cache"
export XDG_CACHE_HOME="$base/simulacion/cache"
export PYTHONUNBUFFERED=1
mkdir -p "$base/instalacion"
exec > >(tee -a "$base/instalacion/isaaclab_prueba.log") 2>&1
cd "$base/herramientas/IsaacLab"
echo 'Prueba de herramientas: Cartpole, 16 entornos, 5 iteraciones, sin ventana.'
echo 'Este ejemplo NO entrena el dron.'
"$base/herramientas/uv-env/bin/uv" run --frozen --no-sync isaaclab train \
    --rl_library rsl_rl --task Isaac-Cartpole-Direct --num_envs 16 \
    --max_iterations 5 --viz none --logger tensorboard physics=newton_mjwarp
