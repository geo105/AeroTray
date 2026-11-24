#!/usr/bin/env bash
set -euo pipefail
base=/home/george/DSM
export UV_CACHE_DIR="$base/herramientas/uv-cache"
export UV_HTTP_TIMEOUT=180
export UV_CONCURRENT_DOWNLOADS=4
uv="$base/herramientas/uv-env/bin/uv"
mkdir -p "$base/instalacion"
exec > >(tee -a "$base/instalacion/isaaclab.log") 2>&1
echo "Preparación Isaac Lab: $(date --iso-8601=seconds)"
cd "$base/herramientas/IsaacLab"
echo "Revisión de código: $(git rev-parse HEAD)"
"$uv" sync --frozen --no-dev
echo 'Entorno instalado. Se puede ejecutar la prueba oficial pequeña.'
