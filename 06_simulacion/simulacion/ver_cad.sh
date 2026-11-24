#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export XDG_CACHE_HOME="$task_dir/cache"
export MPLCONFIGDIR="$task_dir/cache/matplotlib"
exec "$task_dir/../herramientas/IsaacLab/.venv/bin/python" "$task_dir/ver_cad.py" "$@"
