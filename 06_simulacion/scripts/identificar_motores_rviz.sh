#!/usr/bin/env bash
# Vista y controles de identificación, sin cargar firmware automáticamente.
set -euo pipefail
RAIZ="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$RAIZ/ver_dron_rviz.sh" --diagnostico-motores "$@"
