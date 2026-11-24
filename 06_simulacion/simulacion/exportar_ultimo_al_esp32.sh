#!/usr/bin/env bash
# Toma el ultimo modelo guardado en cualquier etapa de Isaac Lab, lo exporta a C++
# (politica_ppo_sim.h) y compila el firmware del ESP32 (100% offline, sin internet).
set -euo pipefail

PROJECT=/home/george/DSM
ULTIMO_DIR=$(ls -dt "$PROJECT/herramientas/IsaacLab/logs/rsl_rl/aerotray_cad/"* | head -n 1)
ULTIMO_PT=$(ls -v "$ULTIMO_DIR"/model_*.pt | tail -n 1)

echo "==> Ultimo checkpoint detectado: $ULTIMO_PT"
"$PROJECT/herramientas/IsaacLab/.venv/bin/python" "$PROJECT/simulacion/exportar_politica_ppo_cpp.py" \
  --checkpoint "$ULTIMO_PT" \
  --salida "$PROJECT/firmware/dron_octo_esp32/politica_ppo_sim.h"

bash "$PROJECT/firmware/compilar_dron_octo.sh"
echo "==> Listo para cargar al ESP32 con: bash /home/george/DSM/firmware/cargar_dron_octo.sh /dev/ttyUSB0"
