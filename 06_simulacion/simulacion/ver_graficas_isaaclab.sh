#!/usr/bin/env bash
set -euo pipefail
echo 'Gráficas del aprendizaje: http://127.0.0.1:6006'
exec /home/george/DSM/herramientas/IsaacLab/.venv/bin/tensorboard \
  --logdir /home/george/DSM/herramientas/IsaacLab/logs/rsl_rl/aerotray_cad \
  --host 127.0.0.1 --port 6006
