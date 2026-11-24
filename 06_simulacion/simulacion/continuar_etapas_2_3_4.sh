#!/usr/bin/env bash
# Continua las Etapas 2, 3 y 4 que faltaron anoche, retomando el mejor modelo de la Etapa 1
# y mostrando el progreso en vivo tanto en la terminal como en TensorBoard (http://127.0.0.1:6006).
set -euo pipefail

PROJECT=/home/george/DSM
LOG_DIR="$PROJECT/simulacion/logs_noche"
mkdir -p "$LOG_DIR"

ITER_POR_ETAPA="${1:-1000}"
NUM_ENVS="${2:-256}"

# Buscar la corrida de Etapa 1 de anoche y usar su mejor checkpoint (model_500.pt)
ETAPA1_DIR=$(ls -dt "$PROJECT/herramientas/IsaacLab/logs/rsl_rl/aerotray_cad/"*curriculo_etapa_1* | head -n 1)
ULTIMO_PT="$ETAPA1_DIR/model_500.pt"

echo "=================================================================="
echo " CONTINUANDO ETAPAS 2, 3 y 4 EN GPU"
echo " Retomando desde: $ULTIMO_PT"
echo " Iteraciones por etapa: $ITER_POR_ETAPA"
echo "=================================================================="

for ETAPA in 2 3 4; do
  RUN_NAME="curriculo_etapa_${ETAPA}"
  echo ""
  echo "=================================================================="
  echo " [Etapa $ETAPA/4] Entrenando $NUM_ENVS drones ($ITER_POR_ETAPA iteraciones)..."
  echo "=================================================================="

  DSM_ETAPA_CURRICULO="$ETAPA" PYTHONUNBUFFERED=1 bash "$PROJECT/simulacion/entrenar_isaaclab.sh" \
    --num_envs "$NUM_ENVS" \
    --max_iterations "$ITER_POR_ETAPA" \
    --run_name "$RUN_NAME" \
    --checkpoint "$ULTIMO_PT" \
    --no-resume_optimizer_state \
    2>&1 | tee "$LOG_DIR/${RUN_NAME}.log"

  ULTIMO_DIR=$(ls -dt "$PROJECT/herramientas/IsaacLab/logs/rsl_rl/aerotray_cad/"*"${RUN_NAME}"* | head -n 1)
  ULTIMO_PT=$(ls -v "$ULTIMO_DIR"/model_*.pt | tail -n 1)
  echo "  -> Etapa $ETAPA completada: $ULTIMO_PT"

  # Evaluar automaticamente y actualizar politica_ppo_sim.h para el ESP32
  PYTHONPATH="$PROJECT/simulacion" UV_CACHE_DIR="$PROJECT/herramientas/uv-cache" XDG_CACHE_HOME="$PROJECT/simulacion/cache" \
    "$PROJECT/herramientas/IsaacLab/.venv/bin/python" \
    "$PROJECT/simulacion/isaaclab_aerotray/evaluar.py" --modo policy --num_envs 64 \
    --checkpoint "$ULTIMO_PT" --salida "$LOG_DIR/evaluacion_etapa_${ETAPA}.json" --visualizer none \
    2>&1 | tee -a "$LOG_DIR/${RUN_NAME}.log"

  "$PROJECT/herramientas/IsaacLab/.venv/bin/python" "$PROJECT/simulacion/exportar_politica_ppo_cpp.py" \
    --checkpoint "$ULTIMO_PT" \
    --evaluacion "$LOG_DIR/evaluacion_etapa_${ETAPA}.json" \
    --salida "$PROJECT/firmware/dron_octo_esp32/politica_ppo_sim.h" \
    2>&1 | tee -a "$LOG_DIR/${RUN_NAME}.log"
done

bash "$PROJECT/firmware/compilar_dron_octo.sh"
echo "=================================================================="
echo " ETAPAS 2, 3 y 4 COMPLETADAS Y EXPORTADAS AL ESP32"
echo "=================================================================="
