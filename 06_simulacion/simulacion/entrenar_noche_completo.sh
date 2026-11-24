#!/usr/bin/env bash
# Entrenamiento continuo multi-etapa (Curriculo Progresivo en GPU) para dejar corriendo
# de 30 minutos a 4+ horas: Dron en rampa (subida/bajada), diferentes angulos e inclinaciones,
# perturbaciones del Ball & Plate y exportacion automatica a C++ (politica_ppo_sim.h).
set -euo pipefail

PROJECT=/home/george/DSM
LOG_DIR="$PROJECT/simulacion/logs_noche"
mkdir -p "$LOG_DIR"

ITER_POR_ETAPA="${1:-2500}"
NUM_ENVS="${2:-256}"

echo "=================================================================="
echo " ENTRENAMIENTO PROGRESIVO EN GPU (AeroTray: Dron + Rampa + Bandeja)"
echo " Entornos paralelos: $NUM_ENVS | Iteraciones por etapa: $ITER_POR_ETAPA"
echo " Logs en: $LOG_DIR"
echo "=================================================================="

# 1. Optimizar controlador acoplado de Ball & Plate (2048 bandejas en GPU)
echo "[Etapa 0] Optimizando Ball & Plate ante rampa e inclinaciones del dron..."
"$PROJECT/herramientas/IsaacLab/.venv/bin/python" "$PROJECT/simulacion/entrenar_ball_plate_gpu.py" \
  | tee "$LOG_DIR/etapa0_ball_plate.log"

# Si el usuario paso el total de iteraciones (ej. 10000 o 15000), repartir entre las 4 etapas
if [ "$ITER_POR_ETAPA" -gt 4000 ]; then
  ITER_ETAPA=$(( ITER_POR_ETAPA / 4 ))
else
  ITER_ETAPA="$ITER_POR_ETAPA"
fi

# 2. Entrenamiento progresivo en Isaac Lab + Newton GPU por etapas (mostrando progreso en vivo)
ULTIMO_PT=""
for ETAPA in 1 2 3 4; do
  RUN_NAME="curriculo_etapa_${ETAPA}"
  echo ""
  echo "=================================================================="
  echo " [Etapa $ETAPA/4] Entrenando $NUM_ENVS drones en paralelo ($ITER_ETAPA iteraciones)..."
  echo "=================================================================="
  EXTRA_ARGS=()
  if [ -n "$ULTIMO_PT" ]; then
    EXTRA_ARGS=(--checkpoint "$ULTIMO_PT" --no-resume_optimizer_state)
  fi
  DSM_ETAPA_CURRICULO="$ETAPA" PYTHONUNBUFFERED=1 bash "$PROJECT/simulacion/entrenar_isaaclab.sh" \
    --num_envs "$NUM_ENVS" \
    --max_iterations "$ITER_ETAPA" \
    --run_name "$RUN_NAME" \
    "${EXTRA_ARGS[@]}" \
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

echo "=================================================================="
echo " ENTRENAMIENTO COMPLETADO. Resultados guardados en: $LOG_DIR"
echo " Firmware C++ actualizado en: $PROJECT/firmware/dron_octo_esp32/politica_ppo_sim.h"
echo "=================================================================="
