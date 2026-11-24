#!/usr/bin/env bash
# Muestra en vivo el progreso del entrenamiento nocturno en GPU (AeroTray + Ball & Plate)
set -euo pipefail

LOG_DIR="/home/george/DSM/simulacion/logs_noche"

echo "=================================================================="
echo " MONITOR EN VIVO DEL ENTRENAMIENTO GPU"
echo " Directorio de registros: $LOG_DIR"
echo " Presiona Ctrl+C en cualquier momento para salir del monitor"
echo " (El entrenamiento seguira corriendo en segundo plano sin detenerse)"
echo "=================================================================="

if [ -f "$LOG_DIR/etapa0_ball_plate.log" ]; then
  echo "--- Resultado Etapa 0 (Ball & Plate + Rampa + Dron) ---"
  cat "$LOG_DIR/etapa0_ball_plate.log"
  echo "-------------------------------------------------------"
fi

ULTIMO_LOG=$(ls -t "$LOG_DIR"/curriculo_etapa_*.log 2>/dev/null | head -n 1 || true)
if [ -z "$ULTIMO_LOG" ]; then
  echo "Esperando a que inicie curriculo_etapa_1.log..."
  tail -f "$LOG_DIR"/*.log
else
  echo "Siguiendo en vivo: $ULTIMO_LOG"
  tail -n 25 -f "$LOG_DIR"/curriculo_etapa_*.log
fi
