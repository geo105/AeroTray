#!/usr/bin/env bash
# Vista CAD local sin comunicación con el robot.
set -eo pipefail
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${DSM_CAD_ROS_DOMAIN_ID:-77}"
export ROS_LOG_DIR="$RAIZ/registros/rviz_cad_ros"
mkdir -p "$ROS_LOG_DIR"
if [[ ! -f "$RAIZ/simulacion/rviz/conjunto_completo.urdf" ]]; then
  python3 "$RAIZ/simulacion/rviz/generar_conjunto_cad.py"
fi
echo 'RViz: conjunto CAD completo, 383 piezas, pose fija. Ctrl+C para cerrar.'
echo "Dominio ROS $ROS_DOMAIN_ID: visor independiente, sin comandos al hardware."
exec ros2 launch "$RAIZ/simulacion/rviz/conjunto_cad.launch.py"
