#!/usr/bin/env bash
set -Eeuo pipefail

BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$BASE_DIR/herramientas:${PYTHONPATH:-}"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
else
  echo "ERROR: No se encontro ROS 2 Jazzy en /opt/ros/jazzy/setup.bash"
  exit 1
fi

echo "=========================================================="
echo " DSM: Ball & Plate en ROS 2 (Jazzy) + RViz"
echo " Vision y Control PD ejecutandose 100% en el PC"
echo " Transmision USB a 2,000,000 baud (19 FPS)"
echo " Ganancias: Kp=0.55 | Kd=0.80 | Ki=0.50 | Limite: +-10 deg"
echo "=========================================================="

exec python3 "$BASE_DIR/herramientas/puente_ball_plate_ros2.py" --rviz "$@"
