#!/usr/bin/env bash
set -Eeuo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$BASE_DIR/instalacion"
exec > >(tee -a "$BASE_DIR/instalacion/gazebo.log") 2>&1
STATUS="$BASE_DIR/instalacion/gazebo.estado"
printf 'esperando_ros2\n' > "$STATUS"
trap 'rc=$?; if (( rc != 0 )); then printf "error (codigo %s)\n" "$rc" > "$STATUS"; echo "La instalacion se detuvo. Consulta instalacion/gazebo.log."; fi' EXIT
echo 'Gazebo Harmonic: esperando a que termine la instalacion de ROS 2.'
while true; do
    estado="$(cat "$BASE_DIR/instalacion/estado" 2>/dev/null || true)"
    case "$estado" in
        completo) break ;;
        en_curso) sleep 5 ;;
        *) echo "ROS 2 no esta listo (estado: $estado)."; exit 1 ;;
    esac
done
printf 'en_curso\n' > "$STATUS"
echo 'Instalando Gazebo Harmonic y el puente ROS 2. Introduce tu contrasena si se solicita.'
sudo -v
sudo apt-get install -y ros-jazzy-ros-gz
set +u
source /opt/ros/jazzy/setup.bash
set -u
gz sim --versions
ros2 pkg prefix ros_gz_bridge
python3 "$BASE_DIR/verificar_gazebo.py"
printf 'completo\n' > "$STATUS"
echo 'Gazebo y su comunicacion con ROS 2 estan instalados y comprobados.'
echo "Para abrir el ejemplo: gz sim $BASE_DIR/prueba.sdf"
