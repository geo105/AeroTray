#!/usr/bin/env bash
# Ubuntu 24.04: ROS 2 Jazzy Desktop y herramientas de desarrollo.
# Ejecutar como usuario normal: bash /home/george/DSM/instalar_ros2.sh
set -Eeuo pipefail
umask 022
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$BASE_DIR/instalacion"
LOG="$BASE_DIR/instalacion/ros2.log"
STATUS="$BASE_DIR/instalacion/estado"
exec > >(tee -a "$LOG") 2>&1
printf 'en_curso\n' > "$STATUS"
trap 'rc=$?; if (( rc != 0 )); then printf "error (codigo %s)\n" "$rc" > "$STATUS"; printf "\nLa instalacion se detuvo. Registro: %s\n" "$LOG"; fi' EXIT

source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 || "$(uname -m)" != x86_64 ]]; then
    echo 'Este instalador es para Ubuntu 24.04 x86_64.'
    exit 1
fi
if (( EUID == 0 )); then
    echo 'Ejecuta este archivo como tu usuario normal, sin anteponer sudo.'
    exit 1
fi
export LANG=C.UTF-8 LC_ALL=C.UTF-8
printf '\nInstalando ROS 2 Jazzy. La contrasena se introduce solo en esta terminal.\n'
sudo -v
sudo apt-get update
sudo apt-get install -y ca-certificates curl git build-essential cmake python3-venv

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -qx 'install ok installed'; then
    # Descarga la configuracion oficial del repositorio ROS y verifica su digest.
    python3 - "$BASE_DIR/instalacion/ros2-apt-source.deb" <<'PY'
import hashlib
import json
import pathlib
import urllib.request
import sys

def download(url):
    req = urllib.request.Request(url, headers={'User-Agent': 'ROS2-Ubuntu-setup'})
    with urllib.request.urlopen(req, timeout=120) as response:
        return response.read()

release = json.loads(download('https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest'))
assets = [a for a in release['assets'] if a['name'].startswith('ros2-apt-source_')
          and a['name'].endswith('.noble_all.deb')]
if len(assets) != 1:
    raise SystemExit('No se encontro un unico paquete oficial para Ubuntu Noble.')
asset = assets[0]
url = asset['browser_download_url']
if not url.startswith('https://github.com/ros-infrastructure/ros-apt-source/releases/download/'):
    raise SystemExit('URL del paquete inesperada.')
data = download(url)
digest = asset.get('digest')
if digest:
    if digest != 'sha256:' + hashlib.sha256(data).hexdigest():
        raise SystemExit('La verificacion SHA256 del paquete fallo.')
else:
    raise SystemExit('La API no publica SHA256 para este paquete. Revisar antes de continuar.')
pathlib.Path(sys.argv[1]).write_bytes(data)
print('Paquete oficial verificado:', asset['name'])
PY
    sudo dpkg -i "$BASE_DIR/instalacion/ros2-apt-source.deb"
fi

sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop ros-dev-tools
if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update --rosdistro jazzy

if ! grep -Fqx '# ROS 2 Jazzy (DSM)' "$HOME/.bashrc"; then
    cp -p "$HOME/.bashrc" "$BASE_DIR/instalacion/bashrc.antes-de-ros2.$(date +%Y%m%d-%H%M%S)"
    cat >> "$HOME/.bashrc" <<'BASHRC'

# ROS 2 Jazzy (DSM)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi
BASHRC
fi

# Los scripts de entorno de ROS pueden consultar variables no definidas.
set +u
source /opt/ros/jazzy/setup.bash
set -u
printf '\nComprobando ROS 2 y la comunicacion local...\n'
ros2 pkg prefix rviz2
python3 "$BASE_DIR/verificar_ros2.py"
printf 'completo\n' > "$STATUS"
printf '\nROS 2 Jazzy instalado y comprobado. Abre una nueva terminal para usar ros2 y rviz2.\n'
printf 'Registro: %s\n' "$LOG"
