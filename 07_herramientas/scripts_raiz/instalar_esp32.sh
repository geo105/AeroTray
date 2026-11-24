#!/usr/bin/env bash
set -Eeuo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$BASE_DIR/instalacion" "$BASE_DIR/herramientas"
exec > >(tee -a "$BASE_DIR/instalacion/esp32.log") 2>&1
STATUS="$BASE_DIR/instalacion/esp32.estado"
printf 'descargando\n' > "$STATUS"
trap 'rc=$?; if (( rc != 0 )); then printf "error (codigo %s)\n" "$rc" > "$STATUS"; echo "ESP-IDF se detuvo. Consulta instalacion/esp32.log."; fi' EXIT
IDF_DIR="$BASE_DIR/herramientas/esp-idf"
if [[ ! -d "$IDF_DIR" ]]; then
    git clone --branch v5.5.6 --depth 1 --shallow-submodules --recursive https://github.com/espressif/esp-idf.git "$IDF_DIR"
fi
if [[ "$(git -C "$IDF_DIR" describe --tags --exact-match)" != v5.5.6 ]]; then
    echo 'La carpeta ESP-IDF existente no corresponde a v5.5.6.'
    exit 1
fi
git -C "$IDF_DIR" submodule update --init --recursive --depth 1
echo 'Esperando a que termine la instalacion de Gazebo antes de instalar dependencias.'
printf 'esperando_gazebo\n' > "$STATUS"
while true; do
    estado="$(cat "$BASE_DIR/instalacion/gazebo.estado" 2>/dev/null || true)"
    case "$estado" in
        completo) break ;;
        en_curso|esperando_ros2) sleep 5 ;;
        *) echo "Gazebo no esta listo (estado: $estado)."; exit 1 ;;
    esac
done
printf 'en_curso\n' > "$STATUS"
sudo -v
sudo apt-get install -y wget flex bison gperf python3-pip python3-venv ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
export IDF_TOOLS_PATH="$BASE_DIR/herramientas/esp-idf-tools"
bash "$IDF_DIR/install.sh" esp32
set +u
source "$IDF_DIR/export.sh"
set -u
idf.py --version
EXAMPLE="$BASE_DIR/esp32_ejemplo"
if [[ ! -d "$EXAMPLE" ]]; then
    cp -a "$IDF_DIR/examples/get-started/hello_world" "$EXAMPLE"
fi
cd "$EXAMPLE"
idf.py set-target esp32
idf.py build
printf 'completo\n' > "$STATUS"
echo 'ESP-IDF instalado. El ejemplo hello_world compila para ESP32. No se ha grabado la placa.'
echo "Para activar las herramientas: source $BASE_DIR/activar_esp32.sh"
