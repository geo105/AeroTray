#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$task_root"
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/ensayo_hover/politica_test.cpp -o /tmp/politica_ensayo_hover
/tmp/politica_ensayo_hover
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/ensayo_hover/aceleracion_test.cpp -o /tmp/aceleracion_ensayo_hover
/tmp/aceleracion_ensayo_hover
g++ -std=c++17 -O2 -Wall -Wextra -Werror -I tests/ensayo_hover/stubs tests/ensayo_hover/firmware_test.cpp -o /tmp/firmware_ensayo_hover
/tmp/firmware_ensayo_hover
python3 -m unittest discover -s tests/ensayo_hover -v
python3 tests/ensayo_hover/integracion_serie.py
python3 -m py_compile herramientas/ensayo_hover/*.py
bash -n probar_hover_1600.sh cargar_firmware_hover_1600.sh
"$task_root/herramientas/arduino-cli/arduino-cli" --config-file "$task_root/herramientas/arduino-cli.yaml" compile --fqbn esp32:esp32:nodemcu-32s --build-path "$task_root/tmp/build_ensayo_hover_1600" "$task_root/firmware/ensayo_hover_1600"
