#!/usr/bin/env bash
# Carga solamente. No calibra, arma ni arranca motores.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_port="${1:-/dev/ttyUSB0}"
task_build="$task_root/tmp/build_ensayo_hover_1600"
task_cli="$task_root/herramientas/arduino-cli/arduino-cli"
task_cfg="$task_root/herramientas/arduino-cli.yaml"
"$task_cli" --config-file "$task_cfg" compile --fqbn esp32:esp32:nodemcu-32s --build-path "$task_build" "$task_root/firmware/ensayo_hover_1600"
# esptool 5 verifica automaticamente los datos escritos; no admite --verify.
"$task_cli" --config-file "$task_cfg" upload --fqbn esp32:esp32:nodemcu-32s:UploadSpeed=115200 --port "$task_port" --input-dir "$task_build" --upload-property 'upload.flags=--no-stub'
printf '\nFirmware cargado. Arranca DESARMADO. Abrir probar_hover_1600.sh para revisar.\n'
