#!/usr/bin/env bash
set -euo pipefail

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLI="$RAIZ/herramientas/arduino-cli/arduino-cli"
CFG="$RAIZ/herramientas/arduino-cli.yaml"
SKETCH="$RAIZ/firmware/dron_octo_esp32"
BIN="$RAIZ/firmware/bin/dron_octo_esp32"
FQBN="esp32:esp32:nodemcu-32s"

PUERTO="${1:-}"
if [[ -z "$PUERTO" ]]; then
  echo "Indica explicitamente el puerto del NodeMCU del dron; no usar el de la camara S3."
  echo "Uso: bash $0 /dev/PUERTO_DEL_NODEMCU"
  exit 1
fi

if [[ -z "$PUERTO" ]]; then
  echo "ERROR: No se detecto puerto USB (/dev/ttyUSB* ni /dev/ttyACM*)."
  echo "Conecta el ESP32 NodeMCU-32S del dron por USB y vuelve a ejecutar:"
  echo "  bash ~/DSM/firmware/cargar_dron_octo.sh"
  exit 1
fi

if [[ ! -r "$PUERTO" || ! -w "$PUERTO" ]]; then
  echo "==> Habilitando permiso de acceso a $PUERTO (pide tu clave de Ubuntu una sola vez)..."
  bash "$RAIZ/habilitar_usb_esp32.sh"
fi

echo "==> Comprobando que el ESP32 lee su flash..."
bash "$RAIZ/firmware/probar_flash_esp32.sh" "$PUERTO" || { echo "==> Carga cancelada: la flash no responde de forma estable."; exit 2; }

echo "==> Compilando candidato de banco actual, con motores bloqueados por defecto..."
"$RAIZ/firmware/compilar_dron_octo.sh"

echo "==> Subiendo dron_octo_esp32 a $PUERTO ($FQBN)..."
echo "    (Si se queda en 'Connecting...', manten presionado el boton BOOT del ESP32 por 2 segundos)"
# 115200 y sin stub: con este NodeMCU el stub y la velocidad alta fallaron (31-oct y 5-nov).
"$CLI" --config-file "$CFG" upload \
  --fqbn "$FQBN:UploadSpeed=115200" \
  --upload-property upload.flags=--no-stub \
  --port "$PUERTO" \
  --input-dir "$BIN" \
  "$SKETCH"

echo ""
echo "==> CARGA EXITOSA en $PUERTO."
echo "==> Ahora inicia RViz + Monitor Serie con:"
echo "    bash ~/DSM/ver_dron_rviz.sh"
