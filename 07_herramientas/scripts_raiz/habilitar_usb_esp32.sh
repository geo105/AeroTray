#!/usr/bin/env bash
set -Eeuo pipefail
BASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REGLA="$BASE_DIR/herramientas/69-dsm-esp32.rules"
DESTINO=/etc/udev/rules.d/69-dsm-esp32.rules

echo "==> Habilitando permisos USB para ESP32 (CP210x 10c4:ea60 y CH340 1a86:7523)..."
sudo -v
sudo install -o root -g root -m 0644 "$REGLA" "$DESTINO"
sudo udevadm control --reload-rules

for puerto in /dev/ttyUSB* /dev/ttyACM*; do
  [[ -e "$puerto" ]] || continue
  sudo udevadm trigger --action=change "/sys/class/tty/${puerto##*/}" || true
  sudo chmod a+rw "$puerto" || true
  echo "==> Puerto listo: $puerto"
  getfacl -p "$puerto" | sed -n '1,6p'
done
