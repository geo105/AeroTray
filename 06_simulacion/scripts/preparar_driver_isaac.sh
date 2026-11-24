#!/usr/bin/env bash
set -euo pipefail

# Usa exclusivamente el paquete disponible en los repositorios de Ubuntu.
# Ejecutar como george; sudo pide la contraseña en la terminal de Ubuntu.
if (( EUID == 0 )); then
    echo 'Ejecuta este script como tu usuario normal, sin sudo delante.' >&2
    exit 1
fi

mkdir -p /home/george/DSM/instalacion
exec > >(tee -a /home/george/DSM/instalacion/driver_isaac.log) 2>&1
echo "Inicio: $(date --iso-8601=seconds)"
echo 'Se actualizará el controlador NVIDIA mediante apt. No se reinicia automáticamente.'
echo 'Guarda tu trabajo antes de reiniciar al finalizar.'
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
sudo -v
sudo apt-get update
sudo apt-get install "linux-headers-$(uname -r)" nvidia-driver-595-open
echo 'Instalación terminada. Guarda tu trabajo y reinicia desde el menú de Ubuntu.'
echo 'Después del reinicio, ejecuta nvidia-smi para verificar el controlador cargado.'
