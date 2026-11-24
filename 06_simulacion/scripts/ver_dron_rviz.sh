#!/usr/bin/env bash
# Abre RViz + graficas y el puente serie/WiFi del dron en ESTA terminal (aqui escribes los comandos).
#   bash ~/DSM/ver_dron_rviz.sh                # dron real por cable USB (autodetecta /dev/ttyUSB*)
#   bash ~/DSM/ver_dron_rviz.sh --wifi         # dron real SIN CABLE por WiFi AP 'DSM-Octo' (clave octo2026)
#   bash ~/DSM/ver_dron_rviz.sh --simular      # sin hardware, para probar que RViz funciona
#   bash ~/DSM/ver_dron_rviz.sh --puerto /dev/ttyUSB0
#   bash ~/DSM/ver_dron_rviz.sh --diagnostico-motores --puerto /dev/PUERTO_NODEMCU
#   bash ~/DSM/ver_dron_rviz.sh --diagnostico-motores --simular
set -eo pipefail
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Comprobar opciones en cualquier orden. Una demo nunca toca puertos/permisos.
usar_usb=1
puerto_elegido=""
opcion_anterior=""
mostrar_ayuda=0
for opcion in "$@"; do
  if [[ "$opcion_anterior" == --puerto ]]; then puerto_elegido="$opcion"; fi
  case "$opcion" in
    --wifi|--simular) usar_usb=0 ;;
    --help|-h) usar_usb=0; mostrar_ayuda=1 ;;
  esac
  opcion_anterior="$opcion"
done
if [[ "$usar_usb" == 1 ]]; then
  puertos=(/dev/ttyUSB* /dev/ttyACM*)
  if [[ -n "$puerto_elegido" ]]; then puertos=("$puerto_elegido"); fi
  for p in "${puertos[@]}"; do
    if [[ -e "$p" && ( ! -r "$p" || ! -w "$p" ) ]]; then
      echo "==> Habilitando permiso de acceso a $p..."
      bash "$RAIZ/habilitar_usb_esp32.sh"
      break
    fi
  done
fi

source /opt/ros/jazzy/setup.bash
if [[ "$mostrar_ayuda" == 1 ]]; then
  exec python3 "$RAIZ/herramientas/puente_dron_ros2.py" "$@"
fi

rviz2 -d "$RAIZ/simulacion/rviz/dron_octo.rviz" >/tmp/dsm_rviz.log 2>&1 &
PID_RVIZ=$!
PID_PLOT=""
if ros2 pkg prefix rqt_plot >/dev/null 2>&1; then
  ros2 run rqt_plot rqt_plot /dron/actitud/vector/x /dron/actitud/vector/y /dron/motores_us/data[0] /dron/motores_us/data[1] >/tmp/dsm_rqt_plot.log 2>&1 &
  PID_PLOT=$!
fi
trap 'kill $PID_RVIZ $PID_PLOT 2>/dev/null || true' EXIT

python3 "$RAIZ/herramientas/puente_dron_ros2.py" "$@"
