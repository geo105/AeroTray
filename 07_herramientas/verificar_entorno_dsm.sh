#!/usr/bin/env bash
# Verificacion de SOLO LECTURA del entorno DSM:
#   Ubuntu, ROS 2 Jazzy, Gazebo, Python, USB/ESP32, GPU, Isaac Lab y pruebas del proyecto.
# No usa sudo, no abre puertos serie, no carga firmware y no mueve motores ni servos.
#
# Uso:   bash ~/DSM/herramientas/verificar_entorno_dsm.sh
# Deja el informe en ~/DSM/registros/verificacion_entorno_<fecha>.txt
set -uo pipefail

RAIZ="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FECHA="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RAIZ/registros"
LOG="$RAIZ/registros/verificacion_entorno_$FECHA.txt"
TMPD="$(mktemp -d /tmp/dsm_verif.XXXXXX)"
trap 'rm -rf "$TMPD"' EXIT
exec > >(tee "$LOG") 2>&1

N_OK=0; N_FALLA=0; N_AVISO=0
ok()     { echo "  [OK]    $*"; N_OK=$((N_OK+1)); }
falla()  { echo "  [FALLA] $*"; N_FALLA=$((N_FALLA+1)); }
aviso()  { echo "  [AVISO] $*"; N_AVISO=$((N_AVISO+1)); }
info()   { echo "          $*"; }
seccion(){ echo; echo "== $* =="; }
# Suma los contadores que imprime un bloque de Python ("##CUENTA ok falla aviso").
sumar()  { local l; l="$(grep '^##CUENTA' "$1" | tail -1)"; [[ -z "$l" ]] && return
           set -- $l; N_OK=$((N_OK+$2)); N_FALLA=$((N_FALLA+$3)); N_AVISO=$((N_AVISO+$4)); }

echo "Verificacion DSM  $(date '+%Y-%m-%d %H:%M:%S')  carpeta: $RAIZ"

# ---------------------------------------------------------------- Sistema
seccion "1. Sistema"
source /etc/os-release 2>/dev/null || true
if [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 ]]; then ok "Ubuntu $VERSION_ID (${PRETTY_NAME:-})"
else falla "Se esperaba Ubuntu 24.04; hay ${PRETTY_NAME:-desconocido}"; fi
[[ "$(uname -m)" == x86_64 ]] && ok "Arquitectura x86_64, kernel $(uname -r)" || falla "Arquitectura $(uname -m)"
info "RAM: $(free -h | awk '/^Mem:/{print $2" total, "$7" disponible"}')"
info "Disco en \$HOME: $(df -h "$HOME" | awk 'NR==2{print $4" libres de "$2" ("$5" usado)"}')"
libre_gb=$(df -BG "$HOME" | awk 'NR==2{gsub("G","",$4);print $4}')
(( libre_gb < 20 )) && aviso "Quedan menos de 20 GB libres; Isaac Lab y los registros crecen rapido"

# ---------------------------------------------------------------- ROS 2
seccion "2. ROS 2 Jazzy"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then ok "/opt/ros/jazzy instalado"
else falla "No existe /opt/ros/jazzy/setup.bash"; fi
n_bashrc=$(grep -c 'source /opt/ros/[a-z]*/setup.bash' "$HOME/.bashrc" 2>/dev/null || true)
otras=$(grep -o 'source /opt/ros/[a-z]*/setup.bash' "$HOME/.bashrc" 2>/dev/null | grep -v jazzy || true)
if grep -q 'source /opt/ros/jazzy/setup.bash' "$HOME/.bashrc" 2>/dev/null; then ok "~/.bashrc carga ROS 2 Jazzy en cada terminal"
else aviso "~/.bashrc no carga ROS; tendras que hacer 'source /opt/ros/jazzy/setup.bash' en cada terminal"; fi
[[ -n "$otras" ]] && falla "~/.bashrc tambien carga otra distro ROS: $otras (mezclar distros rompe ros2)"
(( n_bashrc > 2 )) && aviso "ROS se carga $n_bashrc veces en ~/.bashrc (duplicado, no grave)"

set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null; set -u
info "ROS_DISTRO=${ROS_DISTRO:-?}  RMW=${RMW_IMPLEMENTATION:-por defecto (FastDDS)}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
info "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-por defecto}  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-no definido}"
if command -v ros2 >/dev/null; then ok "Comando ros2 disponible"; else falla "Comando ros2 no encontrado"; fi
faltan=()
for p in rclpy rviz2 rviz_default_plugins tf2_ros visualization_msgs cv_bridge ros_gz_bridge ros_gz_sim \
         demo_nodes_cpp demo_nodes_py rqt_plot xacro robot_state_publisher; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 || faltan+=("$p")
done
if (( ${#faltan[@]} == 0 )); then ok "Paquetes ROS que usa el proyecto: todos presentes"
else falla "Faltan paquetes ROS: ${faltan[*]}"; fi
for herramienta in colcon rosdep; do
  command -v "$herramienta" >/dev/null && ok "$herramienta instalado" || aviso "$herramienta no instalado (sudo apt install ros-dev-tools)"
done
[[ -f "$HOME/.ros/rosdep/sources.cache" ]] && ok "rosdep inicializado y actualizado" || aviso "Falta 'rosdep update'"

echo "  ... probando talker (C++) -> listener (Python) por DDS (hasta 40 s)"
if timeout 60 python3 "$RAIZ/verificar_ros2.py" >"$TMPD/talker.txt" 2>&1; then ok "Comunicacion ROS 2 C++ <-> Python funciona"
else falla "talker/listener no se comunicaron:"; sed 's/^/          /' "$TMPD/talker.txt" | tail -8; fi

echo "  ... ros2 doctor (hasta 60 s)"
timeout 60 ros2 doctor >"$TMPD/doctor.txt" 2>&1
if grep -qi 'checks passed' "$TMPD/doctor.txt" && ! grep -qi 'fail' "$TMPD/doctor.txt"; then
  ok "ros2 doctor: $(grep -i 'checks passed' "$TMPD/doctor.txt" | tail -1 | sed 's/^ *//')"
else
  aviso "ros2 doctor reporto observaciones (los avisos de red/paquetes nuevos suelen ser inofensivos):"
  grep -iE 'warn|fail|error' "$TMPD/doctor.txt" | head -8 | sed 's/^/          /'
fi

# ---------------------------------------------------------------- Gazebo
seccion "3. Gazebo"
if command -v gz >/dev/null; then
  v="$(timeout 20 gz sim --versions 2>/dev/null | head -1)"
  [[ -n "$v" ]] && ok "Gazebo (gz sim) version $v" || aviso "gz existe pero 'gz sim --versions' no respondio"
else aviso "Gazebo (gz) no esta en el PATH; solo hace falta para simular en Gazebo"; fi

# ---------------------------------------------------------------- Python del sistema
seccion "4. Python del sistema (el que usan ROS, el panel y la vision)"
python3 - >"$TMPD/py.txt" 2>&1 <<'PY'
import importlib, os, sys
ok = falla = aviso = 0
def p(tag, msg):
    global ok, falla, aviso
    print(f"  [{tag}]{' '*(6-len(tag))}{msg}")
    if tag == "OK": ok += 1
    elif tag == "FALLA": falla += 1
    else: aviso += 1
v = sys.version_info
p("OK" if (v.major, v.minor) == (3, 12) else "FALLA", f"python3 {v.major}.{v.minor}.{v.micro} en {sys.executable}")
if os.environ.get("VIRTUAL_ENV") or os.environ.get("CONDA_PREFIX"):
    p("AVISO", "Hay un entorno virtual/conda activo; ROS debe usar el python3 del sistema")
home_local = os.path.expanduser("~/.local/lib")
requeridos = {
    "numpy": "vision y ROS", "cv2": "vision ArUco", "serial": "USB con ESP32 (pyserial)",
    "PIL": "imagenes", "psutil": "gestion de procesos", "tkinter": "panel ball and plate",
    "rclpy": "ROS 2 en Python", "cv_bridge": "imagen de camara en ROS", "pytest": "pruebas",
}
for mod, uso in requeridos.items():
    try:
        m = importlib.import_module(mod)
    except Exception as e:
        p("FALLA" if mod not in ("pytest",) else "AVISO", f"{mod}: no importa ({type(e).__name__}: {e}) -> {uso}")
        continue
    ver = getattr(m, "__version__", getattr(m, "VERSION", ""))
    ruta = getattr(m, "__file__", "") or ""
    extra = ""
    if mod == "cv2":
        extra = " con aruco" if hasattr(m, "aruco") else " SIN aruco"
        if not hasattr(m, "aruco"):
            p("FALLA", f"cv2 {ver} no trae aruco (instalar python3-opencv de apt, no opencv-python sin contrib)")
            continue
    p("OK", f"{mod} {ver}{extra}  ({ruta.split('/site-packages/')[0].split('/dist-packages/')[0] if ruta else 'integrado'})")
    if ruta.startswith(home_local) and mod in ("numpy", "cv2"):
        p("AVISO", f"{mod} viene de ~/.local (pip --user); puede romper cv_bridge/rclpy. Lo esperado es /usr/lib/python3/dist-packages")
    if mod == "numpy" and str(ver).split(".")[0] >= "2":
        p("FALLA", "numpy >= 2: cv_bridge de Jazzy esta compilado para numpy 1.x")
try:
    import numpy as np
    from cv_bridge import CvBridge
    CvBridge().cv2_to_imgmsg(np.zeros((4, 4, 3), np.uint8), "bgr8")
    p("OK", "cv_bridge convierte una imagen OpenCV a mensaje ROS")
except Exception as e:
    p("FALLA", f"cv_bridge no funciona con este numpy/OpenCV: {e}")
print(f"##CUENTA {ok} {falla} {aviso}")
PY
grep -v '^##CUENTA' "$TMPD/py.txt"; sumar "$TMPD/py.txt"

# ---------------------------------------------------------------- USB / ESP32
seccion "5. USB y placas ESP32 (sin abrir los puertos)"
if id -nG | tr ' ' '\n' | grep -qx dialout; then ok "Tu usuario esta en el grupo dialout"
else info "Tu usuario no esta en dialout (no es obligatorio si la regla udev 'uaccess' esta instalada)"; fi
if [[ -f /etc/udev/rules.d/69-dsm-esp32.rules ]]; then ok "Regla udev del proyecto instalada (/etc/udev/rules.d/69-dsm-esp32.rules)"
elif id -nG | tr ' ' '\n' | grep -qx dialout; then ok "Acceso serie garantizado por el grupo dialout"
else falla "Sin regla udev ni grupo dialout: ejecuta 'bash $RAIZ/habilitar_usb_esp32.sh'"; fi
if dpkg-query -W -f='${Status}' brltty 2>/dev/null | grep -q 'install ok installed'; then
  aviso "brltty esta instalado: en Ubuntu suele 'robar' los adaptadores CH340 (camara). Si la camara desaparece al conectarla: sudo apt remove brltty"
fi
shopt -s nullglob
puertos=(/dev/serial/by-id/*)
if (( ${#puertos[@]} == 0 )); then aviso "No hay placas USB conectadas ahora (es normal si estan desconectadas)"
else
  for p in "${puertos[@]}"; do
    real="$(readlink -f "$p")"; tipo="desconocido"
    [[ "$p" == *1a86* ]] && tipo="CH340 -> camara ESP32-S3 (ball and plate)"
    [[ "$p" == *Silicon_Labs* || "$p" == *CP210* ]] && tipo="CP2102 -> NodeMCU-32S del dron"
    if [[ -r "$real" && -w "$real" ]]; then ok "$real  [$tipo]  con permiso de lectura/escritura"
    else falla "$real  [$tipo]  SIN permiso: ejecuta 'bash $RAIZ/habilitar_usb_esp32.sh'"; fi
  done
fi
shopt -u nullglob

seccion "6. Herramientas de firmware"
ACLI="$RAIZ/herramientas/arduino-cli/arduino-cli"; ACFG="$RAIZ/herramientas/arduino-cli.yaml"
if [[ -x "$ACLI" ]]; then
  ok "arduino-cli $("$ACLI" version 2>/dev/null | grep -o 'Version: [^ ]*' | cut -d' ' -f2)"
  core="$("$ACLI" --config-file "$ACFG" core list 2>/dev/null | awk '/esp32:esp32/{print $2}')"
  [[ -n "$core" ]] && ok "Nucleo Arduino esp32:esp32 $core" || falla "Falta el nucleo esp32:esp32 en arduino-cli"
else falla "No existe $ACLI"; fi
[[ -f "$RAIZ/herramientas/esp-idf/export.sh" ]] && ok "ESP-IDF presente (source $RAIZ/activar_esp32.sh)" || aviso "ESP-IDF no encontrado (solo hace falta para idf.py)"

# ---------------------------------------------------------------- GPU / Isaac Lab
seccion "7. GPU e Isaac Lab"
if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
  ok "NVIDIA: $(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | head -1)"
else falla "nvidia-smi no responde: el driver NVIDIA no esta cargado (Isaac Lab necesita la GPU)"; fi
PYISAAC="$RAIZ/herramientas/IsaacLab/.venv/bin/python"
if [[ -x "$PYISAAC" ]]; then
  r="$(timeout 120 "$PYISAAC" -c 'import torch;print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else "-")' 2>&1 | tail -1)"
  if [[ "$r" == *" True "* ]]; then ok "Isaac Lab venv: torch $r"
  else falla "Isaac Lab venv no ve la GPU o torch no importa: $r"; fi
else falla "No existe el entorno de Isaac Lab ($PYISAAC)"; fi

# ---------------------------------------------------------------- Proyecto
seccion "8. Carpeta del proyecto"
[[ "$RAIZ" == /home/george/DSM ]] && ok "La carpeta esta en /home/george/DSM (las rutas absolutas de scripts y venvs coinciden)" \
  || falla "La carpeta esta en $RAIZ pero scripts/venvs usan /home/george/DSM"
if git -C "$RAIZ" rev-parse --is-inside-work-tree >/dev/null 2>&1; then ok "Control de versiones git activo"
else aviso "Sin control de versiones (la carpeta .git esta vacia); los respaldos se hacen copiando carpetas"; fi
[[ -f "$RAIZ/labs pasados/chatgpt_amd64.deb" ]] && aviso "'labs pasados/chatgpt_amd64.deb' (~530 MB) no pertenece al proyecto"

n=0; f=0
while IFS= read -r -d '' x; do n=$((n+1)); python3 -m py_compile "$x" 2>"$TMPD/pyc.txt" || { f=$((f+1)); info "no compila: $x"; }; done \
  < <(find "$RAIZ/herramientas" "$RAIZ/simulacion" "$RAIZ"/*.py -maxdepth 2 -name '*.py' -not -path '*/IsaacLab/*' -not -path '*/uv-*' -not -path '*/cad-env/*' -not -path '*/esp-idf*' -not -path '*/arduino-*' -print0 2>/dev/null)
(( f == 0 )) && ok "$n archivos Python del proyecto compilan" || falla "$f de $n archivos Python no compilan"
n=0; f=0
while IFS= read -r -d '' x; do n=$((n+1)); bash -n "$x" 2>/dev/null || { f=$((f+1)); info "error de sintaxis: $x"; }; done \
  < <(find "$RAIZ" "$RAIZ/firmware" "$RAIZ/simulacion" -maxdepth 1 -name '*.sh' -print0)
(( f == 0 )) && ok "$n scripts .sh sin errores de sintaxis" || falla "$f de $n scripts .sh con errores"

echo "  ... pruebas Python de vision, calibracion, control y diagnostico (sin USB)"
pruebas=(test_vision_ball_plate.py test_calibracion_ball_plate.py test_control_ball_plate.py test_diagnostico_motores_rviz.py)
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] && pruebas+=(test_panel_ball_plate.py)
( cd "$RAIZ/herramientas" && timeout 300 python3 -m pytest -q -p no:cacheprovider "${pruebas[@]}" ) >"$TMPD/pytest.txt" 2>&1
if grep -qE '[0-9]+ passed' "$TMPD/pytest.txt" && ! grep -qE '[0-9]+ (failed|error)' "$TMPD/pytest.txt"; then ok "pytest: $(grep -E 'passed' "$TMPD/pytest.txt" | tail -1)"
else falla "pytest:"; tail -15 "$TMPD/pytest.txt" | sed 's/^/          /'; fi

echo "  ... pruebas de integracion ROS del puente del dron (dominio aislado 207, simulado)"
( cd "$RAIZ/herramientas" && ROS_DOMAIN_ID=207 ROS_LOG_DIR="$TMPD/roslog" ROS_HOME="$TMPD/roshome" \
  timeout 180 python3 -m unittest -q test_puente_dron_rviz ) >"$TMPD/ros_test.txt" 2>&1
if grep -q '^OK' "$TMPD/ros_test.txt"; then ok "Puente ROS del dron: $(grep -E '^Ran' "$TMPD/ros_test.txt")"
else falla "Puente ROS del dron:"; tail -12 "$TMPD/ros_test.txt" | sed 's/^/          /'; fi

echo "  ... pruebas C++ de firmware (motores, servos, control, calibracion IMU)"
for t in test_motores_diagnostico test_servos_bandeja test_control_octo test_calibracion_imu; do
  if g++ -std=c++17 -O2 -I "$RAIZ/herramientas/pruebas_arduino" "$RAIZ/herramientas/$t.cpp" -o "$TMPD/$t" 2>"$TMPD/$t.err" && "$TMPD/$t" >"$TMPD/$t.out" 2>&1
  then ok "$t"; else falla "$t:"; tail -5 "$TMPD/$t.err" "$TMPD/$t.out" | sed 's/^/          /'; fi
done

echo "  ... tamano de las carpetas pesadas (hasta 90 s)"
( cd "$RAIZ" && timeout 90 du -sh herramientas/IsaacLab herramientas/arduino-data herramientas/esp-idf herramientas/esp-idf-tools \
  herramientas/uv-cache "labs pasados" tmp registros simulacion firmware cad 2>/dev/null | sed 's/^/          /' )

# ---------------------------------------------------------------- Resumen
echo
echo "== RESUMEN: $N_OK OK, $N_FALLA FALLA, $N_AVISO AVISO =="
echo "Informe guardado en: $LOG"
