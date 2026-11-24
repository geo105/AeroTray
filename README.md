# AeroTray

> **Repositorio:** https://github.com/geo105/AeroTray  
> **Paper (PDF en inglés y español):** https://github.com/geo105/AeroTray/releases/tag/v1.0  
> **Videos y CAD completos (>95 MB):** se publican aparte (Zenodo/Drive); el enlace/DOI se agregará aquí.

**Proyecto:** robot móvil diferencial (TETRIX + NI myRIO-1900) que transporta un octocóptero de 8 motores sobre una rótula pasiva con guía vertical y, encima, una bandeja *ball and plate* controlada por visión a 187 fps (Raspberry Pi 3B+ + ESP32-S3).

**Curso:** MT5001 Diseño de Sistemas Mecatrónicos, UTEC, 2026 — Grupo 1.

**Autores:**
- Sebastián Rejas Córdova
- George Henry Ramos Astuhuamán
- Johan Nireo Naupay Fabián
- Hideky Edu Garnique Pisfil
- Diego André Miranda Zamora

> **English summary.** Companion open-source repository for the HardwareX manuscript *"AeroTray: an open mobile robot carrying a gimbal-mounted octocopter and a vision-based ball-and-plate stage"*. It provides the LaTeX manuscript sources (English and Spanish), CAD assemblies and Proteus schematics, ESP32/ESP32-S3 firmware, Raspberry Pi vision and supervisor software, LabVIEW/myRIO project files, simulation models (MuJoCo, Isaac Lab, ROS 2), analysis scripts, and all experimental logs required to reproduce every figure and metric in the paper.  
> **Licenses:** Hardware — CERN-OHL-S v2 · Software — MIT · Documentation & Data — CC BY 4.0.

---

## Estructura del repositorio

| Carpeta | Contenido |
|---|---|
| `00_paper_HardwareX/` | Fuente LaTeX final v10 en inglés (`final_v10_ingles/fuente_latex/`) y español (`final_v10_espanol/fuente_latex/`), scripts simbólicos/analíticos (`scripts_modelo/`), figuras MATLAB (`figuras_matlab/`), y scripts de reproducibilidad (`scripts_repositorio/bp_metrics.py`, `make_figures.py`). Los PDF compilados están en [Releases](https://github.com/geo105/AeroTray/releases/tag/v1.0). |
| `01_guias_y_documentacion/` | Guías de operación y arquitectura (`GUIA_EQUIPO.md`, `VUELO_GUIA.md`, `UART_MYRIO.md`, `PRUEBA_DRON_BALLPLATE.md`, `RESUMEN_TRABAJO.md`), protocolo de pruebas finales del 23/11 y hoja imprimible de calibración ArUco (`imprimibles/`). |
| `02_cad/` | Esquemáticos Proteus (`esquematicos_proteus/AeroTray_Esquematicos.pdsprj` y vistas PDF), modelo visual 3D del ensamble completo (`.glb`), STEP de la bandeja (`recibidos/ball_and_plate_camara.stp`), componentes COTS y mallas STL procesadas. Los STEP de >95 MB se publican en el archivo externo (Zenodo). |
| `03_firmware/` | Código fuente Arduino de los ESP32 (`firmware/captura_octo_m8_vuelo_guia/` — versión final **v11**, `captura_octo_m8_banco_angulo_laser/`, diagnósticos y scripts de compilación/carga), proyecto PlatformIO de la bandeja (`ball_plate_esp32s3_platformio/`) y librerías Arduino con sus licencias (`librerias_arduino/`). |
| `04_raspberry_pi/` | Supervisor de la Raspberry Pi (`pi_control/seguidor.py`, `comunicacion.py`, `myrio_uart.py`, `pagina_dron.py`, `pagina_robot.py`, `config_pi.json`, `ajustes.json`, `arrancar.sh`) y prueba de cámara (`pi_webcam_fps/`). |
| `05_labview_myrio/` | Proyecto LabVIEW 2019 de la base diferencial (`Control_Robot_Movil/`, VI principal `prueba_prueba.vi`), código extraído de los *Formula Node* en C (`codigo_extraido_de_los_VI/`) y paquete de integración UART con myRIO (`uart_raspberry_myrio_diego/`). |
| `06_simulacion/` | Modelo MuJoCo (`simulacion/aerotray_conjunto_cad.xml`), entorno Isaac Lab (`simulacion/isaaclab_aerotray/`), modelos URDF/SDF y scripts de simulación ROS 2 / Gazebo. |
| `07_herramientas/` | Scripts de análisis, comparación de vuelos contra Isaac Lab (`comparar_vuelo_isaac.py`), visión, puentes ROS 2, diagnósticos y pruebas unitarias. |
| `08_datos_y_registros/` | Registros experimentales usados en el manuscrito: campañas de la Raspberry Pi (`copia_raspberry_pi_20261120/`), vuelos del octocóptero y comparaciones con Isaac Lab (`registros_vuelo_dron/`), y ensayos integrados finales del 23/11 (`ensayos_finales_20261123/`). |

---

## Reproducir métricas y figuras del paper

### 1. Métricas experimentales y figuras principales (Python 3)

Requiere Python 3 con `numpy` y `matplotlib`:

```bash
cd 00_paper_HardwareX/scripts_repositorio
python3 bp_metrics.py
python3 make_figures.py
```

Los scripts leen directamente los registros crudos en `08_datos_y_registros/` y generan las salidas en `00_paper_HardwareX/datos_regenerados/` y `00_paper_HardwareX/figuras_regeneradas/`.

### 2. Verificación simbólica y modelo analítico

En `00_paper_HardwareX/final_v10_ingles/fuente_latex/scripts_modelo/` (o su equivalente en `final_v10_espanol/`) se encuentran los scripts SymPy y numéricos (`derive_ball.py`, `derive_drone.py`, `params.py`, `analysis.py`, `ball_dt.py`, `chain.py`, `rampa.py`, `bateria.py`, `exportar_datos_matlab.py`).

---

## Datos experimentales (`08_datos_y_registros/`)

- **`copia_raspberry_pi_20261120/pi/`:** registros de *ball and plate* a ~187 Hz (`registro_*.csv`), vuelos a 10 Hz (`vuelo_*.csv`), buffer a bordo a 200 Hz (`dron_log_*.csv`), `salida.log`, `ajustes.json` y `config_pi.json`. *(Nota: tras el reinicio del 20/11 a las 09:55 el reloj de la Pi quedó adelantado; los archivos `15xxxx` de ese día corresponden a las 10–11 h locales).*
- **`registros_vuelo_dron/`:** registros de vuelo del octocóptero y reportes de comparación frente a Isaac Lab.
- **`ensayos_finales_20261123/`:** registros, telemetría sincronizada y scripts de análisis de los ensayos completos sobre rampa y despegue del 23/11.

---

## Licencias

| Licencia | Aplica a |
|---|---|
| **CERN-OHL-S v2** (`LICENSES/CERN-OHL-S-2.0.txt`) | Hardware: `02_cad/` (diseños CAD y esquemáticos). |
| **MIT** (`LICENSE`) | Software: `03_firmware/`, `04_raspberry_pi/`, `05_labview_myrio/`, `06_simulacion/`, `07_herramientas/` y scripts de `00_paper_HardwareX/`. Las librerías de terceros en `03_firmware/librerias_arduino/` conservan sus licencias originales. |
| **CC BY 4.0** (`LICENSES/CC-BY-4.0.txt`) | Documentación y datos: `00_paper_HardwareX/` (manuscrito y figuras), `01_guias_y_documentacion/` y `08_datos_y_registros/`. |

---

## Archivos alojados aparte (por límite de 100 MB de GitHub)

Para mantener el repositorio ligero y navegable, los archivos de gran tamaño y medios crudos se distribuyen aparte (enlace/DOI en la cabecera):

- Videos de ensayos y pruebas (`IMG_1867–1869.mov`, campañas del 18/11 y 23/11).
- Ensambles STEP completos de >95 MB (`junta nuevo de 8 motores con camara.step`, `x8_UAV_R350_imprimible_bandeja_v5.stp`, `nuevo+8+camara.stp` y sus `.f3z`).
- Artefactos compilados (`.elf`, `.map`, `.merged.bin`), geometrías intermedias (`.brep`) e instalaciones de terceros (Isaac Lab, ROS 2, Gazebo, Raspberry Pi OS, arduino-cli), que se obtienen desde sus fuentes oficiales.
