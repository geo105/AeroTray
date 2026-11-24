# Isaac Lab, Isaac Sim y el modelo del dron

## Qué está instalado

**Isaac Lab** está instalado en `/home/george/DSM/herramientas/IsaacLab`,
con entorno Python en `.venv`. Revisión fijada:
`44044b0963001962abe5701133427e37b003fffe`, rama `release/3.0.0`.
Se completó un ensayo oficial `Isaac-Cartpole-Direct`, con Newton/MJWarp en
CUDA, 16 entornos y 5 iteraciones. Ver `verificacion_isaaclab.json`.
Esto verificó inicialmente las herramientas. Ahora también está registrada la
tarea real del dron `DSM-AeroTray-Direct-v0`; ver los comandos siguientes.

**Isaac Sim completo no está instalado**. Se eligió la ruta de Isaac Lab
con Newton para la laptop (RTX 4060 Laptop de 8 GB, aproximadamente 24 GB RAM).
No se contrató equipo remoto ni servicio de nube.

**La ventana que abre `ver_cad.sh` es MuJoCo**, no Isaac Sim ni una tarea de
Isaac Lab. El visor ahora indica MuJoCo/control de referencia en pantalla.
Se usa para revisar geometría, reconstruir uniones y probar un controlador
antes de preparar el aprendizaje. Ejecutar ese visor con el Python de la
carpeta Isaac Lab no convierte la simulación en una tarea de Isaac Lab.

## Qué ya existe para el dron

- CAD completo con carrito, guía, rótula, ocho motores y bandeja nueva.
- Modelo `/home/george/DSM/simulacion/aerotray_conjunto_cad.xml`.
- Subida desde la pose CAD limitada a 76.63 mm; quedan 2 mm de margen antes
  del contacto geométrico de topes. No es la identificación del recorrido total.
- Control de referencia PI-D de altura y actitud, asignación acotada de ocho
  fuerzas y anti-windup, en `control_simulacion.py`.
- Doce ensayos sintéticos aprobados en MuJoCo: sostén nominal de 60 s y casos
  de 12 s de perturbación, variación de masa, inercia, COM, un motor más débil,
  ruido/retardo y saturación. Ver `control_referencia/resultados.json`.

El modelo conserva masa total de 3.6 kg del pesaje. Reparto de masa, inercias,
empuje máximo y dinámica de motores no están identificados. Hay solapes
heredados del CAD, sin colisiones físicas activas. La bola, plato y servos
siguen rígidos; la cámara aún no controla el sistema.

## Tarea real de Isaac Lab y cómo verla

`simulacion/isaaclab_aerotray` registra `DSM-AeroTray-Direct-v0` con
`DirectRLEnv`, Newton/MJWarp en CUDA y PPO de RSL-RL. Se convirtió el modelo
CAD a USD usando el conversor oficial `mujoco_usd_converter`, conservando
la guía prismática y la rótula. El carrito permanece fijo.

Hay 8 acciones (variación del empuje de cada motor alrededor del trim),
17 observaciones y control a 50 Hz, con integración a 500 Hz. La política
aprende directamente las fuerzas; no tiene el PI-D escondido dentro. Los
episodios duran 8 s y comienzan con inclinación de hasta 5 grados por eje,
altura desplazada hasta 10 mm y pequeñas velocidades. El límite de caída
de 30 grados es una condición de fin de episodio, no un límite mecánico medido.

Se completó una corrida de 1000 iteraciones con 64 entornos en GPU
(2 048 000 pasos, aproximadamente 4 min 20 s):
`herramientas/IsaacLab/logs/rsl_rl/aerotray_cad/2026-10-30_14-45-15_primer_entrenamiento`.
El log está en `simulacion/isaaclab_aerotray_entrenamiento.log`.

Para las gráficas, abrir http://127.0.0.1:6006 en la misma laptop. Si el
servidor está apagado, ejecutar en una terminal:

```bash
/home/george/DSM/simulacion/ver_graficas_isaaclab.sh
```

`Train/mean_reward` es la recompensa; `Train/mean_episode_length` es la
duración de los episodios; `Metrics/success_rate` indica qué proporción llegó
al final del episodio sin terminar antes. No significa éxito en el prototipo.

Para observar en 3D la última política guardada, ejecutar y dejar la terminal abierta:

```bash
/home/george/DSM/simulacion/ver_politica_isaaclab.sh
```

Después abrir http://127.0.0.1:8080. Es la tarea real de Isaac Lab con su
visualizador web Viser y Newton; no es la aplicación completa Isaac Sim.
La reproducción usa un checkpoint fijo: no modifica los pesos ni continúa
el aprendizaje. Al reiniciar el visor se carga el checkpoint más reciente.
Ambos servidores escuchan solo en esta laptop, sin compartir públicamente.

Para iniciar otra corrida, `simulacion/entrenar_isaaclab.sh`. No es necesario
volver a entrenar para observar el resultado ya guardado.

`evaluar_isaaclab.py` compara referencia y política durante 8 s con 64 estados
iniciales de una semilla separada. Los informes se guardan en
`isaaclab_aerotray/evaluacion_referencia.json` y `evaluacion_policy.json`.
La prueba de trim de 2 s aprobó en Isaac Lab con la masa móvil de 3.6 kg.
La evaluación inicial no incluye incertidumbre física ni perturbaciones externas.

Resultados de evaluación (64 estados iniciales, semilla 73021, 8 s):

| Control | Episodios completos | RMS altura últimos 2 s | RMS inclinación últimos 2 s |
|---|---:|---:|---:|
| Referencia PI-D | 64/64 | 0.0153 mm | 0.0403 grados |
| PPO, checkpoint model_999.pt | 64/64 | 0.0403 mm | 1.0768 grados |

Los errores anteriores se calculan sobre los episodios que siguen vivos;
en esta evaluación sobrevivieron todos. La política inicial sostiene el modelo,
pero nivela con menos precisión que la referencia. No se afirma mejora por
usar aprendizaje ni robustez física. El checkpoint está guardado en la carpeta
de la corrida, y `play` exportó también JIT y ONNX en su subcarpeta `exported`.
La captura `isaaclab_aerotray/visor_isaaclab.jpg` muestra el visualizador real.

La primera política, si usa estos parámetros provisionales, solo representará
ese modelo de simulación. Compararla con el control de referencia bajo los
mismos ensayos. Antes del prototipo hacen falta curva empuje/voltaje/PWM,
inercias, retardos y ruido medidos; no basta con exportar una red al ESP32.

La entrega Lab 5 requiere validación experimental: simulación y entrenamiento
apoyan las pruebas, sin sustituirlas. El hardware está en la universidad.

## Referencias oficiales consultadas

- https://docs.nvidia.com/isaaclab/3.0/isaac-lab/installation
- https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
- https://github.com/isaac-sim/IsaacLab/tree/release/3.0.0
