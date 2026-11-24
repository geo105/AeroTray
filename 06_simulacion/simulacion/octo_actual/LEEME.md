# Simulación actual del octocóptero + ball and plate

Esta carpeta reúne el **control angular del ZIP que probaron en la junta** y el
**PID de ball and plate de Raspberry Pi + PS3 Eye**, confirmado por el usuario.
Gazebo calcula la dinámica del dron; RViz muestra el CAD, los sensores, la placa,
la pelota y las órdenes calculadas. Isaac Lab permite entrenar correcciones
pequeñas sobre ambos controles. Todo funciona con archivos y tópicos de simulación.

## Abrir la simulación

En una terminal de esta laptop:

```bash
# Gazebo + RViz, con los dos controles originales:
bash /home/george/DSM/simular_dron_actual.sh banco

# Comparar el candidato de PID, exclusivamente en simulación:
bash /home/george/DSM/simular_dron_actual.sh banco candidato

# RViz reproduciendo el TXT real, sin inventar medidas de altura o pelota:
bash /home/george/DSM/simular_dron_actual.sh real
```

Terminar con Ctrl+C en la terminal. Cerrar solo la ventana de RViz no termina el
nodo: Ctrl+C cierra también los procesos de simulación iniciados por el script.
Los datos ROS aparecen bajo `/dsm_sim/`; no se abre USB, WiFi ni un puente al ESP32.
El servidor de Gazebo usa una partición separada de los proyectos anteriores.

## Lo que está incluido y su procedencia

- **Angular:** P de ángulo `[5.00, 5.03]`, PI de velocidad P `[0.760, 0.753]`,
  I `[0.1, 0.1]`; integral limitada, mezclador que conserva empuje modelado,
  límite de velocidad, conversión provisional a PWM y cortes del firmware.
  Control a 200 Hz, órdenes ESC a 50 Hz y dinámica de motor de primer orden.
  El PI de yaw permanece desactivado, como en el firmware recibido.
- **Ball and plate:** Kp=1.875, Ki=0.5, Kd=1.5625; neutros 1359/1463 us,
  límite de PID 220 us, factores/signos y holguras `[6,12,35,15]` con rampa 30 us.
  Se conserva la integral condicional y limitada y la derivada sobre medición.
  La saturación continúa antes de sumar holgura, igual que en el original.
  La velocidad externa se estima mediante recta de los últimos 60 ms.
- **Geometría y masa:** CAD del ZIP, transformado a FLU: X frente, Y izquierda,
  Z arriba. Se mantiene la numeración física del firmware, M1 al frente y M8
  delante a la izquierda. El mapa CAD/físico del ZIP está en `datos/cad.json`.
  Las mallas se separaron para mover las dos mitades de la placa y sustituir
  la bola rígida de referencia por una esfera móvil de 27 mm.
- **Evidencia real:** 260 muestras, 25.9 s, a 10 Hz. Se conserva el archivo original
  y su huella SHA256. No contiene altura, posición de pelota ni empuje medido.

Los originales copiados están en `datos/`. `config.json` registra las fuentes,
huellas y supuestos. No se sobrescribió el firmware, los ajustes de la Pi ni
las simulaciones anteriores.

## ToF en brazo 1 e IMU en brazo 8

El ToF se coloca a `[0.35, 0, 0.0955]` m desde el pivote y apunta hacia abajo.
La altura se obtiene de la intersección del rayo con el suelo, compensando la
inclinación estimada y el brazo del sensor. Su alcance máximo de 2 m **no define
el recorrido de las guías**.

La IMU se coloca provisionalmente en `[0.2475, 0.2475, 0.1055]` m. Su señal incluye
gravedad, aceleración de la base y términos tangenciales y centrípetos por estar
fuera del pivote. El estimador reproduce el filtro complementario del ZIP.
La ubicación en brazo 8 fue comunicada como tentativa; distancias exactas y
orientaciones de ambos sensores siguen pendientes de medir.

El firmware previo identifica el ToF como VL53L0X; eso no verifica su etiqueta.
Su frecuencia de 20 Hz, ruido de 2 mm y filtro de velocidad de 50 ms son supuestos.
La simulación usa el ruido de IMU en reposo documentado en el proyecto; el TXT
de motores encendidos no se interpreta como ruido puro del sensor.

## Qué dinámica sigue siendo aproximada

Las masas del dron (2.8 kg) y de la junta completa (0.6 kg) proceden del proyecto.
Se usan 2.999524 kg rotatorios y 0.400476 kg en la guía: el reparto por volumen,
CG e inercias proceden del CAD y siguen siendo estimaciones. No se mezcla este
modelo con el antiguo de 3.6 kg. Falta identificar la masa adicional que suba
con las guías, si existe, así como rozamiento y curva empuje/PWM/tensión.

La pelota usa un modelo reducido de esfera maciza en rodadura (factor 5/7), con
servos de primer orden, juego, latencia y medidas de posición sintéticas. El
cambio de par por desplazamiento de la pelota perturba el dron. Placa y pelota
ya están incluidas en la masa nominal: no se suma de nuevo su peso.
No se representan las reacciones completas de los servos, contactos de mallas,
deslizamiento, cámara renderizada ni el detector por color. Al salir la pelota
de la superficie, se marca fallo y se conserva el primer estado de salida;
no se simula una caída ni un rebote artificial. Masas de pelota/placa, ganancia
mecánica, ejes cámara/FLU, FPS efectivos y latencia son supuestos editables.
La cámara simula 100 Hz; los 125 fps del archivo de la Pi son una petición,
no una medición confirmada.

## Resultados de comparación

`resultados/resumen.json` y `resultados/comparacion.png` reúnen los resultados.

La búsqueda de PID probó 48 candidatos sobre 24 casos y reservó otros 24 para
evaluación, cambiando condiciones iniciales, inclinaciones, retardo, servo y
ruido. El candidato **Kp=2.8125, Ki=0.75, Kd=2.34375** redujo el RMS radial de la
pelota de 30.8 a 19.9 mm en la reserva del modelo reducido, sin pérdidas.

Después se comparó en **tres parejas de corridas de Gazebo de 12 s**, con semillas
81243, 91357 y 104729 y perturbaciones idénticas por pareja: el RMS medio de
pelota bajó de **26.3 a 15.0 mm (43.0%)**, sin pérdidas ni cortes en el candidato.
La ganancia no se volvió a seleccionar usando estas pruebas. Es una mejora del
modelo provisional; no acredita ese porcentaje en el prototipo.

## Altura

```bash
# Ensayo con el techo y curva originales: caso que muestra la falta de autoridad.
bash /home/george/DSM/simular_dron_actual.sh altura

# Estudio del lazo de altura con fuerzas ideales:
bash /home/george/DSM/simular_dron_actual.sh altura_ideal
```

Con la curva SUPUESTA del ZIP, los ocho motores a 1500 us producen 10.72 N,
frente a 33.35 N de peso modelado. En `altura`, la demanda satura, se pierde
autoridad angular y aparece un corte por inclinación. Ese resultado no mide
la capacidad real de los motores ni autoriza cambiar el techo físico.

`altura_ideal` usa fuerzas de hasta 5.3955 N por motor sin equivalencia PWM validada
(se muestra PWM=-1). Empieza a **15 mm con empuje de equilibrio ya establecido**,
no desde un despegue, y aumenta la referencia a 5 mm/s hacia 38.3 mm. En la prueba
de 14 s terminó a 39.8 mm, sin corte ni pérdida de pelota. Los límites de aceleración
siguen activos. El recorrido de 76.63 mm procede de la subida desde la pose CAD;
no es una medición de todo el recorrido físico.

## Isaac Lab: tarea nueva y resultado del entrenamiento

Se registró `DSM-Octo-Actual-Residual-v0`, independiente de las políticas viejas:

- Junta con altura fija; Newton/MJWarp en la GPU de la laptop.
- Ambos controles originales siguen activos. Cuatro acciones residuales:
  dos correcciones RP de hasta ±0.025 Nm y dos de servo de hasta ±10 us.
- Dieciséis observaciones: actitud/velocidad estimadas, estados de integral,
  posición/velocidad de pelota del estimador y acciones previas.
- Física de junta a 1 kHz, angular a 200 Hz y política a 50 Hz.
  La rodadura/sensores usan los mismos módulos reducidos CPU que Gazebo.
- 100 iteraciones, 8 entornos y 25,600 transiciones. Se guardó
  `modelos/residual_100_iteraciones.pt`.

En reserva de 8 episodios de 6 s, ambos completaron 8/8. Los controles originales
dieron RMS de pelota 24.2 mm e inclinación 0.598°; la política inicial dio 26.5 mm
y 0.684°. **La política no mejoró la referencia y no fue seleccionada**.
El entrenamiento queda disponible para continuar investigaciones; no se exportó
ni aplicó esta política al hardware. La tarea nueva no entrena altura.

```bash
# Entrenar una corrida nueva (sin reemplazar checkpoints anteriores):
bash /home/george/DSM/simulacion/octo_actual/isaac.sh train

# Evaluar controles originales:
bash /home/george/DSM/simulacion/octo_actual/isaac.sh evaluar

# Evaluar la política guardada con la misma semilla de reserva:
bash /home/george/DSM/simulacion/octo_actual/isaac.sh evaluar \
  --checkpoint /home/george/DSM/simulacion/octo_actual/modelos/residual_100_iteraciones.pt

# Ver su trayectoria en RViz, incluida la pelota del modelo reducido:
bash /home/george/DSM/simulacion/octo_actual/rviz.sh \
  --fuente /home/george/DSM/simulacion/octo_actual/resultados/isaac_residual.csv
```

La reproducción de CSV en RViz muestra el estado simulado guardado. El visor Viser
de Isaac representa el CAD rígido de referencia; la placa y pelota reducidas se
animan en Gazebo/RViz. Isaac Sim completo no es necesario para esta tarea de Newton.

## Repetir las comprobaciones

```bash
python3 /home/george/DSM/simulacion/octo_actual/verificar_control.py
python3 /home/george/DSM/simulacion/octo_actual/ajustar_ball_plate.py
bash /home/george/DSM/simulacion/octo_actual/gazebo.sh --probar --segundos 12
bash /home/george/DSM/simulacion/octo_actual/gazebo.sh --probar \
  --modo altura_ideal --segundos 5 --fallo-tof --salida altura_fallo_tof
```

La equivalencia del PID contra su C++ original se comprobó en 1600 muestras:
máximo error previo a holgura 0.0000303 us y **cero diferencia en pulsos enteros**.
También se verificaron la compensación geométrica del ToF, la aceleración de IMU
fuera del pivote y los cortes por inclinación y ToF inválido. La prueba de fallo
ToF en Gazebo guarda su diagnóstico aparte. Los informes de visores comprueban
ventanas OpenGL y recepción de datos ROS; las capturas están en `resultados/`.

Para reimportar STL desde el ZIP: `python3 importar_mallas.py`; transforma también
normales y verifica los límites de 382 piezas contra el JSON CAD. Para regenerar
SDF/MJCF: `python3 preparar.py` desde esta carpeta. Para regenerar
el USD con el entorno instalado: `bash isaac.sh preparar`. Después de cambiar
parámetros físicos o controles, repetir comparación y entrenamiento: un
checkpoint anterior no queda actualizado automáticamente.

Implementación consultada: [API Python de Gazebo Harmonic](https://gazebosim.org/api/sim/8/python_interfaces.html)
y el código local de Isaac Lab instalado en `/home/george/DSM/herramientas/IsaacLab`.
