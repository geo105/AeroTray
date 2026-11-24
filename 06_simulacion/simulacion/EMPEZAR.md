# Simulación actual del conjunto CAD

El conjunto del carrito y la etapa UAV ya está incorporado como geometría
CAD, con la bandeja nueva de cámara. Son 383 componentes; los originales
quedaron intactos. El prototipo real está en la universidad.

Para abrirlo en una terminal de Ubuntu, cierra el visor anterior y ejecuta:

```bash
/home/george/DSM/simulacion/ver_cad.sh
```

El visor abre `/home/george/DSM/simulacion/aerotray_conjunto_cad.xml`, con los
paneles laterales ocultos. Muestra el control de referencia de altura e inclinación activo. Cierra la ventana
para salir. La captura de prueba está en `conjunto_cad_simulacion.png`.

## Qué viene del CAD

- Geometría del carrito, guías, rótula, UAV de ocho motores y bandeja nueva.
- Centro esférico de la rótula y ubicación de los ocho ejes, a radio 350 mm.
- Guía vertical y clasificación de piezas en cuerpos, interpretadas del STEP.
- Contacto de topes tras subir 78.629459 mm desde la pose guardada. Se limita
  la simulación a 76.629459 mm con 2 mm de margen numérico.

El usuario pidió priorizar CAD sobre el dato anterior de 20 cm. Q=0 es la
pose guardada, no un tope inferior identificado. Este rango solo representa
subida desde esa pose; no es la medición del recorrido total del prototipo.

## Qué sigue provisional

La masa móvil total de 3.6 kg procede del pesaje comunicado. El reparto
0.3 kg lineal / 3.3 kg rotatorio, COM, inercias y amortiguamientos no están
identificados. 6 N por motor y par/empuje de 0.015 m son valores ficticios
para probar el modelo, sin relación calibrada con PWM. La numeración se toma
de las hélices CAD; se debe verificar contra GPIO y giro real de cada motor.

Hay cuatro grados de libertad: subida y rótula esférica ideal. El carrito
permanece fijo (lo trabaja el compañero). La bola, plato y servos son rígidos,
sin visión, cámara simulada ni control del ball-and-plate. La tarea de Isaac Lab
entrena únicamente la altura y actitud de la etapa UAV.

La interfaz original tiene solapes heredados con la rótula. Se conservó para
revisión y se marca en magenta en la vista CAD de detalle. Las colisiones de
las mallas están desactivadas. No se han identificado límites angulares.

## Verificación realizada

- El STEP completo se volvió a leer con los mismos 383 componentes y nombres.
- `verificacion_conjunto_cad.json`: MuJoCo, 2 s de equilibrio ideal; Newton /
  MJWarp en GPU, 0.2 s. Ocho actuadores y masa móvil 3.6 kg.
- `supuestos_conjunto_cad.json`: procedencia, límites y fuerzas de trim.
- Gravedad explícita de 9.80665 m/s² en ambos motores físicos.

La prueba comprueba importación, unidades, cuerpos y fuerzas ideales; no
valida el control ni el prototipo. Isaac Lab está instalado y su ejemplo
oficial Cartpole se probó previamente en GPU. Esa prueba no entrenó el dron.

Los archivos `aerotray_provisional.xml` y `aerotray_cad_visual.xml`, y sus
informes antiguos, se conservan como revisiones anteriores. Sus guías,
posiciones y el rango de 0.20 m ya no definen el modelo actual.

## Control de referencia del UAV

El visor aplica un PI-D de altura e inclinación con reparto acotado entre
ocho motores y anti-windup. Pulsa P para un impulso pequeño de prueba y R
para reiniciar solo la simulación. Para repetir la vista antigua sin control:

```bash
/home/george/DSM/simulacion/ver_cad.sh --sin-control
```

Doce ensayos sintéticos pasaron: nominal de 60 s; inclinación inicial, guiñada,
impulso, cambio de altura, masa ±15 %, inercia +30 %, desplazamiento de COM,
un motor 15 % más débil, ruido/retardo de 30 ms y recuperación de saturación.
Resultados y CSV: `/home/george/DSM/simulacion/control_referencia`.
Estas son pruebas del modelo provisional, no mediciones del prototipo.

El visor anterior sigue siendo MuJoCo. Ahora también existe la tarea real
de Isaac Lab `DSM-AeroTray-Direct-v0`, con PPO y Newton/MJWarp en GPU.
Las gráficas están en http://127.0.0.1:6006 mientras corre TensorBoard.
Para observar la política aprendida en Isaac Lab:

```bash
/home/george/DSM/simulacion/ver_politica_isaaclab.sh
```

Abrir después http://127.0.0.1:8080. Es una reproducción con Viser de la
política guardada, no aprendizaje en vivo. Para las curvas y cómo reiniciar
los servidores, consultar `ISAAC_PLAN.md`. Isaac Sim completo no está instalado.

El siguiente trabajo mecánico es revisar la interfaz que presenta solapes y
confirmar la asignación de cuerpos. Después, identificar empuje, inercias y
fricción; ampliar la evaluación de la política fuera de los estados iniciales entrenados.
Ver `/home/george/DSM/cad/LEEME.md`, `REVISION_EMPUJE.md` e `ISAAC_PLAN.md`.
