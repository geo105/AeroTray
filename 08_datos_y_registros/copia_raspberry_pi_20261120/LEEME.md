# Datos para el informe

Copia completa de lo que guardó la Raspberry Pi, traída a la laptop el 20/11/2026.

**Ojo con las horas:** desde el reinicio del 20/11 a las 09:55, la Pi tiene la hora mal. Los archivos de esa tarde (nombres 15xxxx) son en realidad de alrededor de las 10–11 h.

| Carpeta / archivo | Qué es |
|---|---|
| `pi/registros/vuelo_*.csv` | Un archivo por cada encendido del dron, a 10 Hz: roll, pitch, yaw, p, q y r, los 8 motores en µs, el acelerómetro, el láser y el colectivo objetivo. Lo escribe la Pi con la telemetría del ESP32 del dron. |
| `pi/registros/dron_log_*.csv` | Los últimos 16 s a 100 Hz, exportados desde el firmware del dron con el botón «Exportar log». |
| `pi/registros/registro_*.csv` | Ball and plate: un archivo por cada arranque del programa. Tiene la posición de la pelota (cámara) y los pulsos de los servos (ESP32-S3). |
| `pi/salida.log` | Todo lo que imprimieron la Pi, el ESP32 del dron y el ESP32-S3: arranques, órdenes, cortes y mensajes de la UART del myRIO. |
| `pi/ajustes.json`, `pi/config_pi.json` | Las ganancias y la configuración actuales del ball and plate y de la UART. |
| `../registros_dron/` | Los mismos `vuelo_*.csv` más sus comparaciones con Isaac Lab (`*_comparacion.md` y `.png`). |
| `../registros/` | Los ensayos de banco de octubre: identificación de motores, comparaciones de control y simulación. |
| `../LAB6/analizar_ensayos.py`, `../herramientas/comparar_vuelo_isaac.py` | Los scripts que hacen las tablas y las gráficas. |

## Vuelos del 20/11 (firmware y resultado)

| Registro | Firmware | Resultado |
|---|---|---|
| vuelo_091438, vuelo_092412 | v2–v3 | Cortes por vibración del acelerómetro y por yaw |
| vuelo_093101 | v4 | El pitch se fue a 9,7° a 1700 µs |
| vuelo_093636 | v5 | Flotó a 1750–1800 µs, pero inestable (picos de 9°) |
| vuelo_094224, vuelo_150927 | v6 | Corte por pitch o por roll entre 1700 y 1725 µs |
| vuelo_152249 | v8 | **20 s estable a 1725–1775 µs** (roll RMS 0,55°); cortó al aterrizar |
| vuelo_152743 | v9 | 24 s sobre 1700 µs (roll RMS 1,2°, pitch 1,0°); **aterrizó sin corte** |
| vuelo_154605 | v9 | 48 s sobre 1700 µs, hasta 1816 µs; terminó con corte por pitch a 12° |

Los datos de los ESP32 llegan a través de la Pi y quedan en estos archivos. Los ESP32 no guardan nada por su cuenta, salvo el buffer de 16 s del dron, que se exporta con «Exportar log».

## Ball and plate del 18/11 (el día de los videos del carrito y las rampas)

La hora de la Pi era correcta ese día. Los videos de WhatsApp no conservan la hora de grabación; las horas de envío fueron entre las 18:53 y las 18:57. Para emparejar cada video con su tramo de datos, usen la hora de grabación que muestra la galería del celular.

| Archivo | Horario | Tramos con PID encendido y pelota (inicio, duración) |
|---|---|---|
| registro_20261118_143523.csv | 14:35–15:15 | 14:57:39 (78 s), 15:07:44 (54 s), entre otros |
| registro_20261118_151550.csv | 15:15–15:23 | 15:16:39 (124 s), 15:19:44 (88 s), 15:22:52 (46 s) |
| registro_20261118_153948.csv | 15:39–16:28 | 15:44:16 (284 s), 15:53:14 (105 s), 16:15:32 (66 s) |
| registro_20261118_162411.csv | 16:24–17:08 | 16:53:27 (82 s), 16:57:21 (99 s), 17:00:54 (126 s) |
| registro_20261118_170817.csv | 17:08–17:17 | 17:13:54 (49 s), 17:15:09 (131 s) |

Columnas: `tiempo_unix`, `x_mm`, `y_mm`, `area_mm2`, `pulso_x_us`, `pulso_y_us`, `estado` y `umbral`. Cuando `estado & 3 == 3`, el PID está encendido y ve la pelota.
