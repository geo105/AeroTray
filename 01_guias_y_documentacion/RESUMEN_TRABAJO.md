# Resumen rápido: Raspberry Pi y códigos del proyecto

## 1. Conectarse a la Pi

- **Red:** hotspot local.
- **IP de la Pi:** `<IP_PI>`. Si cambia, búsquenla por su MAC (`<MAC_PI>`).
- **Usuario:** `<usuario>`.

Para entrar por ssh:

```bash
ssh <usuario>@<IP_PI>
```

Páginas web:

| Página | Para qué sirve |
|---|---|
| `http://<IP_PI>:8080/` | Ball and plate (cámara, PID) |
| `http://<IP_PI>:8080/dron` | Dron: calibrar, armar, vuelo en la guía, log |
| `http://<IP_PI>:8080/robot` | Robot completo |
| `http://<IP_PI>:8080/estado` | Estado en JSON |

## 2. En la Pi (`~/pi_control`)

| Archivo | Qué es |
|---|---|
| `seguidor.py` | El programa principal: cámara, PID, web y enlaces |
| `comunicacion.py` | El enlace con el dron por USB (clase `Uav`) y con el myRIO por UART y TCP (clase `Enlace`) |
| `myrio_uart.py` | La UART del myRIO: `/dev/serial0` a 9600 8N1 |
| `pagina_dron.py` y `pagina_robot.py` | Las páginas web del dron y del robot |
| `config_pi.json` y `ajustes.json` | La configuración. **No usen** las versiones del zip de Diego. |
| `arrancar.sh` | Lazo que arranca `seguidor.py` y lo reinicia si se cae |
| `salida.log` | Todo lo que imprime el programa |
| `registros/vuelo_*.csv` | Un archivo por cada encendido del dron, a 10 Hz |

- El entorno de Python (venv) es `~/venv`. Ahí también está esptool.

Ver el log en vivo:

```bash
ssh <usuario>@<IP_PI> tail -f pi_control/salida.log
```

**Reiniciar el programa.** Ojo: `pkill` mataría su propia sesión de ssh si va en la misma línea del comando. Por eso conviene entrar primero por ssh y ejecutar:

```bash
cd ~/pi_control
pkill -f seguidor.py
setsid nohup sh ./arrancar.sh >/dev/null 2>&1 </dev/null &
```

Traer los registros de vuelo a la PC:

```bash
scp <usuario>@<IP_PI>:pi_control/registros/vuelo_*.csv ~/DSM/registros_dron/
```

## 3. En la PC (`~/DSM`)

| Dónde | Qué |
|---|---|
| `pi_control/` | La copia local de lo que corre en la Pi. Se edita aquí y se sube con `scp` o con los scripts de abajo. |
| `pi_control/subir_uart_a_pi.sh` | Sube `comunicacion.py`, `seguidor.py`, `myrio_uart.py` y `config_pi.json`, y reinicia el programa |
| `firmware/captura_octo_m8_vuelo_guia/` | El firmware de vuelo del dron (**v8**). Hay copias de cada versión anterior: `.antes_v8`, `.antes_v7`, etc. |
| `firmware/captura_octo_m8_banco_angulo_laser/` | El firmware de banco de siempre |
| `firmware/cargar_vuelo_guia_desde_pi.sh` | Graba el firmware de vuelo en el dron desde la Pi, **con las LiPo desconectadas**. Con `--volver` regresa al de banco. |
| `firmware/compilar_*.sh` | Compilan los firmwares (arduino-cli en `herramientas/`) |
| `herramientas/comparar_vuelo_isaac.py` | Compara un vuelo real con Isaac Lab |
| `VUELO_GUIA.md` | El procedimiento de vuelo y los riesgos |
| `UART_MYRIO.md` | El cableado y el protocolo Pi ↔ myRIO |
| `LAB6/` | La presentación, el póster, el video y sus scripts |

Compilar el firmware del dron:

```bash
cd ~/DSM && herramientas/arduino-cli/arduino-cli --config-file herramientas/arduino-cli.yaml compile --fqbn esp32:esp32:nodemcu-32s --output-dir firmware/bin/captura_octo_m8_vuelo_guia firmware/captura_octo_m8_vuelo_guia
```

Grabarlo desde la Pi:

```bash
~/DSM/firmware/cargar_vuelo_guia_desde_pi.sh
```

Analizar un vuelo:

```bash
python3 ~/DSM/herramientas/comparar_vuelo_isaac.py ~/DSM/registros_dron/vuelo_AAAAMMDD_HHMMSS.csv --masa 3.6
```

## 4. Problemas conocidos

- **«IMU=0» o Calibrar rechazado:** pulsen EN en el NodeMCU del dron.
- **El dron no manda datos después de EN o de grabar:** en `dmesg` aparece `urb stopped`. Reinicien el programa de la Pi (sección 2).
- **Sin hb de la Pi, el colectivo no pasa de 1400 µs.** Si el hb se corta 1,5 s, el dron baja solo.
- **Cortes del firmware:** 12° de inclinación, 20° de yaw y 35 °/s de giro en yaw.

## 5. Pendiente

- Probar el v8 en vuelo. Si oscila rápido, bajen `rateRequestLimitVuelo` (60 dps) o `kiAngleVuelo` (2,0).
- Pasar peso hacia M6 para corregir el desbalance (unos 12 mm hacia M8).
- Revisar el láser: siempre marca 8191.
- Conectar el myRIO por UART (ver `UART_MYRIO.md`). En `salida.log` debe aparecer `[com] myRIO UART: N -> OK,N`.
- La hora de la Pi está mal desde el reinicio. Solo afecta los nombres de los archivos.
