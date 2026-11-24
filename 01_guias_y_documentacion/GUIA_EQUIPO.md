# Robot integrado DSM – Grupo 1: qué hicimos y dónde está cada cosa

Esta guía explica, para todo el equipo, cómo está armado el robot, en qué estado está cada parte, dónde está cada archivo y cómo conectarse a la Raspberry Pi.

## 1. El robot en una frase

Es un solo robot con tres partes, una sobre otra:

| Parte | Hardware | Quién la controla |
|---|---|---|
| **Base móvil (carrito)** | Motores TETRIX con NiMH de 12 V | myRIO (LabVIEW), alimentado con un power bank de 5 a 12 V |
| **Dron octocóptero** sobre una rótula con guía vertical | 8 motores con ESC, 2 LiPo 3S (una por cada rama de 4 ESC, con fusible de 30 A y relé), IMU MPU6050, láser VL53L0X | NodeMCU-32S (ESP32) |
| **Ball and plate** encima del dron | Bandeja con 2 servos y cámara PS3 Eye | ESP32-S3 para los servos; Raspberry Pi 3B+ para la cámara, el PID y la web |

La **Raspberry Pi** es el centro de todo:
- mira la pelota con la cámara y manda los pulsos de los servos al ESP32-S3;
- habla con el dron por USB;
- habla con el myRIO por la UART de los GPIO;
- muestra las páginas web de control.

### Alimentación

| Qué | De dónde se alimenta |
|---|---|
| Pi | Fuente propia: pack 3S de 18650 con BMS y regulador XL4015 a 5,2 V |
| Servos y relés | Fuente de laboratorio, porque se quemó su BMS |
| Motores del dron | 2 LiPo 3S |
| myRIO | Power bank de 5 a 12 V |
| Carrito | NiMH TETRIX de 12 V |

### Pines (los válidos son los del código; los del PDF están desactualizados)

| Qué | Pines |
|---|---|
| ESC del dron M1–M8 | GPIO 25, 26, 27, 14, 21, 19, 5, 17 |
| I2C del dron (IMU y láser) | SDA 23, SCL 22 |
| Servos de la bandeja | X = GPIO47, Y = GPIO21 (ESP32-S3) |
| Pi → myRIO | TX GPIO14 (pin 8) → MXP pin 10 |
| myRIO → Pi | RX GPIO15 (pin 10) ← MXP pin 14 |
| Tierra Pi ↔ myRIO | Pin 6 ↔ MXP pin 12 |

Geometría del dron:
- el eje +X apunta a M8 y el eje +Y a M7;
- M1–M4 giran en sentido horario y M5–M8 en antihorario.

## 2. Estado actual (20 de noviembre)

| Parte | Estado |
|---|---|
| Ball and plate | **Funciona.** PID en la Pi (kp 1,5, ki 0,8, kd 4,5; neutro de los servos 1386/1392 µs). Cámara a 187 fps. |
| Carrito (myRIO) | **Funciona solo.** Hace giros, cambio de dirección y subida de rampa hacia adelante y en reversa (videos en `LAB6/`). |
| Pi ↔ myRIO por UART | **Implementado y probado en la PC; falta probarlo con el myRIO.** La UART de la Pi ya está activada y conectada (`/dev/serial0`, 9600 8N1). Falta cablear el myRIO y probar las tramas. |
| Dron | **Ya flota en la guía.** Con el firmware v8 flotó unos 20 s entre 1725 y 1775 µs (roll ±1°, pitch ±3°). Ahora está grabado el **v9**, que corrige el corte al aterrizar y sube lento desde 1650 µs. Falta confirmar en un ensayo que el v9 aterriza sin cortar. |
| Isaac Lab | **Solo simulación.** Predijo el hover en ≈1898 µs; en la realidad flota desde ≈1725–1750 µs, porque hay más empuje que el que suponía el modelo. La estabilidad real es mucho peor que la simulada. |
| Robot integrado | **Pendiente.** Orden de las pruebas: dron + ball and plate (`PRUEBA_DRON_BALLPLATE.md`), y luego todo sobre el carrito. |

### Historia del vuelo (por qué hay tantas versiones)

| Versión | Qué pasó | Qué se cambió en la siguiente |
|---|---|---|
| v2 | Cortó por inclinación a 1625 µs: el acelerómetro vibraba. | Filtro del acelerómetro a 10 Hz (v3). |
| v3 | Cortó por yaw a −20°: el dron sí giraba. | Se activó el control de yaw (v4). |
| v4 | El pitch se fue a 6,5° a 1700 µs. | Integral de ángulo más grande (v5). |
| v5 | Flotó a 1750–1800 µs, pero se fue inclinando despacio (picos de 9°). | Ganancias más suaves (v6). |
| v6 | Cortó a 1700 y luego a 1725 µs por roll. | v7 (ganancias completas) y luego v8. |
| v8 | **Primer vuelo estable: 20 s.** Se encontró la causa real: el pedido de giro estaba recortado a 25 °/s, y eso limitaba el par de corrección a ≈0,37 N·m, menos que el desbalance del dron (~0,42 N·m). En vuelo se abrió a 60 °/s, con más integral. Cortó solo al aterrizar. | v9. |
| v9 | Grabado; falta el ensayo. Flotar sube a 1750 µs, despacio desde 1650 µs, y mantiene el modo vuelo hasta bajar de 1450 µs. | — |

**Pendientes del dron:**
- el centro de masa está unos 12 mm hacia M8: hay que pasar peso hacia M6;
- el láser casi siempre marca 8191 (fuera de rango);
- la Pi tiene la hora mal desde el reinicio, así que los nombres de los archivos de registro salen con la hora equivocada.

## 3. Dónde está cada cosa (laptop de George, carpeta `~/DSM`)

### Lo que corre en la Raspberry

| Ruta | Qué es |
|---|---|
| `pi_control/seguidor.py` | Programa principal de la Pi: cámara, PID de la bandeja, web en el puerto 8080 y enlaces |
| `pi_control/comunicacion.py` | Enlace con el dron por USB (clase `Uav`: órdenes, hb, registro de vuelos) y con el myRIO por UART y TCP (clase `Enlace`) |
| `pi_control/myrio_uart.py` | La UART del myRIO, hecha por Diego |
| `pi_control/pagina_dron.py` y `pi_control/pagina_robot.py` | Las páginas `/dron` y `/robot` |
| `pi_control/config_pi.json` y `pi_control/ajustes.json` | La configuración: esquinas de la placa, ganancias, neutros, UART |
| `pi_control/arrancar.sh` | Arranca `seguidor.py` y lo reinicia si se cae |
| `pi_control/subir_uart_a_pi.sh` | Sube los cambios de la Pi y la reinicia |
| `pi_control/firmware_esp32/` | El binario del ESP32-S3 de la bandeja |

### Firmware del dron

| Ruta | Qué es |
|---|---|
| `firmware/captura_octo_m8_vuelo_guia/` | El firmware de vuelo actual (**v9**). Las versiones anteriores están en la misma carpeta como `.antes_v2` … `.antes_v9`. |
| `firmware/captura_octo_m8_banco_angulo_laser/` | El firmware de banco, con el que empezamos |
| `firmware/bin/` | Los binarios compilados, listos para grabar |
| `firmware/cargar_vuelo_guia_desde_pi.sh` | Graba el dron desde la Pi. Con `--volver` regresa al de banco. |

### Documentos y análisis

| Ruta | Qué es |
|---|---|
| `VUELO_GUIA.md` | Cómo volar en la guía, las predicciones y los riesgos |
| `PRUEBA_DRON_BALLPLATE.md` | La prueba siguiente: dron + bandeja |
| `UART_MYRIO.md` | Cableado y protocolo Pi ↔ myRIO |
| `RESUMEN_TRABAJO.md` | Comandos rápidos |
| `registros_dron/` | Los registros de todos los vuelos (`vuelo_*.csv`) y sus comparaciones con Isaac (`*_comparacion.md` y `.png`) |
| `herramientas/comparar_vuelo_isaac.py` | Analiza un vuelo y lo compara con Isaac Lab |

### Laboratorio 6, simulación y otros

| Ruta | Qué es |
|---|---|
| `LAB6/` | Laboratorio 6 (detalle abajo) |
| `simulacion/` y `herramientas/IsaacLab/` | Isaac Lab y MuJoCo: modelo CAD, PPO y control de referencia (`simulacion/ISAAC_PLAN.md`) |
| `cad/` | Los archivos CAD (STEP) |
| `diego_uart/` | El zip original de Diego. **Solo como referencia:** sus `config_pi.json` y `ajustes.json` son viejos, no los copien. |
| `PROYECTO.md` y `REVISION_*.md` | La historia larga del proyecto, desde octubre |

Contenido de `LAB6/`:
- `Lab6_DSM_Grupo1.pptx`: la presentación;
- `Poster_Lab6_DSM_Grupo1.pptx`: el póster;
- `Video_Lab6_DSM_Grupo1.mp4`: el video;
- `Guion_*.docx`: los guiones;
- `DSM_Lab6_Grupo1_codigos_y_ensayos.zip`: el paquete para compartir;
- `video/`: los scripts que arman el video.

## 4. Conectarse a la Raspberry desde tu terminal

### Requisitos

- Estar en la **misma red que la Pi** (hotspot local).
- La Pi: IP `<IP_PI>` y usuario `<usuario>`.
- Si la IP cambia, búsquenla en la lista de dispositivos del hotspot por su dirección MAC (`<MAC_PI>`), o desde Linux con:

```bash
nmap -sn <subred>/24
```

### Entrar por ssh

Funciona igual en Linux, macOS y Windows (PowerShell ya trae `ssh`):

```bash
ssh <usuario>@<IP_PI>
```

**Opcional:** para no escribir la contraseña cada vez, creen una llave una sola vez desde su máquina (en Windows, `ssh-copy-id` no existe: copien la llave pública a mano a `~/.ssh/authorized_keys` de la Pi):

```bash
ssh-keygen -t ed25519
```

```bash
ssh-copy-id <usuario>@<IP_PI>
```

### Ver lo que pasa

```bash
ssh <usuario>@<IP_PI> tail -f pi_control/salida.log
```

Las páginas web abren desde cualquier navegador de la red:
- `http://<IP_PI>:8080/` (ball and plate);
- `/dron`;
- `/robot`;
- `/estado` (estado en JSON).

### Programar la Pi

**Opción A, recomendada:** editar en su computadora con VS Code.
1. Instalen la extensión «Remote - SSH».
2. Usen «Connect to Host» y escriban `<usuario>@<IP_PI>`.
3. Abran la carpeta `~/pi_control`. Editan directo en la Pi.

**Opción B:** editar en la laptop y subir los archivos.

```bash
scp pi_control/seguidor.py pi_control/comunicacion.py <usuario>@<IP_PI>:pi_control/
```

**Opción C:** editar en la Pi desde la terminal.

```bash
ssh <usuario>@<IP_PI>
```

```bash
nano ~/pi_control/seguidor.py
```

### Reiniciar el programa después de un cambio

Entren primero por ssh. **No pongan** `pkill` en la misma línea del comando `ssh`: mataría su propia sesión.

```bash
cd ~/pi_control
pkill -f seguidor.py
setsid nohup sh ./arrancar.sh >/dev/null 2>&1 </dev/null &
```

```bash
tail -n 20 salida.log
```

Python en la Pi: el entorno virtual es `~/venv`, que es el que usa `arrancar.sh`. Para instalar algo:

```bash
~/venv/bin/pip install <paquete>
```

### Grabar el ESP32 del dron

Se graba siempre desde la Pi, porque el dron está conectado a ella por USB. Antes, **desconecten las LiPo**. Ejecútenlo desde la laptop:

```bash
~/DSM/firmware/cargar_vuelo_guia_desde_pi.sh
```

Si después de grabar o de pulsar EN el dron no manda datos, reinicien el programa de la Pi como arriba.

### Configuración de la UART de la Pi (ya está hecha)

Se hizo con `raspi-config`: Interface Options, Serial Port, *login shell* en No y *hardware serie* en Sí. Si hay que repetirlo:

```bash
ssh -t <usuario>@<IP_PI> sudo raspi-config
```

## 5. Problemas típicos

| Síntoma | Qué hacer |
|---|---|
| «IMU=0» o Calibrar rechazado | Pulsar EN en el NodeMCU del dron |
| El dron no manda datos (en `dmesg` aparece `urb stopped`) | Reiniciar el programa de la Pi |
| El colectivo no pasa de 1400 µs | Falta el hb de la Pi: la página `/dron` no detectó el firmware de vuelo |
| Corte del dron | El firmware corta a 12° de inclinación, 20° de yaw o 35 °/s de giro en yaw. Exportar el log y revisarlo con `comparar_vuelo_isaac.py`. |
| No llegan comandos del myRIO | Revisar el cableado cruzado y la tierra, 9600 8N1, y que las líneas terminen en `\n`. En `salida.log` debe aparecer `[com] myRIO UART: N -> OK,N`. |

## 6. Seguridad (siempre)

- Nunca grabar el dron con las LiPo conectadas.
- En cada vuelo:
  - una persona en `/dron`, con la mano en **Espacio** (parada inmediata);
  - otra mirando el dron;
  - el corte físico (relés o XT60) a mano.
- Hacer intentos cortos y cargar las LiPo entre intentos. Los fusibles son de 30 A.
