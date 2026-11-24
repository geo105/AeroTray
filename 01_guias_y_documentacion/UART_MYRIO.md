# Comunicación Raspberry Pi ↔ myRIO por la UART de los GPIO

Se unió el trabajo de Diego (`myrio_uart.py`) al programa actual de la Pi. Se mantiene todo lo demás: el vuelo, el hb, las páginas `/dron` y `/robot`, y ff.

## Qué cambió en `pi_control/`

| Archivo | Cambio |
|---|---|
| `myrio_uart.py` | Nuevo (de Diego). Abre `/dev/serial0` a 9600 8N1 y se reconecta solo. El error de apertura se avisa una sola vez. |
| `comunicacion.py` | La clase `Enlace` ahora usa la UART en lugar del USB. El resto del archivo no cambió (incluye `Uav`). |
| `seguidor.py` | Lee `myrio_uart` y `myrio_baudios` de `config_pi.json`. |
| `config_pi.json` | Solo se agregaron `"myrio_uart": "/dev/serial0"` y `"myrio_baudios": 9600`. |

Las copias de antes quedaron como `*.antes_uart`.

**No usen** `config_pi.json` ni `ajustes.json` del zip de Diego, porque son versiones viejas:
- el orden de las esquinas está mal;
- tienen las ganancias anteriores (kp 1,875);
- el neutro es 1350/1518.

Tampoco usen su `comunicacion.py` ni su `seguidor_actualizado.py`, que no tienen el código de vuelo.

## Protocolo (igual que el VI de LabVIEW)

- **myRIO → Pi:** una línea `cmd1,cmd2\n` con dos códigos.
  - Los códigos 1–4 van al UAV y los 5–11 al ball and plate.
  - Si una trama tiene un código fuera de 1–11, se descarta entera.
  - La Pi no responde OK/ERR por la UART.
- **Pi → myRIO:** cada 50 ms envía `x,y,roll,pitch,uav_activo,bp_activo\n`, con 3 decimales.
  - x e y valen `0.000` si no se ve la pelota.
- **Prioridad:** durante 0,25 s después de una trama del myRIO, la web/TCP recibe `ERR,0,prioridad myRIO activa`.
- **TCP:** el puerto TCP sigue enviando las líneas `U,…` y `B,…` de siempre.

Se probó en la PC con un puerto serie virtual:
- las tramas válidas se ejecutaron;
- la trama `3,99` se descartó;
- la prioridad funcionó;
- la telemetría salió con 6 campos.

## Cableado (3,3 V TTL, cruzado)

| Raspberry Pi | myRIO (MXP) |
|---|---|
| TX GPIO14 (pin 8) | UART.RX (pin 10) |
| RX GPIO15 (pin 10) | UART.TX (pin 14) |
| GND (pin 6) | DGND (pin 12) |

Conecten primero la tierra. **No usen** el UART RS-232 del conector de audio del myRIO, porque tiene otro nivel de voltaje.

## Activar la UART en la Pi (una sola vez, lo hacen ustedes con sudo)

Hoy la Pi no tiene `/dev/serial0`, y la consola de login está ocupando el puerto (`console=serial0` en `cmdline.txt`).

```bash
sudo raspi-config
```

1. Ir a *Interface Options → Serial Port*.
2. En «¿login shell por serie?» elegir **No**.
3. En «¿hardware serie?» elegir **Sí**.
4. Salir y reiniciar.

Si quieren la UART buena (PL011) en vez de la mini UART, agreguen `dtoverlay=disable-bt` a `/boot/firmware/config.txt`. Es opcional: a 9600 baudios la mini UART funciona.

Al volver, comprueben:

```bash
ls -l /dev/serial0
```

El usuario george ya está en el grupo `dialout`.

## Prueba en la Pi

En `salida.log` debe aparecer `[myrio] UART conectada en /dev/serial0, 9600 8N1`.

Sin el myRIO, pueden puentear TX con RX (pin 8 con pin 10) y ver si llega el eco de la telemetría:

```bash
python3 -c "import serial;s=serial.Serial('/dev/serial0',9600,timeout=1);print(s.readline())"
```

Con el myRIO conectado, al pulsar un botón del panel aparece en `salida.log` `[com] myRIO UART: N -> OK,N`.
