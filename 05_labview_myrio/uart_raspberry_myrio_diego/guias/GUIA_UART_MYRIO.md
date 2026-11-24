# Guia breve UART myRIO y Raspberry Pi 3

## Que se agrego

El modulo `myrio_uart.py` abre la UART TTL integrada de Raspberry Pi en `/dev/serial0` y se reconecta si el dispositivo deja de estar disponible. Sus parametros son 9600 baudios, 8 bits, sin paridad, un bit de parada y sin control de flujo. `comunicacion.py` usa ese modulo sin modificar los protocolos USB de los dos ESP32.

El myRIO manda una unica linea ASCII de dos comandos:

```text
comando_1,comando_2\n
4,10\n
```

Ambos campos se validan antes de hacer nada. Los codigos 1-4 se envian al UAV y 5-11 al Ball and Plate; por eso `4,10` equivale a parar el UAV y nivelar la placa. Una trama invalida no genera ninguna accion. La Raspberry no devuelve confirmaciones al myRIO; reserva esa direccion para telemetria:

```text
posicion_x_mm,posicion_y_mm,roll_deg,pitch_deg,uav_activo,bp_activo\n
10.000,8.000,40.000,50.000,1,1\n
```

La telemetria se actualiza cada 50 ms. X e Y se toman de la camara y Roll/Pitch del ESP32 UAV. Si no se detecta bola, X e Y se envian como `0.000` para preservar los seis campos numericos.

## Cableado y sistema operativo

Con ambos equipos apagados, conectar **Raspberry TX GPIO14, pin fisico 8(morado) -> myRIO UART.RX MXP 10**; **Raspberry RX GPIO15, pin fisico 10(blanco) <- myRIO UART.TX MXP 14**; y **Raspberry GND, pin 6 <-> myRIO DGND MXP 12**. Es UART TTL de 3.3 V: no conectar 5 V ni niveles RS-232.

En la Raspberry ejecutar una sola vez `sudo raspi-config`, elegir *Interface Options -> Serial Port*, responder **No** al login shell y **Yes** a habilitar el hardware. Reiniciar. Despues comprobar:

```sh
ls -l /dev/serial0
groups
```

El usuario que corre `seguidor.py` debe pertenecer a `dialout`; si no, ejecutar `sudo usermod -aG dialout $USER` y volver a iniciar sesion. En el myRIO configurar el VI a 9600, 8N1, sin flow control y finalizar cada trama con LF (`0x0A`).

## Implementacion

1. Copiar la carpeta `1_raspberry_pi` a la Raspberry y conservar `config_pi.json` con `myrio_uart` en `/dev/serial0` y `myrio_baudios` en `9600`.
2. Con el entorno que ya usa OpenCV y pyserial, comprobar la logica sin conectar equipos: `python3 -m unittest -v test_myrio_uart.py`.
3. Ejecutar `python3 seguidor.py`. En `salida.log` debe aparecer `UART conectada en /dev/serial0, 9600 8N1`.
4. Para una primera prueba segura, desde el VI enviar `4,6` seguido de LF. La Pi debe reenviar `4` al UAV y `6` al Ball and Plate, sin encender motores ni PID.
5. Verificar que el VI recibe una sola linea de seis campos a 20 Hz. Solo despues probar los demas codigos y con helices despejadas cuando corresponda.

La pagina web y TCP puerto 5005 siguen activos. Las confirmaciones `OK`/`ERR` se mantienen para esas dos vias; la UART del myRIO prioriza una orden que llegue en el mismo instante.
