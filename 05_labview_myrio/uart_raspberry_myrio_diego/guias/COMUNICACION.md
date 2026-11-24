# Comunicacion Raspberry Pi 3 con ESP32 y myRIO

## Puertos

| Equipo | Enlace de la Raspberry | Parametros |
|---|---|---|
| ESP32 UAV | Adaptador USB CP2102, detectado automaticamente | 115200 8N1 |
| ESP32 Ball and Plate | Adaptador USB CH340 | 2000000 8N1 |
| myRIO 1900 | UART TTL GPIO, configurada en `config_pi.json` | 9600 8N1, sin control de flujo |

El myRIO usa `myrio_uart: "/dev/serial0"`. No es un adaptador USB ni se busca en `/dev/serial/by-id`.

## Protocolo UART del myRIO

La UART utiliza ASCII, coma como separador decimal y LF (`\n`) como terminador. No se envian textos `OK` ni `ERR` al myRIO.

| Direccion | Formato | Ejemplo |
|---|---|---|
| myRIO -> Pi | `cmd_1,cmd_2\n` | `4,10\n` |
| Pi -> myRIO | `x_mm,y_mm,roll_deg,pitch_deg,uav_activo,bp_activo\n` | `10.000,8.000,40.000,50.000,1,1\n` |

Cada campo RX debe ser un entero entre 1 y 11. Los codigos 1-4 van al UAV y los codigos 5-11 al Ball and Plate. Por ejemplo, `4,10\n` detiene el UAV y ejecuta nivelar placa. La Pi descarta una trama que no tenga exactamente dos campos, tenga valores no enteros o no termine en LF.

La telemetria se transmite a 20 Hz. Si la camara no encuentra la bola, X e Y son `0.000`; el VI debe usar los indicadores de actividad para interpretar el estado. Los clientes TCP conservan las lineas de diagnostico `U,...` y `B,...` de la version anterior.

## Prioridad de mandos

La pagina web y TCP siguen disponibles. Cuando llega una trama valida desde el myRIO, la Pi adquiere el turno de comandos y bloquea una orden web/TCP concurrente durante 250 ms; por tanto, la orden del panel myRIO se aplica primero. La pagina web conserva las respuestas `OK,...` y `ERR,...`.

## Archivos

- `myrio_uart.py`: parser, formateador de telemetria y puerto UART fijo con reconexion.
- `comunicacion.py`: arbitraje de comandos, prioridad myRIO y reenvio de telemetria.
- `seguidor.py`: obtiene la telemetria real de camara/UAV e inicializa el enlace.
- `test_myrio_uart.py`: pruebas sin hardware del formato y validacion.
- `GUIA_UART_MYRIO.md`: configuracion, cableado y prueba segura.
