# Sistema de comunicación (Raspberry Pi ↔ ESP32 ↔ myRIO)

    Laptop --WiFi--> myRIO --USB (COM D)--> Raspberry Pi --USB--> ESP32_UAV (COM A, NodeMCU del dron, 115200)
                                                          \-USB--> ESP32_BALL AND PLATE (COM B, 2 000 000)

La Pi reconoce cada equipo por el chip de su adaptador USB (`/dev/serial/by-id`):
CP2102 = dron, CH340 (1a86) = ball and plate, FTDI/Prolific = myRIO. Se puede cambiar en
`config_pi.json` con `uav_puerto`, `myrio_puerto` y `myrio_baudios` (115200 por defecto, 8N1).

## Comandos (un número y un salto de línea)

| N | Comando | Va a | Equivale a |
|---|---|---|---|
| 1 | Calibrar | ESP32_UAV | `cal` (5 s quieto y nivelado) |
| 2 | 1er inicio | ESP32_UAV | `arm BANCO_FIJO` (arma; hay 10 s para el 3) |
| 3 | 2do inicio | ESP32_UAV | `iniciar` (**enciende los 8 motores**) |
| 4 | Parar | ESP32_UAV | `parar` |
| 5 | Inicio PID | ball and plate | |
| 6 | Neutro | ball and plate | placa plana |
| 7 | Apagar PID | ball and plate | servos sin señal |
| 8 | Marcar esquinas | Pi | solo en la página web (clic en las 4 esquinas) |
| 9 | Guardar placa vacía | Pi | |
| 10 | Nivelar placa | Pi + ball and plate | |
| 11 | Borrar placa vacía | Pi | |

Respuesta: `OK,<n>` o `ERR,<n>,<motivo>`.

Tres formas de mandarlos, todas hacen lo mismo:
- Página web `http://IP_DE_LA_PI:8080` (botones numerados) o `http://IP:8080/comando?n=5`.
- myRIO por puerto serie: escribir `5\n`.
- TCP al puerto 5005 de la Pi (por ejemplo desde LabVIEW con TCP Open): escribir `5\n`.

## Lecturas continuas (la Pi las manda 20 veces por segundo al myRIO y a cada cliente TCP)

    U,<tiempo_ms>,<roll_grados>,<pitch_grados>,<activo>     dron (activo = motores en rampa o control)
    B,<tiempo_ms>,<x_mm>,<y_mm>,<activo>                    ball and plate (x, y vacíos si no ve la bola)

## Firmware del dron

`~/DSM/firmware/captura_octo_m8_banco_angulo_pi/` = `captura_octo_m8_banco_angulo` (mismo control y cortes)
más los comandos 1–4 y la línea `T,<ms>,<roll>,<pitch>,<activo>` cada 50 ms. Los comandos de texto
siguen funcionando. Cualquier orden desconocida sigue parando los motores.

## Archivos

- `seguidor.py`: programa principal de la Pi (cámara, ball and plate, página, comandos).
- `comunicacion.py`: puertos del dron y del myRIO, servidor TCP, lecturas continuas.
