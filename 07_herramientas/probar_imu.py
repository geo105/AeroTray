#!/usr/bin/env python3
"""Monitor interactivo de IMU / Giroscopio en terminal.

Lee la telemetría del sketch imu_diagnostico_esp32 a 115200 baud y muestra
barras visuales de Roll, Pitch, velocidades angulares (Gx, Gy, Gz) y aceleración.
"""
import argparse
import glob
import json
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("ERROR: pyserial no está instalado. Ejecuta: pip install pyserial")


def barra_ascii(val, vmin=-45.0, vmax=45.0, ancho=26):
    """Genera una barra centrada en 0 con marcador."""
    val_clamped = max(vmin, min(vmax, val))
    frac = (val_clamped - vmin) / (vmax - vmin)
    pos = int(round(frac * (ancho - 1)))
    centro = int(round(0.5 * (ancho - 1)))

    chars = [" "] * ancho
    chars[centro] = "|"
    if pos < centro:
        for i in range(pos, centro):
            chars[i] = "◄"
    elif pos > centro:
        for i in range(centro + 1, pos + 1):
            chars[i] = "►"
    else:
        chars[centro] = "●"

    return "[" + "".join(chars) + "]"


def encontrar_puerto():
    candidatos = glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    return candidatos[0] if candidatos else "/dev/ttyUSB0"


def main():
    p = argparse.ArgumentParser(description="Monitor IMU MPU-6050")
    p.add_argument("puerto", nargs="?", default=None, help="Puerto serie del NodeMCU (ej: /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=115200)
    args = p.parse_args()

    puerto = args.puerto or encontrar_puerto()
    print(f"==> Abriendo {puerto} a {args.baud} baud...")

    try:
        ser = serial.Serial(puerto, args.baud, timeout=0.5)
    except Exception as e:
        sys.exit(f"ERROR: No se pudo abrir {puerto}: {e}\nComprueba la conexion USB y permisos.")

    ser.dtr = False
    ser.rts = False
    time.sleep(0.1)
    ser.reset_input_buffer()

    print("Esperando telemetria del sensor MPU-6050...\n")
    n_muestras = 0

    try:
        while True:
            linea_bytes = ser.readline()
            if not linea_bytes:
                continue
            linea = linea_bytes.decode(errors="replace").strip()
            if not linea:
                continue

            if linea.startswith("#"):
                print(f"\033[90m{linea}\033[0m")
                continue

            if linea.startswith("{") and linea.endswith("}"):
                try:
                    d = json.loads(linea)
                except json.JSONDecodeError:
                    continue

                roll = d.get("roll", 0.0)
                pitch = d.get("pitch", 0.0)
                yaw = d.get("yaw", 0.0)
                gx = d.get("gx", 0.0)
                gy = d.get("gy", 0.0)
                gz = d.get("gz", 0.0)
                ax = d.get("ax", 0.0)
                ay = d.get("ay", 0.0)
                az = d.get("az", 0.0)
                temp = d.get("temp", 0.0)

                b_roll = barra_ascii(roll, -35.0, 35.0)
                b_pitch = barra_ascii(pitch, -35.0, 35.0)

                # Imprimir interfaz en el mismo lugar de la consola (ANSI)
                sys.stdout.write(
                    "\033[H\033[J"  # Limpiar pantalla y subir cursor
                    "===========================================================\n"
                    "        DSM: PRUEBA EN VIVO DE IMU (MPU-6050)\n"
                    f"        Puerto: {puerto}  |  Temp sensor: {temp:.1f} °C\n"
                    "===========================================================\n"
                    f"  INCLINACION (Angulos calculados):\n"
                    f"    Roll  (alabeo):   {roll:+6.2f}°  {b_roll}\n"
                    f"    Pitch (cabeceo):  {pitch:+6.2f}°  {b_pitch}\n"
                    f"    Yaw   (rumbo):    {yaw:+6.2f}°  (integrado)\n\n"
                    f"  GIROSCOPIO (Velocidad angular real):\n"
                    f"    Gx: {gx:+6.1f} °/s   |   Gy: {gy:+6.1f} °/s   |   Gz: {gz:+6.1f} °/s\n\n"
                    f"  ACELEROMETRO (Gravedad/Fuerza g):\n"
                    f"    Ax: {ax:+6.2f} g     |   Ay: {ay:+6.2f} g     |   Az: {az:+6.2f} g\n"
                    "===========================================================\n"
                    "  [Mueve el dron con la mano para verificar los ejes]\n"
                    "  Roll (+) al inclinar a la derecha | Pitch (+) hacia adelante\n"
                    "  Pulsa Ctrl+C para salir\n"
                )
                sys.stdout.flush()
                n_muestras += 1

    except KeyboardInterrupt:
        print("\n\nMonitor finalizado.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
