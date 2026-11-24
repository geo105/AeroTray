"""Prueba autónoma UART Raspberry Pi 3 <-> ESP32-C3/myRIO.

Recibe exactamente dos comandos CSV del UART GPIO y transmite, a 20 Hz, una
trama de seis números formada con el último par válido:

    a,b\n  ->  a,a,a,b,b,b\n
No importa ni ejecuta ``seguidor.py`` ni ``comunicacion.py``. Reutiliza
``MyRIOUART`` y ``parsear_comandos`` de myrio_uart.py para conservar 9600 8N1,
reconexión, lectura por LF y validación de comandos 1..11.
"""
from __future__ import annotations

import argparse
import signal
import threading
import time
from typing import Optional

from myrio_uart import BAUDIOS_POR_DEFECTO, MyRIOUART, TramaInvalida, parsear_comandos

FRECUENCIA_HZ = 20.0


def trama_simulada(a: int, b: int) -> bytes:
    """Forma la telemetría de prueba requerida: a,a,a,b,b,b seguido de LF."""
    return f"{a},{a},{a},{b},{b},{b}\n".encode("ascii")


class SimuladorTelemetria:
    """Conserva el último par UART válido y publica su telemetría repetida."""

    def __init__(self) -> None:
        self._ultimo: Optional[tuple[int, int]] = None
        self._lock = threading.Lock()

    def recibir_comandos(self, linea: str) -> None:
        """Callback ejecutado por el hilo de recepción de MyRIOUART."""
        try:
            comandos = parsear_comandos(linea)
        except TramaInvalida as error:
            print(f"[simulador] trama descartada {linea!r}: {error}", flush=True)
            return

        with self._lock:
            self._ultimo = comandos.codigos
        print(
            f"[simulador] RX {comandos.primero},{comandos.segundo} -> "
            f"TX {comandos.primero},{comandos.primero},{comandos.primero},"
            f"{comandos.segundo},{comandos.segundo},{comandos.segundo}",
            flush=True,
        )

    def ultima_trama(self) -> Optional[bytes]:
        """Devuelve None hasta que haya llegado un par válido."""
        with self._lock:
            if self._ultimo is None:
                return None
            return trama_simulada(*self._ultimo)


def argumentos() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simula la telemetría myRIO con a,a,a,b,b,b a partir de a,b."
    )
    parser.add_argument("--puerto", default="/dev/serial0", help="UART Raspberry (por defecto /dev/serial0)")
    parser.add_argument("--baudios", type=int, default=BAUDIOS_POR_DEFECTO, help="velocidad UART (por defecto 9600)")
    parser.add_argument("--frecuencia", type=float, default=FRECUENCIA_HZ, help="tramas por segundo (por defecto 20)")
    args = parser.parse_args()
    if args.baudios <= 0 or args.frecuencia <= 0:
        parser.error("--baudios y --frecuencia deben ser positivos")
    return args


def main() -> int:
    args = argumentos()
    simulador = SimuladorTelemetria()
    detener = threading.Event()

    def solicitar_salida(_signo: int, _marco: object) -> None:
        detener.set()

    signal.signal(signal.SIGINT, solicitar_salida)
    signal.signal(signal.SIGTERM, solicitar_salida)

    uart = MyRIOUART(args.puerto, args.baudios, simulador.recibir_comandos)
    periodo = 1.0 / args.frecuencia
    print(
        f"[simulador] esperando a,b\\n en {args.puerto}, {args.baudios} 8N1; "
        f"telemetría a {args.frecuencia:g} Hz. Ctrl+C para finalizar.",
        flush=True,
    )

    try:
        while not detener.is_set():
            inicio = time.monotonic()
            trama = simulador.ultima_trama()
            if trama is not None:
                uart.escribir(trama)
            detener.wait(max(0.0, periodo - (time.monotonic() - inicio)))
    finally:
        uart.detener()
        print("[simulador] UART cerrada.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
