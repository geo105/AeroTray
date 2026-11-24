"""UART TTL entre Raspberry Pi 3 y myRIO.

Protocolo de produccion (9600 8N1, ASCII):
  myRIO -> Pi: cmd_1,cmd_2\n
  Pi -> myRIO: pos_x_mm,pos_y_mm,roll_deg,pitch_deg,uav_activo,bp_activo\n
No contiene comandos de los ESP32 ni logica de control. Enlace en
``comunicacion.py`` decide como aplicar los dos comandos recibidos.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Optional

try:
    import serial
except ModuleNotFoundError:  # permite probar el parser en una PC sin pyserial
    serial = None

BAUDIOS_POR_DEFECTO = 9600
MAX_LINEA = 96
COMANDOS_UAV = frozenset((1, 2, 3, 4))
COMANDOS_BP = frozenset((5, 6, 7, 8, 9, 10, 11))


class TramaInvalida(ValueError):
    """La linea recibida no corresponde al protocolo del myRIO."""


@dataclass(frozen=True)
class ComandosMyRIO:
    """Los dos codigos numericos enviados en una unica linea terminada en LF."""

    primero: int
    segundo: int

    @property
    def codigos(self) -> tuple[int, int]:
        return self.primero, self.segundo


def _codigo(texto: str) -> int:
    """Acepta 4 o 4.000, pero nunca NaN, infinito ni un decimal no entero."""
    try:
        numero = float(texto)
    except ValueError as exc:
        raise TramaInvalida("campo no numerico") from exc
    if not math.isfinite(numero) or not numero.is_integer():
        raise TramaInvalida("los comandos deben ser enteros finitos")
    codigo = int(numero)
    if codigo not in COMANDOS_UAV | COMANDOS_BP:
        raise TramaInvalida("comando fuera de 1..11")
    return codigo


def parsear_comandos(linea: str) -> ComandosMyRIO:
    """Valida exactamente dos codigos separados por una coma.

    La ruta se determina por el codigo: 1..4 se envia al UAV y 5..11 a
    Ball and Plate. Asi la trama documentada ``4,10\n`` ordena parar el UAV y
    nivelar la placa, independientemente de que el VI formatee los DBL como
    ``4.000,10.000``.
    """
    texto = linea.strip()
    if not texto or len(texto) > MAX_LINEA:
        raise TramaInvalida("longitud de trama invalida")
    campos = texto.split(",")
    if len(campos) != 2 or any(not campo.strip() for campo in campos):
        raise TramaInvalida("se esperaban exactamente dos campos separados por coma")
    return ComandosMyRIO(_codigo(campos[0].strip()), _codigo(campos[1].strip()))


def formato_telemetria(pos_x_mm: Optional[float], pos_y_mm: Optional[float],
                       roll_deg: float, pitch_deg: float, uav_activo: object,
                       bp_activo: object) -> bytes:
    """Construye la unica trama que la Pi envia al myRIO.

    Si la camara no ve la bola, X e Y se emiten como 0.000; el indicador de
    estado B&P permite al VI distinguir una placa sin PID de una activa.
    """
    valores = (0.0 if pos_x_mm is None else float(pos_x_mm),
               0.0 if pos_y_mm is None else float(pos_y_mm),
               float(roll_deg), float(pitch_deg))
    if not all(math.isfinite(valor) for valor in valores):
        raise ValueError("telemetria no finita")
    return (f"{valores[0]:.3f},{valores[1]:.3f},{valores[2]:.3f},{valores[3]:.3f},"
            f"{int(bool(uav_activo))},{int(bool(bp_activo))}\n").encode("ascii")


class MyRIOUART:
    """Puerto UART fijo que se reconecta y entrega solamente lineas completas.

    ``/dev/serial0`` es la UART TTL de los GPIO de la Raspberry, no un
    adaptador USB. El hilo de lectura no escribe confirmaciones: esa direccion
    queda reservada para telemetria de seis campos.
    """

    def __init__(self, puerto: str, baudios: int,
                 al_recibir: Callable[[str], None]) -> None:
        if serial is None:
            raise RuntimeError("falta pyserial; instalar con: python3 -m pip install pyserial")
        self.puerto = puerto
        self.baudios = int(baudios)
        self.al_recibir = al_recibir
        self._serie: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._detener = threading.Event()
        threading.Thread(target=self._bucle, name="uart-myrio", daemon=True).start()

    @property
    def conectado(self) -> bool:
        with self._lock:
            return self._serie is not None and self._serie.is_open

    @property
    def dispositivo(self) -> str:
        return self.puerto if self.conectado else "no conectado"

    def _abrir(self) -> bool:
        try:
            serie = serial.Serial(
                port=self.puerto,
                baudrate=self.baudios,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.10,
                write_timeout=0.50,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
        except (serial.SerialException, OSError) as error:
            if str(error) != getattr(self, "_ultimo_error", None):   # avisa una vez, no cada segundo
                print(f"[myrio] no se puede abrir {self.puerto}: {error}", flush=True)
                self._ultimo_error = str(error)
            return False
        self._ultimo_error = None
        with self._lock:
            self._serie = serie
        print(f"[myrio] UART conectada en {self.puerto}, {self.baudios} 8N1", flush=True)
        return True

    def _cerrar(self) -> None:
        with self._lock:
            serie, self._serie = self._serie, None
        if serie is not None:
            try:
                serie.close()
            except (serial.SerialException, OSError):
                pass

    def escribir(self, datos: bytes) -> bool:
        if not datos.endswith(b"\n"):
            raise ValueError("toda trama UART debe terminar en LF")
        with self._lock:
            serie = self._serie
            if serie is None:
                return False
            try:
                serie.write(datos)
                return True
            except (serial.SerialException, OSError):
                pass
        self._cerrar()
        return False

    def detener(self) -> None:
        self._detener.set()
        self._cerrar()

    def _bucle(self) -> None:
        resto = b""
        while not self._detener.is_set():
            if not self.conectado:
                if not self._abrir():
                    self._detener.wait(1.0)
                    continue
                resto = b""
            with self._lock:
                serie = self._serie
                try:
                    datos = serie.read(256) if serie is not None else b""
                except (serial.SerialException, OSError):
                    datos = None
            if datos is None:
                print("[myrio] UART desconectada; reintentando", flush=True)
                self._cerrar()
                continue
            if not datos:
                continue
            resto += datos
            lineas = resto.split(b"\n")
            resto = lineas.pop()[-MAX_LINEA:]
            for bruta in lineas:
                try:
                    self.al_recibir(bruta.rstrip(b"\r").decode("ascii"))
                except Exception as error:  # no perder la UART por una trama invalida
                    print(f"[myrio] error procesando trama: {error}", flush=True)
