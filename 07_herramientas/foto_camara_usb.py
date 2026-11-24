"""Obtiene una foto del diagnostico DSM 0.2/0.3 a 115200, sin cambiar de Wi-Fi."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import serial

parser = argparse.ArgumentParser()
parser.add_argument("--port", default="/dev/ttyUSB0")
parser.add_argument("--wifi", action="store_true", help="Activa la red local opcional del diagnostico")
parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent.parent / "estado_camara" / "primera_foto.jpg")
args = parser.parse_args()

port = serial.Serial(port=None, baudrate=115200, timeout=0.2)
port.dtr = False
port.rts = False
port.port = args.port
try:
    port.open()
    # Da tiempo al arranque si el adaptador provoca un reset al abrirse.
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        port.read(2048)
    port.write(b"ver\n")
    state = None
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if line.startswith("{"):
            try:
                candidate = json.loads(line)
            except json.JSONDecodeError:
                continue
            if candidate.get("version") in ("camara-diag-0.2", "camara-diag-0.3", "camara-diag-0.4", "camara-diag-0.5"):
                state = candidate
                break
    if state is None:
        raise SystemExit("No responde el diagnostico DSM 0.2/0.3 a 115200. Cierra el visor rapido o reinicia la camara. No se pidio ninguna foto.")
    if args.wifi:
        port.write(b"wifi\n")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            line = port.readline().decode("utf-8", errors="replace").strip()
            if line.startswith("# Red:") or line.startswith("# ERROR"):
                print(line, flush=True)
        port.write(b"ver\n")
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            line = port.readline().decode("utf-8", errors="replace").strip()
            if line.startswith("{"):
                try:
                    candidate = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if candidate.get("version") in ("camara-diag-0.2", "camara-diag-0.3", "camara-diag-0.4", "camara-diag-0.5"):
                    state = candidate
                    break
    print("Estado:", json.dumps(state, ensure_ascii=False), flush=True)
    port.write(b"foto\n")
    size = None
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        line = port.readline()
        if line.startswith(b"# FOTO_JPEG "):
            size = int(line.split()[2])
            break
        if line.startswith(b"# ERROR_FOTO"):
            raise SystemExit(line.decode(errors="replace").strip())
    if size is None or not 1 <= size <= 96 * 1024:
        raise SystemExit("La camara no envio una cabecera de foto valida.")
    data = bytearray()
    deadline = time.monotonic() + 15
    while len(data) < size and time.monotonic() < deadline:
        data.extend(port.read(size - len(data)))
    if len(data) != size or data[:2] != b"\xff\xd8" or data[-2:] != b"\xff\xd9":
        raise SystemExit(f"JPEG incompleto: {len(data)} de {size} bytes. No se guardo como imagen valida.")
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (240, 320):
        diagnostic = Path(__file__).resolve().parent.parent / "tmp/camara_recibida_no_qvga.bin"
        diagnostic.write_bytes(data)
        raise SystemExit(f"La imagen no se decodifica como QVGA 320x240: dimensiones={None if image is None else image.shape}. Copia de diagnostico: {diagnostic}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(".partial")
    temporary.write_bytes(data)
    temporary.replace(args.output)
    metadata = {"port": args.port, "bytes": len(data), "width": 320, "height": 240, "state": state}
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n")
    print("Foto valida guardada:", args.output)
finally:
    if port.is_open:
        port.close()
