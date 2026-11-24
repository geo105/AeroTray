"""Carga unicamente el diagnostico de camara tras comprobar su respaldo local."""
import hashlib
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent.parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', default='/dev/ttyUSB0')
parser.add_argument('--backup',type=Path,help='Respaldo actual verificado con JSON y backup_sha256')
args = parser.parse_args()
ESPTOOL = ROOT / "herramientas/arduino-data/packages/esp32/tools/esptool_py/5.3.1/esptool"
BIN = ROOT / "firmware/bin/camara_s3_diagnostico"
BACKUP = args.backup or ROOT / "firmware/backups/s3_region_0_1MiB_antes_diag.bin"
HEADER = ROOT / "firmware/backups/s3_cabecera_original_64k.bin"
FILES = [
    (0x0, BIN / "camara_s3_diagnostico.ino.bootloader.bin"),
    (0x8000, BIN / "camara_s3_diagnostico.ino.partitions.bin"),
    (0xE000, ROOT / "herramientas/arduino-data/packages/esp32/hardware/esp32/3.3.12/tools/partitions/boot_app0.bin"),
    (0x10000, BIN / "camara_s3_diagnostico.ino.bin"),
]

original = BACKUP.read_bytes()
if args.backup:
    meta=json.loads(BACKUP.with_suffix('.json').read_text())
    if len(original)<0x100000 or len(original)%4096 or hashlib.sha256(original).hexdigest()!=meta['backup_sha256']:
        raise SystemExit('El respaldo actual no pasa la verificacion de longitud/hash.')
elif len(original) != 0x100000 or original[:0x10000] != HEADER.read_bytes():
    raise SystemExit("El respaldo no tiene el tamano esperado o no coincide con la primera lectura.")
coverage = []
for offset, file in FILES:
    content = file.read_bytes()
    end = ((offset + len(content) + 0xFFF) // 0x1000) * 0x1000
    if offset % 0x1000 != 0 or not content or end > len(original):
        raise SystemExit("El respaldo no cubre todos los sectores de " + file.name)
    coverage.append({"offset": offset, "bytes": len(content), "sector_end": end, "path": str(file), "sha256": hashlib.sha256(content).hexdigest()})

metadata = {"backup_bytes": len(original), "backup_sha256": hashlib.sha256(original).hexdigest(),
            "scope": f"region [0, {hex(len(original))}); resto de Flash intacto", "write_regions": coverage}
registro=BACKUP.with_suffix('.carga.json')
registro.write_text(json.dumps(metadata,indent=2)+'\n');registro.chmod(0o600)
command = [str(ESPTOOL), "--chip", "esp32s3", "--port", args.port, "--baud", "115200", "--no-stub",
           "write-flash", "--flash-mode", "keep", "--flash-freq", "keep", "--flash-size", "keep"]
for offset, file in FILES:
    command += [hex(offset), str(file)]
print("Respaldo verificado y todos los sectores cubiertos. Iniciando carga del diagnostico.", flush=True)
log = ROOT / "estado_camara/carga_diagnostico.log"
log.parent.mkdir(exist_ok=True)
with log.open("w") as stream:
    result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
print(log.read_text()[-4500:], flush=True)
if result.returncode:
    raise SystemExit(result.returncode)
print("Carga terminada y verificada por esptool. Pendiente comprobar el arranque y la imagen.")
