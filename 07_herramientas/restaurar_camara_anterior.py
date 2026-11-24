"""Restaura la region respaldada antes del diagnostico (solo ejecutar si se requiere)."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent.parent
BACKUP = ROOT / "firmware/backups/s3_region_0_1MiB_antes_diag.bin"
ESPTOOL = ROOT / "herramientas/arduino-data/packages/esp32/tools/esptool_py/5.3.1/esptool"

if __name__ == "__main__":
    data = BACKUP.read_bytes()
    metadata = json.loads(BACKUP.with_suffix(".json").read_text())
    if len(data) != 0x100000 or hashlib.sha256(data).hexdigest() != metadata["backup_sha256"]:
        raise SystemExit("El respaldo no pasa la verificacion. No se escribe nada.")
    subprocess.run([
        str(ESPTOOL), "--chip", "esp32s3", "--port", "/dev/ttyUSB0", "--baud", "115200", "--no-stub",
        "write-flash", "--flash-mode", "keep", "--flash-freq", "keep", "--flash-size", "keep", "0x0", str(BACKUP)
    ], check=True)
    print("Region original restaurada. El resto de la Flash no se ha modificado.")
