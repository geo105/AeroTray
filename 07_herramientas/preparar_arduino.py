"""Instala Arduino CLI oficial y el core ESP32 en carpetas de este proyecto."""
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import urllib.request

ROOT = Path(__file__).resolve().parent.parent
TOOLS = ROOT / "herramientas"
CLI = TOOLS / "arduino-cli" / "arduino-cli"
CONFIG = TOOLS / "arduino-cli.yaml"


def get(url):
    request = urllib.request.Request(url, headers={"User-Agent": "DSM-Arduino-setup"})
    with urllib.request.urlopen(request, timeout=180) as response:
        return response.read()


if not CLI.exists():
    release = json.loads(get("https://api.github.com/repos/arduino/arduino-cli/releases/latest"))
    assets = [a for a in release["assets"] if a["name"].endswith("_Linux_64bit.tar.gz")]
    if len(assets) != 1:
        raise SystemExit("No se encontro un binario oficial unico para Linux x86_64")
    asset = assets[0]
    url = asset["browser_download_url"]
    if not url.startswith("https://github.com/arduino/arduino-cli/releases/download/"):
        raise SystemExit("URL inesperada")
    archive = get(url)
    digest = "sha256:" + hashlib.sha256(archive).hexdigest()
    if asset.get("digest") != digest:
        raise SystemExit("El SHA256 del binario no coincide o no esta publicado")
    with tarfile.open(fileobj=io.BytesIO(archive), mode="r:gz") as tar:
        member = tar.getmember("arduino-cli")
        if not member.isfile():
            raise SystemExit("El binario del archivo no es un archivo regular")
        content = tar.extractfile(member).read()
    CLI.parent.mkdir(parents=True, exist_ok=True)
    CLI.write_bytes(content)
    CLI.chmod(0o755)
    print("Arduino CLI verificado:", release["tag_name"], flush=True)

if not CONFIG.exists():
    CONFIG.write_text(
        "board_manager:\n"
        "  additional_urls:\n"
        "    - https://espressif.github.io/arduino-esp32/package_esp32_index.json\n"
        "directories:\n"
        f"  data: {TOOLS / 'arduino-data'}\n"
        f"  downloads: {TOOLS / 'arduino-downloads'}\n"
        f"  user: {ROOT / 'firmware' / 'arduino-user'}\n"
    )

for arguments in [["version"], ["core", "update-index"], ["core", "install", "esp32:esp32@3.3.12"], ["core", "list"]]:
    subprocess.run([str(CLI), "--config-file", str(CONFIG), *arguments], check=True)
