"""Puerto exclusivo e identificacion por version antes de enviar comandos DSM."""
import json
import time
import serial


class DiagnosticoDSM:
    def __init__(self, puerto, version):
        self.version = version
        self.ser = serial.Serial(port=None, baudrate=115200, timeout=0.05, write_timeout=0.5, exclusive=True)
        self.ser.dtr = self.ser.rts = False
        self.ser.port = puerto
        try:
            self.ser.open()
            self.estado = None
            time.sleep(1.5)
            self.enviar('ver')
            fin = time.monotonic()+5
            while time.monotonic()<fin:
                r = self.leer()
                if r is not None:
                    self.estado = r
                    return
            raise RuntimeError(f'No responde {version}. No se enviaron ordenes de armado ni prueba.')
        except Exception:
            self.ser.close()
            raise

    def enviar(self, texto):
        self.ser.write((texto+'\n').encode())

    def leer(self):
        line = self.ser.readline()
        if not line.startswith(b'{'):
            if line.startswith(b'# ERROR'):
                print(line.decode(errors='replace').strip(), flush=True)
            return None
        try:
            r = json.loads(line)
        except (ValueError, UnicodeDecodeError):
            return None
        return r if r.get('version')==self.version else None

    def close(self):
        self.ser.close()
