"""Protocolo DSMH v1/v2. Solo stdlib; apto para analizar sin ROS ni placa."""
import binascii
import math
import struct

MAGIC = b"DSMH"
FRAME = struct.Struct("<IHHIIIBBHf3f3f6h2f8H2f2f4I2H2H")
FRAME_V2 = struct.Struct("<IHHIIIBBHf3f3f6h2f8H2f2f4I2HHffIBBHH")
SCHEMAS = {(128,1): FRAME, (144,2): FRAME_V2}
MODES = ("OFF", "VISTA", "ARMADO", "RAMPA", "CONTROL")
FIELDS = ["host_monotonic_s", "seq", "ms", "us", "modo", "flags", "colectivo_us",
          "base_modelo_no_medido", "roll_deg", "pitch_deg", "yaw_rel_deg",
          "p_dps", "q_dps", "r_dps", "ax_raw", "ay_raw", "az_raw",
          "gx_raw", "gy_raw", "gz_raw", "escala_rp", "escala_yaw",
          *[f"M{i}_us" for i in range(1, 9)], "integral_angulo_roll", "integral_angulo_pitch",
          "integral_tasa_roll", "integral_tasa_pitch", "intervalo_us", "lectura_imu_us",
          "escritura_pwm_us", "tx_descartadas", "adc34_raw", "adc35_raw", "motivo_corte",
          "accel_norma_g", "accel_filtrada_g", "accel_fuera_us", "accel_tipo", "revision_accel"]


def decode(frame):
    if len(frame) < 8 or frame[:4] != MAGIC:
        raise ValueError("longitud/magic")
    schema = SCHEMAS.get(struct.unpack_from("<HH",frame,4))
    if schema is None or len(frame) != schema.size:
        raise ValueError("cabecera")
    v = schema.unpack(frame)
    if binascii.crc_hqx(frame[:-2], 0xffff) != v[-1]:
        raise ValueError("crc")
    if v[6] >= len(MODES) or not all(math.isfinite(v[i]) for i in (*range(9, 16), 22, 23, 32, 33, 34, 35)):
        raise ValueError("datos invalidos")
    if any(not 1000 <= p <= 1600 for p in v[24:32]):
        raise ValueError("PWM fuera de rango")
    if v[2] == 2 and (not all(math.isfinite(v[i]) for i in (43,44)) or v[46]>4 or v[47]!=2):
        raise ValueError("proteccion aceleracion invalida")
    return v


def row(v, host_time):
    extra = list(v[43:48]) if v[2] == 2 else [math.sqrt(sum(n*n for n in v[16:19]))/16384,None,0,0,0]
    return [host_time, *v[3:6], MODES[v[6]], *v[7:43], *extra]


class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data):
        """Produce ('frame', valores) o ('text', str); tolera cortes y resincro."""
        self.buf.extend(data)
        out = []
        while self.buf:
            if self.buf.startswith(MAGIC):
                if len(self.buf) < 8:
                    break
                schema = SCHEMAS.get(struct.unpack_from("<HH", self.buf, 4))
                if schema is None:
                    self.errors += 1
                    del self.buf[0]
                    continue
                if len(self.buf) < schema.size:
                    break
                try:
                    value = decode(bytes(self.buf[:schema.size]))
                except ValueError:
                    self.errors += 1
                    del self.buf[0]
                    continue
                del self.buf[:schema.size]
                out.append(("frame", value))
                continue
            magic = self.buf.find(MAGIC)
            newline = self.buf.find(b"\n")
            if magic > 0 and (newline < 0 or magic < newline):
                out.append(("text", bytes(self.buf[:magic]).decode("utf-8", "replace").strip()))
                del self.buf[:magic]
            elif newline >= 0:
                out.append(("text", bytes(self.buf[:newline]).decode("utf-8", "replace").strip()))
                del self.buf[:newline+1]
            elif len(self.buf) > 4096:
                out.append(("text", bytes(self.buf[:-3]).decode("utf-8", "replace")))
                del self.buf[:-3]
            else:
                break
        return out
