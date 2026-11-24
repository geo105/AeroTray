#!/usr/bin/env python3
"""Consola y registro completo. Nunca calibra, arma ni arranca automaticamente."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty
from datetime import datetime
from zoneinfo import ZoneInfo

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(HERE.parent))
from ensayo_hover.protocolo import FIELDS, MODES, Parser, row

ALLOWED = {"cal", "vista", "estado", "imu_reiniciar", "armar", "arrancar", "iniciar", "apagar", "parar",
           "mantener", "subir", "bajar", "rampa", "rampa1400", "hover", "marca", "log"}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Recorder:
    def __init__(self, directory, port):
        directory.mkdir(parents=True, exist_ok=False)
        self.path = directory
        self.raw = open(directory/"serial.bin", "wb")
        self.csvfile = open(directory/"telemetria.csv", "w", newline="")
        self.csv = csv.writer(self.csvfile)
        self.csv.writerow(FIELDS)
        self.events = open(directory/"eventos.jsonl", "w")
        files = [ROOT/"firmware/ensayo_hover_1600/ensayo_hover_1600.ino",
                 ROOT/"firmware/ensayo_hover_1600/PoliticaEnsayo.h", HERE/"protocolo.py",
                 HERE/"consola.py", HERE/"rviz.py"]
        self.meta = {"inicio": datetime.now(ZoneInfo("America/Lima")).isoformat(), "puerto": port,
                     "baud": 460800, "protocolos_admitidos": [1,2], "limite_motor_us": 1600, "corte_tilt_deg": 15,
                     "proteccion_accel_rev_2": {"tau_ms":20,"persistencia_ms":40,"limites_filtrados_g":[.55,1.55],
                                                "limites_inmediatos_raw_g":[.20,1.90]},
                     "heartbeat_ms": 500, "duracion_max_ms": 90000, "referencia_grados": [0, 0],
                     "frame": "+X=M8 +Y=M7 +Z arriba", "imu": "brazo M8", "tof": "brazo M1 desactivado",
                     "altura_valida": False, "adc": "crudo, sin calibracion de voltaje",
                     "ganancias_angulo_P": [4.62, 4.65], "ganancias_angulo_D": [.12, .12],
                     "ganancia_angulo_I": .8, "ganancias_tasa_P": [.848, .842, .378],
                     "ganancias_tasa_I": [.100, .100, .288],
                     "curva_empuje": "modelo provisional, NO medicion de fuerza",
                     "archivos_sha256": {str(p.relative_to(ROOT)): sha(p) for p in files}}
        self.last_flush = time.monotonic()
        self.save_meta()

    def save_meta(self):
        (self.path/"sesion.json").write_text(json.dumps(self.meta, indent=2, ensure_ascii=False)+"\n")

    def event(self, kind, **data):
        self.events.write(json.dumps({"evento": kind, "host_monotonic_s": time.monotonic(), **data}, ensure_ascii=False)+"\n")

    def flush(self, force=False):
        now = time.monotonic()
        if force or now-self.last_flush >= .5:
            for f in (self.raw, self.csvfile, self.events):
                f.flush()
            self.last_flush = now

    def close(self, stats):
        self.meta.update(stats)
        self.meta["fin"] = datetime.now(ZoneInfo("America/Lima")).isoformat()
        self.flush(True)
        for f in (self.raw, self.csvfile, self.events):
            os.fsync(f.fileno())
            f.close()
        self.save_meta()


class Console:
    def __init__(self, args, ser, recorder):
        self.args, self.ser, self.rec = args, ser, recorder
        self.parser = Parser()
        self.latest = None
        self.last_received = self.last_hb = self.last_display = self.last_status = 0.
        self.previous_seq = self.previous_us = None
        self.count = self.gaps = self.reboots = 0
        self.text = ""
        self.running = True
        self.display = None
        self.last_mode = None
        self.stale_stopped = False
        self.stable_since = None
        self.initial_stop_sent = False
        self.auto_state = 0 if getattr(args, "calibrar_vista", False) else 9
        self.auto_deadline = 0.
        if args.rviz:
            from ensayo_hover.rviz import Display, rclpy
            rclpy.init()
            self.display = Display()

    def send(self, command):
        self.ser.write(b"!\n" if command == "!" else command.encode("ascii")+b"\n")

    def emergency(self, reason):
        # Orden primero: no depende de que el archivo de eventos pueda escribirse.
        try:
            self.send("!")
        except Exception:
            pass  # el watchdog de comunicacion del firmware sigue siendo independiente
        self.text = ""
        self.auto_state = 9
        self.rec.event("parada_pc", motivo=reason)
        print("\nAPAGAR solicitado: desarmar y 1000 us. Comprobar estado OFF.", flush=True)

    def command(self, command):
        if command in ("salir", "exit"):
            self.emergency("salir")
            self.running = False
            return
        if command in ("!", "apagar", "parar"):
            self.emergency(command)
            return
        if command not in ALLOWED:
            print("Orden desconocida. armar/arrancar/apagar/mantener/subir/bajar/rampa1400/rampa/hover/estado", flush=True)
            return
        now = time.monotonic()
        diagnostic = command in ("estado", "imu_reiniciar")
        if not diagnostic and (not self.latest or now-self.last_received >= .25
                               or self.stable_since is None or now-self.stable_since < .5):
            print("Sin telemetria DSMH reciente y estable; orden bloqueada. APAGAR sigue disponible.", flush=True)
            return
        self.rec.event("comando", comando=command, muestra=row(self.latest, now) if self.latest else None)
        self.rec.flush(True)
        self.send(command)

    def input(self):
        if not select.select([sys.stdin], [], [], 0)[0]:
            return
        c = os.read(sys.stdin.fileno(), 1)
        if not c:
            self.emergency("entrada_cerrada")
            self.running = False
            return
        if c in (b"!", b" ", b"\x1b"):
            self.emergency("tecla_inmediata")
        elif c in (b"\r", b"\n"):
            command, self.text = self.text.strip(), ""
            if command:
                print()
                self.command(command)
        elif c in (b"\x7f", b"\x08"):
            self.text = self.text[:-1]
            sys.stdout.write("\b \b"); sys.stdout.flush()
        elif 32 < c[0] < 127 and len(self.text) < 40:
            self.text += c.decode("ascii")
            sys.stdout.write(c.decode("ascii")); sys.stdout.flush()

    def receive(self):
        data = self.ser.read(8192)
        if not data:
            return
        self.rec.raw.write(data)
        for kind, v in self.parser.feed(data):
            now = time.monotonic()
            if kind == "text":
                if v:
                    self.rec.event("texto_esp32", texto=v)
                    if "# ENSAYO: INICIO DESARMADO" in v or "# ENSAYO_HOVER_V1 protocolo=" in v:
                        if self.previous_seq is not None:
                            self.reboots += 1
                            self.rec.event("reinicio_anunciado_esp32")
                        self.previous_seq = self.previous_us = None
                        if self.auto_state == 1:
                            self.emergency("reinicio_durante_calibracion")
                        self.latest = None
                        self.last_received = 0.
                        self.stable_since = None
                        self.initial_stop_sent = False
                        self.last_mode = None
                        self.text = ""
                    if v.startswith("#"):
                        print("\n"+v, flush=True)
                        if self.display:
                            self.display.ultimo_evento = v[:100]
                continue
            if not self.initial_stop_sent:
                self.send("!")
                self.rec.event("parada_inicial_al_conectar")
                self.initial_stop_sent = True
            if not self.last_received or now-self.last_received >= .25:
                self.stable_since = now
            if self.previous_seq is not None:
                delta = (v[3]-self.previous_seq) & 0xffffffff
                if delta > 0x7fffffff or delta == 0:
                    self.reboots += 1
                    self.rec.event("reinicio_o_secuencia_invalida", previo=self.previous_seq, nuevo=v[3])
                    self.emergency("reinicio_esp32")
                elif delta > 1:
                    self.gaps += delta-1
                    self.rec.event("hueco_secuencia", perdidas=delta-1, previo=self.previous_seq, nuevo=v[3])
            self.previous_seq, self.previous_us = v[3], v[5]
            self.latest, self.last_received = v, now
            if self.count==0 or self.rec.meta.get("protocolo_observado")!=v[2]:
                self.rec.meta["protocolo_observado"]=v[2]
                self.rec.meta["revision_accel_observada"]=v[47] if v[2]==2 else 0
                self.rec.save_meta()
            self.count += 1
            self.rec.csv.writerow(row(v, now))
            if v[6] != self.last_mode:
                self.rec.event("modo", modo=MODES[v[6]], motivo_corte=v[42])
                print(f"\nESTADO {MODES[v[6]]}; PWM: {list(v[24:32])}; corte={v[42]}", flush=True)
                self.last_mode = v[6]
            if self.display and now-self.last_display >= .1:
                self.display.update(v)
                self.last_display = now

    def auto_preview(self, now, fresh):
        if self.auto_state == 9:
            return
        v = self.latest
        if self.auto_state == 0 and fresh and v[4] >= 4500 and v[6] == 0 \
                and self.stable_since is not None and now-self.stable_since >= .5:
            print("\n--calibrar-vista: calibrando 5 s; mantener inmovil y horizontal.", flush=True)
            self.command("cal")
            self.auto_state, self.auto_deadline = 1, now+10
        elif self.auto_state == 1 and fresh and v[7]&1 and not v[7]&2 and v[6] == 0:
            self.command("vista")
            self.auto_state = 9
        elif self.auto_state == 1 and now >= self.auto_deadline:
            self.emergency("calibracion_no_confirmada")
            print("Calibracion no confirmada. Revisar IMU y nivel; repetir cal manualmente.", flush=True)

    def run(self):
        started = time.monotonic()
        print(f"Registro: {self.rec.path}\nNO hay arranque automatico. cal (quieto/nivelado), vista; armar + arrancar.\n"
              "rampa1400 = objetivo 1400; rampa = hasta 1600 a 5 us/s; subir/bajar = +/-10; mantener; hover = marca manual.\n"
              "APAGADO: ! o ESPACIO sin Enter; apagar + Enter. salir tambien apaga.\n"
              "ALTURA INVALIDA. Para emergencia sin USB usar el corte fisico de ambas baterias.", flush=True)
        while self.running:
            self.receive()
            self.rec.flush()
            now = time.monotonic()
            fresh = self.latest is not None and now-self.last_received < .25
            if fresh:
                self.stale_stopped = False
                if now-self.last_hb >= .1:
                    self.send("hb")
                    self.last_hb = now
            elif self.latest is not None and not self.stale_stopped:
                self.emergency("telemetria_caducada")
                self.stale_stopped = True
                if self.display:
                    self.display.update(self.latest, stale=True)
            self.auto_preview(now, fresh)
            if not self.args.sin_consola:
                self.input()
            if self.latest and now-self.last_status >= 1 and not self.text:
                v = self.latest
                tilt = math.degrees(math.acos(max(-1., min(1., math.cos(math.radians(v[10]))*math.cos(math.radians(v[11]))))))
                print(f"[{MODES[v[6]]}{'' if fresh else ' SIN DATOS'}] cal={bool(v[7]&1)} tilt={tilt:.2f}/15 grados "
                      f"colectivo={v[8]} PWM={min(v[24:32])}..{max(v[24:32])} "
                      f"seq_perdidas={self.gaps} CRC={self.parser.errors} TXdrop={v[39]}"
                      + (f" accel={v[43]:.2f}g filt={v[44]:.2f}g pendiente={v[45]/1000:.0f}ms tipo={v[46]}" if v[2]==2 else ""), flush=True)
                self.last_status = now
            if self.args.monitor_segundos and now-started >= self.args.monitor_segundos:
                return

    def finish(self):
        # Tambien al Ctrl+C, excepcion de disco, fallo ROS o puerto desconectado.
        try:
            self.send("!")
            # Recoger confirmacion OFF si el enlace aun responde.
            deadline = time.monotonic()+.3
            while time.monotonic() < deadline:
                self.receive()
        finally:
            self.ser.close()
            stats = {"muestras": self.count, "huecos_secuencia": self.gaps, "crc_o_formato_invalidos": self.parser.errors,
                     "reinicios": self.reboots, "tx_descartadas_ultimo": self.latest[39] if self.latest else None,
                     "ultimo_modo": MODES[self.latest[6]] if self.latest else None,
                     "ultimo_pwm_us": list(self.latest[24:32]) if self.latest else None}
            self.rec.close(stats)
            if self.display:
                self.display.destroy_node()
                import rclpy
                rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--puerto", default="/dev/ttyUSB0")
    ap.add_argument("--rviz", action="store_true")
    ap.add_argument("--calibrar-vista", action="store_true",
                    help="calibrar y abrir VISTA: solo con motores sin potencia y dron quieto/horizontal; nunca arma")
    ap.add_argument("--sin-consola", action="store_true", help="solo monitor; nunca arma ni inicia")
    ap.add_argument("--monitor-segundos", type=float, default=0)
    ap.add_argument("--registro", type=Path)
    args = ap.parse_args()
    import serial
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout, ser.write_timeout, ser.exclusive = args.puerto, 460800, .005, .05, True
    ser.dtr = ser.rts = False
    console = rec = None
    previous = None
    try:
        ser.open()
        directory = args.registro or ROOT/"registros/ensayo_hover"/datetime.now(ZoneInfo("America/Lima")).strftime("%Y%m%d_%H%M%S_%f")
        rec = Recorder(directory, args.puerto)
        console = Console(args, ser, rec)
        if not args.sin_consola:
            if not sys.stdin.isatty():
                raise RuntimeError("La consola necesita una terminal visible. Usa el lanzador probar_hover_1600.sh.")
            previous = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)
        def shutdown(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, shutdown)
        console.run()
    except KeyboardInterrupt:
        pass
    finally:
        if previous:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, previous)
        if console:
            console.finish()
        else:
            if ser.is_open:
                try:
                    ser.write(b"!\n")
                finally:
                    ser.close()
            if rec:
                rec.close({"error": "fallo de inicializacion"})


if __name__ == "__main__":
    main()
