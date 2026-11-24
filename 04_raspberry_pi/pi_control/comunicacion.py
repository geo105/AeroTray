"""Sistema de comunicacion de la Raspberry Pi (lo usa seguidor.py).

    Laptop --WiFi--> myRIO --USB (COM D)--> Raspberry Pi --USB--> ESP32_UAV (COM A, dron)
                                                          \\-USB--> ESP32_BALL AND PLATE (COM B)

Comandos (un numero por linea, desde la pagina web, el myRIO por serie o cualquier equipo por TCP 5005):
    1 Calibrar   2 1er inicio (armar)   3 2do inicio (motores)   4 Parar              -> ESP32_UAV
    5 Inicio PID   6 Neutro   7 Apagar PID   8 Marcar esquinas   9 Guardar placa vacia
    10 Nivelar placa   11 Borrar placa vacia                                           -> ball and plate
Respuesta a cada comando:  OK,<n>   o   ERR,<n>,<motivo>
Lecturas que la Pi manda sin parar al myRIO (20 veces por segundo; las del dron se actualizan
10 veces por segundo y solo mientras el dron esta en VISTA, ARMADO, RAMPA o CONTROL):
    U,<tiempo_ms>,<roll_grados>,<pitch_grados>,<activo>      dron
    B,<tiempo_ms>,<x_mm>,<y_mm>,<activo>                     ball and plate (x,y vacios si no ve la bola)
"""
import glob
import os
import socket
import threading
import time
from collections import deque

import serial

from myrio_uart import MyRIOUART, TramaInvalida, formato_telemetria, parsear_comandos

NOMBRES = {1: "Calibrar", 2: "1er inicio", 3: "2do inicio", 4: "Parar", 5: "Inicio PID", 6: "Neutro",
           7: "Apagar PID", 8: "Marcar esquinas", 9: "Guardar placa vacia", 10: "Nivelar placa",
           11: "Borrar placa vacia"}
PUERTO_TCP = 5005


def puertos_serie():
    """[(dispositivo, identificacion)] de los adaptadores USB-serie conectados. La identificacion es el
    nombre de /dev/serial/by-id (dice el chip: CP2102 = NodeMCU del dron, 1a86 = CH340 del ball and plate)."""
    vistos = {}
    for enlace in sorted(glob.glob("/dev/serial/by-id/*")):
        vistos[os.path.realpath(enlace)] = os.path.basename(enlace)
    for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        vistos.setdefault(dev, dev)
    return sorted(vistos.items())


def coincide(identificacion, dev, patrones):
    return any(p and (p.lower() in identificacion.lower() or p == dev) for p in patrones)


class PuertoSerie:
    """Puerto serie elegido por el chip del adaptador, que se vuelve a buscar solo si se desconecta."""

    def __init__(self, nombre, patrones, baudios, al_recibir):
        self.nombre, self.patrones, self.baudios, self.al_recibir = nombre, patrones, baudios, al_recibir
        self.s, self.dev = None, ""
        self.turno = threading.Lock()
        threading.Thread(target=self.bucle, daemon=True).start()

    def abrir(self):
        for dev, ident in puertos_serie():
            if not coincide(ident, dev, self.patrones):
                continue
            try:
                s = serial.Serial()
                s.port, s.baudrate, s.timeout = dev, self.baudios, 0.1
                s.dtr = s.rts = False
                s.open()
            except (serial.SerialException, OSError):
                continue
            self.s, self.dev = s, dev
            print(f"[com] {self.nombre} conectado en {dev} ({ident})", flush=True)
            return

    def cerrar(self):
        if self.s is not None:
            print(f"[com] {self.nombre} desconectado; lo sigo buscando", flush=True)
            try:
                self.s.close()
            except (serial.SerialException, OSError):
                pass
        self.s, self.dev = None, ""

    def escribir(self, datos):
        s = self.s
        if s is None:
            return False
        try:
            with self.turno:
                s.write(datos)
            return True
        except (serial.SerialException, OSError):
            self.cerrar()
            return False

    def bucle(self):
        resto = b""
        while True:
            if self.s is None:
                self.abrir()
                if self.s is None:
                    time.sleep(1)
                    continue
                resto = b""
            try:
                datos = self.s.read(1024)
            except (serial.SerialException, OSError, TypeError, AttributeError):
                self.cerrar()
                continue
            if not datos:
                continue
            resto += datos
            lineas = resto.split(b"\n")
            resto = lineas.pop()[-300:]
            for linea in lineas:
                try:
                    self.al_recibir(linea.decode("utf-8", "replace").strip())
                except Exception as e:      # una linea rara no debe tumbar el hilo
                    print(f"[com] {self.nombre}: error con una linea: {e}", flush=True)


class Uav:
    """ESP32 NodeMCU del dron (control de inclinacion).

    Funciona con el firmware original captura_octo_m8_banco_angulo y con su variante _pi:
    - Ordenes: siempre por texto (cal, arm BANCO_FIJO, iniciar, parar, vista, marca, log, !), que
      entienden los dos. Ese firmware PARA LOS MOTORES ante cualquier orden que no conoce, asi que
      solo se mandan las de la lista ORDENES y nada mas de forma automatica.
    - Lecturas: la telemetria larga (ms,FASE,roll,pitch,yaw,p,q,r,escala_RP,escala_yaw,M1..M8,accel_g),
      10 por segundo en VISTA/ARMADO/RAMPA/CONTROL, y la corta T,ms,roll,pitch,activo de la variante _pi.
    - Exportacion del log (orden "log"): las filas entre "# LOG INICIO" y "# LOG FIN" se guardan en
      registros/dron_log_<fecha>.csv.
    """
    ORDENES = {"cal": "cal", "arm": "arm BANCO_FIJO", "iniciar": "iniciar", "parar": "parar",
               "vista": "vista", "marca": "marca", "log": "log", "!": "!",
               # solo con el firmware captura_octo_m8_vuelo_guia (se reconoce por su mensaje de arranque)
               "flotar": "flotar", "maximo": "maximo", "subir": "subir", "bajar": "bajar", "mantener": "mantener",
               "aterrizar": "aterrizar"}
    ORDENES_VUELO = ("flotar", "maximo", "subir", "bajar", "mantener", "aterrizar")
    NUMEROS = {1: "cal", 2: "arm", 3: "iniciar", 4: "parar"}
    FASES = ("OFF", "VISTA", "ARMADO", "RAMPA", "CONTROL")

    def __init__(self, patrones, carpeta_registros="registros"):
        self.estado = {"conectado": False, "t_ms": 0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "activo": 0,
                       "fase": "OFF", "p": 0.0, "q": 0.0, "r": 0.0, "escala_rp": 1.0, "motores": [1000] * 8,
                       "accel_g": 0.0, "mensaje": "", "edad_s": None, "calibrado": False, "exportando": False,
                       "ultimo_log": "", "laser_mm": None, "laser_ok": False, "laser_existe": False,
                       "altura_cm": None, "laser_min_mm": None, "laser_max_mm": None,
                       "laser_crudo_mm": None, "laser_estado": "",
                       "firmware_vuelo": False, "colectivo": None, "colectivo_obj": None,
                       "acepta_yb": False, "giro_base_dps": None}
        self._t_yb = 0.0                          # ultimo envio de "yb" (giro de la base) al dron
        self.cero_mm = None
        self.mensajes = deque(maxlen=60)          # (hora, texto) de las lineas "#"
        self.mensajes_t = deque(maxlen=200)       # (marca, hora, texto) para la pagina del dron
        self.historial = deque(maxlen=1200)       # (t_pc, roll, pitch, accel) para las graficas
        self.t_ultima = 0.0
        self.carpeta = carpeta_registros
        self.log_filas = None
        self.vuelo_csv = None                     # registro continuo de cada encendido (registros/vuelo_*.csv)
        self.puerto = PuertoSerie("ESP32_UAV", patrones, 115200, self.linea)
        threading.Thread(target=self.latido, daemon=True).start()

    def latido(self):
        """Con el firmware de vuelo, manda "hb" cada 0,3 s: sin el, el dron no pasa de 1400 us y,
        si se corta, baja solo a 1300. Al firmware de banco NO se le manda (lo tomaria como orden
        desconocida y pararia los motores)."""
        preguntado = None
        while True:
            time.sleep(0.3)
            # si la Pi abrio el puerto despues del arranque del dron no vio su mensaje inicial: con los motores
            # apagados manda una orden vacia de prueba; el firmware responde con su lista de ordenes
            if (self.puerto.s is not None and not self.estado["firmware_vuelo"] and not self.estado["activo"]
                    and preguntado is not self.puerto.s and time.time() - self.t_ultima < 1):
                preguntado = self.puerto.s
                self.puerto.escribir(b"version\n")
            if self.estado["firmware_vuelo"] and self.puerto.s is not None:
                self.puerto.escribir(b"hb\n")

    def giro_base(self, dps):
        """Velocidad de giro de la base (grados/s, + antihorario) calculada por la myRIO. Se reenvia al dron
        como "yb<dps>" a 10 Hz como maximo, y SOLO si su firmware la entiende (al v9 una orden desconocida
        le pararia los motores)."""
        self.estado["giro_base_dps"] = round(float(dps), 1)
        ahora = time.time()
        if self.estado["acepta_yb"] and self.puerto.s is not None and ahora - self._t_yb >= 0.1:
            self._t_yb = ahora
            self.puerto.escribir(f"yb{float(dps):.1f}\n".encode())

    def linea(self, s):
        if not s:
            return
        if self.log_filas is not None and not s.startswith("#"):
            self.log_filas.append(s)
            return
        if s.startswith("#"):
            texto = s[1:].strip()
            if texto.startswith("LASER DIAG"):
                # llega cada 2 s con los motores apagados: se guarda aparte para que no tape CAL OK ni otros avisos
                self.estado["laser_diag"] = texto[:200]
                print(f"[uav] {s}", flush=True)
                return
            self.estado["mensaje"] = texto[:160]
            self.anotar(texto[:200])
            print(f"[uav] {s}", flush=True)
            if " yb" in texto or "con yb" in texto:
                self.estado["acepta_yb"] = True      # firmware v10b o posterior: entiende el giro de la base
            if texto.startswith("OCTO M8"):
                self.estado["firmware_vuelo"] = texto.startswith("OCTO M8 VUELO GUIA")
            elif "flotar" in texto or texto.startswith("VUELO"):
                self.estado["firmware_vuelo"] = True     # respuesta que solo da el firmware de vuelo
            elif texto.startswith("COLECTIVO"):
                import re
                m = re.search(r"objetivo=(\d+) actual=(\d+)", texto)
                if m:
                    self.estado.update(colectivo_obj=int(m.group(1)), colectivo=int(m.group(2)))
            if texto.startswith("CAL OK"):
                self.estado["calibrado"] = True
            elif texto.startswith(("DESARMADO", "CAL:", "CAL RECHAZADA")):
                self.estado["calibrado"] = False   # al empezar una calibracion nueva la anterior deja de valer
            if texto.startswith("LOG INICIO"):
                self.log_filas, self.estado["exportando"] = [], True
            elif texto.startswith("LOG FIN") and self.log_filas is not None:
                self.guardar_log()
            if texto.startswith(("PARADA", "CORTE", "ARMADO VENCIDO", "RECHAZADO", "ORDEN RECHAZADA",
                                 "LINEA INVALIDA", "FALLO", "DESARMADO")):
                self.estado.update(fase="OFF", activo=0, colectivo=None, colectivo_obj=None)
                self.cerrar_vuelo()
            return
        p = s.split(",")
        try:
            if p[0] == "T" and len(p) == 5:
                self.estado.update(t_ms=int(p[1]), roll=float(p[2]), pitch=float(p[3]), activo=int(p[4]))
            elif len(p) in (19, 21) and p[1] in self.FASES and p[0].isdigit():
                v = [float(x) for x in p[2:]]
                self.estado.update(t_ms=int(p[0]), fase=p[1], roll=v[0], pitch=v[1], yaw=v[2], p=v[3], q=v[4],
                                   r=v[5], escala_rp=v[6], motores=[int(x) for x in v[8:16]], accel_g=v[16],
                                   activo=int(p[1] in ("RAMPA", "CONTROL")))
                if len(p) == 21:
                    self.laser(int(v[17]), int(v[18]))
            else:
                return
        except ValueError:
            return
        self.t_ultima = time.time()
        e = self.estado
        self.registrar_vuelo(s)
        self.historial.append((round(self.t_ultima, 3), e["roll"], e["pitch"], e["accel_g"], e["altura_cm"]))

    def registrar_vuelo(self, s):
        """Guarda cada linea de telemetria (10 por segundo) mientras los motores giran."""
        import os
        if s.count(",") != 20:                    # solo la telemetria larga con laser
            return
        if self.estado["activo"]:
            if self.vuelo_csv is None:
                os.makedirs(self.carpeta, exist_ok=True)
                ruta = os.path.join(self.carpeta, time.strftime("vuelo_%Y%m%d_%H%M%S.csv"))
                self.vuelo_csv = open(ruta, "w")
                self.vuelo_csv.write("t_pc,ms,modo,roll_deg,pitch_deg,yaw_rel_deg,p_dps,q_dps,r_dps,escala_RP,escala_yaw,"
                                     "M1_us,M2_us,M3_us,M4_us,M5_us,M6_us,M7_us,M8_us,accel_g,laser_mm,laser_estado,colectivo_obj\n")
                print(f"[uav] registrando el encendido en {ruta}", flush=True)
            obj = self.estado["colectivo_obj"]
            self.vuelo_csv.write(f"{time.time():.3f},{s},{'' if obj is None else obj}\n")
        else:
            self.cerrar_vuelo()

    def cerrar_vuelo(self):
        if self.vuelo_csv is not None:
            self.vuelo_csv.close()
            self.vuelo_csv = None

    def laser(self, mm, codigo):
        """Lectura del laser de altura (mira hacia abajo desde el brazo de M8). La altura es la distancia
        vertical respecto a la lectura tomada como cero (corrige la inclinacion del dron)."""
        import math
        e = self.estado
        ok = codigo == 0
        e.update(laser_existe=True, laser_ok=ok, laser_mm=mm if ok else None, laser_crudo_mm=mm,
                 laser_estado={0: "valida", 1: "sigma alta", 2: "senal debil", 3: "rango minimo", 4: "fuera de rango",
                               5: "fallo de hardware", 254: "sin medida reciente",
                               255: "laser no iniciado"}.get(codigo, f"codigo {codigo}"))
        if not ok:
            return
        vertical = mm * math.cos(math.radians(e["roll"])) * math.cos(math.radians(e["pitch"]))
        if self.cero_mm is None:
            self.cero_mm = vertical
        e["altura_cm"] = round((vertical - self.cero_mm) / 10, 1)
        e["laser_min_mm"] = mm if e["laser_min_mm"] is None else min(e["laser_min_mm"], mm)
        e["laser_max_mm"] = mm if e["laser_max_mm"] is None else max(e["laser_max_mm"], mm)

    def cero_altura(self):
        self.cero_mm = None
        self.estado.update(altura_cm=None, laser_min_mm=None, laser_max_mm=None)

    def anotar(self, texto):
        self.mensajes.append((time.strftime("%H:%M:%S"), texto))
        self.mensajes_t.append((time.time(), time.strftime("%H:%M:%S"), texto))

    def guardar_log(self):
        import os
        filas, self.log_filas = self.log_filas, None
        self.estado["exportando"] = False
        os.makedirs(self.carpeta, exist_ok=True)
        ruta = os.path.join(self.carpeta, time.strftime("dron_log_%Y%m%d_%H%M%S.csv"))
        with open(ruta, "w") as f:
            f.write("\n".join(filas) + "\n")
        self.estado["ultimo_log"] = os.path.basename(ruta)
        print(f"[uav] log del dron guardado en {ruta} ({len(filas)} filas)", flush=True)

    def leer(self):
        e = self.estado
        e["conectado"] = self.puerto.s is not None
        e["edad_s"] = round(time.time() - self.t_ultima, 2) if self.t_ultima else None
        return e

    def orden(self, nombre):
        """Manda una orden de la lista ORDENES. Devuelve None si se envio, o el motivo."""
        if nombre not in self.ORDENES:
            return "orden desconocida"
        if nombre in self.ORDENES_VUELO and not self.estado["firmware_vuelo"]:
            return "el firmware grabado no tiene modo vuelo (grabar captura_octo_m8_vuelo_guia)"
        if self.puerto.s is None:
            return "ESP32_UAV no conectado"
        texto = self.ORDENES[nombre]
        datos = b"!" if texto == "!" else texto.encode() + b"\n"
        if not self.puerto.escribir(datos):
            return "fallo al escribir al ESP32_UAV"
        self.anotar(f"> {texto}")
        return None

    def comando(self, n):
        return self.orden(self.NUMEROS[n]) if n in self.NUMEROS else "comando del dron desconocido"


class Enlace:
    """Arbitra web/TCP y UART del myRIO, y envia telemetria a ambos destinos.

    El myRIO envia dos codigos por linea. Los codigos 1..4 se enrutan al UAV y
    los 5..11 a Ball and Plate, como define ``Variables y significados.png``.
    """

    def __init__(self, ejecutar, lectura_uav, lectura_bp, puerto_myrio, baudios_myrio, al_giro_base=None):
        self.ejecutar, self.lectura_uav, self.lectura_bp = ejecutar, lectura_uav, lectura_bp
        self.al_giro_base = al_giro_base          # recibe la velocidad de giro de la base (grados/s)
        self.clientes = []
        self.ultimo = {"origen": "", "comando": "", "respuesta": "", "hace_s": None}
        self.t_ultimo = 0.0
        self._turno_comandos = threading.Lock()
        self._prioridad_myrio_hasta = 0.0
        self.myrio = MyRIOUART(puerto_myrio, baudios_myrio, self.trama_myrio)
        threading.Thread(target=self.servidor_tcp, daemon=True).start()
        threading.Thread(target=self.difundir, daemon=True).start()

    def _ejecutar(self, codigo, origen):
        """Ejecuta un codigo y registra el resultado bajo el mismo turno."""
        error = self.ejecutar(codigo) if codigo in NOMBRES else "comando desconocido (1..11)"
        respuesta = f"OK,{codigo}" if error is None else f"ERR,{codigo},{error}"
        self.ultimo.update(origen=origen, comando=str(codigo), respuesta=respuesta)
        self.t_ultimo = time.time()
        print(f"[com] {origen}: {codigo} -> {respuesta}", flush=True)
        return respuesta

    def trama_myrio(self, texto):
        """Procesa ``cmd_1,cmd_2\\n``; no genera confirmacion por UART.
        Ademas acepta ``w,<grados/s>\\n``: velocidad de giro de la base (odometria de la myRIO), que se
        reenvia al dron para que siga el giro de la base."""
        t = texto.strip()
        if t[:2].lower() == "w,":
            try:
                dps = float(t[2:].replace(" ", ""))
            except ValueError:
                return
            if self.al_giro_base and abs(dps) <= 180:
                self.al_giro_base(dps)
            return
        try:
            trama = parsear_comandos(texto)
        except TramaInvalida as error:
            self.ultimo.update(origen="myRIO UART", comando=texto[:20], respuesta=f"ERR,{error}")
            self.t_ultimo = time.time()
            print(f"[myrio] trama descartada {texto!r}: {error}", flush=True)
            return

        # Durante esta breve ventana, un clic web/TCP concurrente no puede
        # adelantarse a la orden del panel myRIO.
        with self._turno_comandos:
            self._prioridad_myrio_hasta = time.monotonic() + 0.25
            for codigo in trama.codigos:
                self._ejecutar(codigo, "myRIO UART")

    def atender(self, texto, origen):
        """Una linea web/TCP -> respuesta. myRIO siempre tiene prioridad."""
        texto = texto.strip()
        if not texto:
            return ""
        try:
            n = int(texto)
        except ValueError:
            r = "ERR,0,comando no numerico"
        else:
            with self._turno_comandos:
                if time.monotonic() < self._prioridad_myrio_hasta:
                    r = "ERR,0,prioridad myRIO activa"
                else:
                    r = self._ejecutar(n, origen)
        if not r.startswith("OK,"):
            self.ultimo.update(origen=origen, comando=texto[:20], respuesta=r)
            self.t_ultimo = time.time()
            print(f"[com] {origen}: {texto[:20]!r} -> {r}", flush=True)
        return r + "\n"

    def estado(self):
        self.ultimo["hace_s"] = round(time.time() - self.t_ultimo, 1) if self.t_ultimo else None
        return {"myrio_uart": self.myrio.dispositivo, "myrio_baudios": self.myrio.baudios,
                "clientes_tcp": len(self.clientes), "ultimo_comando": self.ultimo}

    def servidor_tcp(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("", PUERTO_TCP))
        srv.listen(4)
        while True:
            c, origen = srv.accept()
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=self.cliente_tcp, args=(c, f"TCP {origen[0]}"), daemon=True).start()

    def cliente_tcp(self, c, origen):
        print(f"[com] {origen} conectado", flush=True)
        self.clientes.append(c)
        resto = b""
        try:
            while True:
                datos = c.recv(256)
                if not datos:
                    break
                resto += datos
                lineas = resto.replace(b"\r", b"\n").split(b"\n")
                resto = lineas.pop()[-64:]
                for linea in lineas:
                    r = self.atender(linea.decode("utf-8", "replace"), origen)
                    if r:
                        c.sendall(r.encode())
        except OSError:
            pass
        finally:
            if c in self.clientes:
                self.clientes.remove(c)
            c.close()
            print(f"[com] {origen} desconectado", flush=True)

    def difundir(self):
        while True:
            time.sleep(0.05)
            if not self.myrio.conectado and not self.clientes:
                continue
            u = self.lectura_uav()
            t, x, y, activo = self.lectura_bp()
            telemetria_myrio = formato_telemetria(x, y, u["roll"], u["pitch"], u["activo"], activo)
            self.myrio.escribir(telemetria_myrio)
            texto_tcp = (f"U,{u['t_ms']},{u['roll']:.2f},{u['pitch']:.2f},{u['activo']}\n"
                         f"B,{t},{'' if x is None else f'{x:.1f}'},{'' if y is None else f'{y:.1f}'},{activo}\n").encode()
            for c in list(self.clientes):
                try:
                    c.sendall(texto_tcp)
                except OSError:
                    pass
