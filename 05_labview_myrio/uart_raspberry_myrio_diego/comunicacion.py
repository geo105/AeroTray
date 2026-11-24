"""Sistema de comunicacion de la Raspberry Pi (lo usa seguidor.py).

    Laptop --WiFi--> pagina web --+
                                 +--> Raspberry Pi --USB--> ESP32_UAV / ESP32_BALL AND PLATE
    myRIO --UART TTL GPIO14/15--+

El myRIO recibe y transmite una trama propia. La Pi nunca devuelve ``OK`` o
``ERR`` por la UART del myRIO: esa direccion queda reservada a telemetria.

Comandos de pagina/TCP (un numero por linea):
    1 Calibrar   2 1er inicio (armar)   3 2do inicio (motores)   4 Parar              -> ESP32_UAV
    5 Inicio PID   6 Neutro   7 Apagar PID   8 Marcar esquinas   9 Guardar placa vacia
    10 Nivelar placa   11 Borrar placa vacia                                           -> ball and plate
Respuesta a cada comando:  OK,<n>   o   ERR,<n>,<motivo>
Telemetria que la Pi manda al myRIO (20 veces por segundo):
    pos_x_mm,pos_y_mm,roll_grados,pitch_grados,uav_activo,bp_activo\n
Los clientes TCP conservan las lineas U y B originales para no romper las
herramientas de diagnostico existentes.
"""
import glob
import os
import socket
import threading
import time

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
    """ESP32 NodeMCU del dron (control de inclinacion). Recibe 1..4 y manda T,ms,roll,pitch,activo."""

    def __init__(self, patrones):
        self.estado = {"conectado": False, "t_ms": 0, "roll": 0.0, "pitch": 0.0, "activo": 0,
                       "fase": "", "mensaje": "", "edad_s": None}
        self.t_ultima = 0.0
        self.puerto = PuertoSerie("ESP32_UAV", patrones, 115200, self.linea)

    def linea(self, s):
        if s.startswith("T,"):
            p = s.split(",")
            if len(p) == 5:
                try:
                    self.estado.update(t_ms=int(p[1]), roll=float(p[2]), pitch=float(p[3]), activo=int(p[4]))
                    self.t_ultima = time.time()
                except ValueError:
                    pass
        elif s.startswith("#"):
            self.estado["mensaje"] = s[1:].strip()[:120]
            print(f"[uav] {s}", flush=True)
        else:
            p = s.split(",")            # telemetria larga del firmware: ms,FASE,roll,pitch,...
            if len(p) >= 19 and p[1] in ("VISTA", "ARMADO", "RAMPA", "CONTROL"):
                self.estado["fase"] = p[1]

    def leer(self):
        e = self.estado
        e["conectado"] = self.puerto.s is not None
        e["edad_s"] = round(time.time() - self.t_ultima, 2) if self.t_ultima else None
        if e["edad_s"] is None or e["edad_s"] > 1.0:
            e["fase"] = ""
        return e

    def comando(self, n):
        if self.puerto.s is None:
            return "ESP32_UAV no conectado"
        return None if self.puerto.escribir(f"{n}\n".encode()) else "fallo al escribir al ESP32_UAV"


class Enlace:
    """Arbitra web/TCP y UART del myRIO, y envia telemetria a ambos destinos.

    El myRIO envia dos codigos por linea. Los codigos 1..4 se enrutan al UAV y
    los 5..11 a Ball and Plate, como define ``Variables y significados.png``.
    """

    def __init__(self, ejecutar, lectura_uav, lectura_bp, puerto_myrio, baudios_myrio):
        self.ejecutar, self.lectura_uav, self.lectura_bp = ejecutar, lectura_uav, lectura_bp
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
        """Procesa ``cmd_1,cmd_2\\n``; no genera confirmacion por UART."""
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
