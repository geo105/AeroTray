#!/usr/bin/env python3
"""Ball and plate con la PS3 Eye en la Raspberry Pi.

La Pi captura con la PS3 Eye (320x240 a 100-187 FPS), detecta la bola y manda su posicion al
ESP32 por USB ("p<x>,<y>"). El ESP32 hace el PID y mueve los servos, igual que con su propia camara.

Pagina web para ver la camara, calibrar y encender / apagar el control:
    http://<ip de la pi>:8080

Calibracion (desde la pagina):
  1. Esquinas: clic en las 4 esquinas de la placa, en este orden segun los ejes de los servos:
     X- Y+ (arriba izquierda), X+ Y+ (arriba derecha), X+ Y- (abajo derecha), X- Y- (abajo izquierda).
     La Pi endereza la imagen (vista desde arriba, en milimetros) y la alinea con los servos.
  2. Placa vacia: sin la bola, boton "Guardar placa vacia". Desde ahi se detecta solo lo que es
     distinto de esa imagen: reflejos fijos, tornillos, marcas y bordes ya no se confunden con la bola.

Uso:
    python3 seguidor.py                  # usa config_pi.json y ajustes.json de esta carpeta
    python3 seguidor.py /dev/ttyUSB1     # un puerto fijo (sin argumento lo busca solo)

config_pi.json: camara, esquinas de la placa y deteccion (se guarda solo al calibrar).
ajustes.json:   ganancias y neutros del PID (el mismo archivo que usa visor.py en la laptop).
fondo.png:      la placa vacia ya enderezada.
Los archivos se recargan solos si se editan mientras corre.
"""
import csv
import glob
import json
import os
import subprocess
import sys
import threading
from collections import deque
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import serial

import comunicacion
from pagina_dron import PAGINA_DRON
from pagina_robot import PAGINA_ROBOT

CARPETA = os.path.dirname(os.path.abspath(__file__))
ARCHIVO_CONFIG = os.path.join(CARPETA, "config_pi.json")
ARCHIVO_AJUSTES = os.path.join(CARPETA, "ajustes.json")
ARCHIVO_FONDO = os.path.join(CARPETA, "fondo.png")
PUERTO = sys.argv[1] if len(sys.argv) > 1 else None   # sin argumento: el primer /dev/ttyUSB* o ttyACM*
BAUDIOS = 2000000

CONFIG_DEFECTO = {
    "ancho": 320, "alto": 240, "fps": 125,
    "exposicion": 120,        # 0-255 (con 125 fps la imagen tiene como maximo 8 ms de luz)
    "ganancia": 20,           # 0-63
    # esquinas de la placa en la imagen de la PS3 Eye: X-Y+, X+Y+, X+Y-, X-Y- (ejes de los servos)
    "esquinas": [[60, 20], [260, 20], [260, 220], [60, 220]],
    "placa_mm": [220, 290],   # ancho (eje X) y alto (eje Y) de la placa
    "px_por_mm": 0.8,         # resolucion de la imagen enderezada
    # zona de busqueda: la placa negra ampliada esto hacia afuera (negativo: se achica para dejar
    # fuera su canto). La bola tambien rueda por la parte clara que la rodea
    "margen_mm": 40,
    # el PID del ESP32 esta ajustado en pixeles de su camara: 1 px = 3 mm
    "mm_por_unidad": 3.0,
    "diametro_bola_mm": 40,
    # "color": por el color de la bola (lo mas robusto con la placa negra y luz pareja);
    # "entorno": la bola es lo que difiere de su alrededor (no le afecta que la placa se incline);
    # "placa_vacia": lo que difiere de la placa vacia guardada (solo sirve con la placa plana)
    "metodo": "entorno",
    # "color": la bola es lo que tiene el color de color_bola (placa negra, sin color); color_min es
    # cuanto color hace falta (0-255, la placa negra da ~0)
    "color_bola": "azul", "color_min": 25,
    # tamano minimo aceptado, como fraccion del area de la bola. Moviendose, la bola se ve borrosa y
    # mas chica (en color baja hasta ~40 %); con el metodo color no hay reflejos ni mano que se cuelen
    "area_min_fraccion": 0.4,
    # la bola es lo que difiere de su entorno o de la placa vacia en mas de esto (0-255)
    "diferencia_min": 28,
    # sin placa vacia: umbral = fondo + fraccion * (pico - fondo)
    "umbral_fraccion": 0.3, "contraste_min": 40,
    "ventana_mm": 60,         # medio lado de la zona de busqueda alrededor de la prediccion
    "salto_max_mm": 40,       # mas que esto entre dos imagenes es un error de deteccion
    # velocidad calculada en la Pi (recta ajustada a las posiciones de los ultimos ventana_vel_s) y
    # mandada al ESP32 para el derivativo del PID; mas limpia que la que saca el ESP32 restando
    "velocidad_pi": True, "ventana_vel_s": 0.06,
}

ESTADO_TEXTO = {0: "apagado", 1: "PID (sin bola)", 3: "PID", 4: "neutro fijo", 6: "neutro fijo", 12: "seno", 14: "seno"}


def cargar(archivo, defecto):
    datos = dict(defecto)
    try:
        datos.update(json.load(open(archivo)))
    except (OSError, ValueError):
        pass
    return datos


def mtime(archivo):
    try:
        return os.path.getmtime(archivo)
    except OSError:
        return 0


# OpenCV con un solo hilo: las imagenes que se procesan son pequenas (la ventana alrededor de la bola)
# y repartirlas entre 4 hilos costaba mas que hacerlas (medido en la Pi 3: 17.9 ms -> 11.9 ms)
cv2.setNumThreads(1)

config = cargar(ARCHIVO_CONFIG, CONFIG_DEFECTO)
if not os.path.exists(ARCHIVO_CONFIG):
    json.dump(config, open(ARCHIVO_CONFIG, "w"), indent=2)
ajustes = cargar(ARCHIVO_AJUSTES, {})
mtimes = {ARCHIVO_CONFIG: mtime(ARCHIVO_CONFIG), ARCHIVO_AJUSTES: mtime(ARCHIVO_AJUSTES)}

# Sistema de comunicacion: que adaptador USB-serie es cada equipo (por el nombre de su chip en
# /dev/serial/by-id). Se puede cambiar en config_pi.json: "uav_puerto", "myrio_puerto", "myrio_baudios"
PATRON_UAV = [config.get("uav_puerto", "CP210")]                  # NodeMCU del dron: chip CP2102
# El myRIO va por la UART TTL de los GPIO (TX GPIO14 pin 8, RX GPIO15 pin 10, GND pin 6), no por USB.
PATRON_MYRIO = []
PUERTO_MYRIO = config.get("myrio_uart", "/dev/serial0")
BAUDIOS_MYRIO = int(config.get("myrio_baudios", 9600))
T_INICIO = time.time()

# modo pedido al ESP32: 0 apagado, 1 PID, 2 neutro fijo (arranca en neutro: placa plana)
modo = 2
bloqueo = threading.Lock()
vistas = {}                  # JPEG para la pagina web: "camara", "placa" y "color"
# cuantas paginas miran cada vista: las imagenes solo se dibujan y comprimen si alguien las mira
# (con la Pi lenta, dibujarlas siempre le quitaba muchos FPS a la deteccion)
espectadores = {"camara": 0, "placa": 0, "color": 0}
VISTA_CADA_S = 0.1           # 10 imagenes por segundo en la pagina es suficiente para mirar
pedir_fondo = False          # la pagina pidio guardar la placa vacia
estado_esp = {"pulso_x": 0, "pulso_y": 0, "estado": 0, "medidas_s": 0, "int_x": 0, "int_y": 0}
estado_pi = {"fps": 0.0, "det_ms": 0.0, "detectada": 0.0, "x_mm": None, "y_mm": None, "area_mm2": 0,
             "umbral": 0, "con_fondo": False, "nivelacion": ""}
# posiciones recientes de la bola (tiempo, x_mm, y_mm): para la velocidad y para nivelar
historial = deque(maxlen=400)


# ---------------------------------------------------------------- ESP32
class PuertoESP:
    """Puerto serie del ESP32 que se reconecta solo: si se desenchufa (o pasa a otro USB / hub), las
    escrituras se descartan y cada segundo se busca de nuevo. Sin ESP32 se puede probar igual la
    camara, la deteccion y la pagina web."""

    def __init__(self):
        self.s = None
        # el bucle principal (posiciones, ~100/s) y la pagina web (botones) escriben desde hilos
        # distintos: sin turno, sus mensajes se mezclaban en el cable y el ESP32 los descartaba
        self.turno = threading.Lock()
        self.abrir()
        if self.s is None:
            print("[pi] AVISO: no encuentro el ESP32; sigo buscandolo (camara y deteccion funcionan)", flush=True)
        threading.Thread(target=self.vigilar, daemon=True).start()

    def abrir(self):
        # el del ball and plate es cualquier adaptador que no sea el del dron ni el del myRIO
        candidatos = [PUERTO] if PUERTO else [dev for dev, ident in comunicacion.puertos_serie()
                                              if not comunicacion.coincide(ident, dev, PATRON_UAV + PATRON_MYRIO)]
        for p in candidatos:
            try:
                # DTR y RTS apagados antes de abrir: con el CH340 en la Pi, si quedan encendidos el
                # ESP32 se queda en reset o en modo de carga y no arranca
                s = serial.Serial()
                s.port, s.baudrate, s.timeout = p, BAUDIOS, 0.05
                s.dtr = s.rts = False
                s.open()
            except (serial.SerialException, OSError):
                continue
            time.sleep(2.5)  # el ESP32 arranca (inicia su camara) antes de recibir ajustes
            self.s = s
            print(f"[pi] ESP32 conectado en {p}", flush=True)
            return

    def cerrar(self):
        # lo pueden llamar a la vez el hilo que lee y el que escribe: se toma el puerto una sola vez
        # (antes el segundo encontraba None, fallaba y el hilo de lectura moria sin avisar)
        s, self.s = self.s, None
        if s is not None:
            print("[pi] ESP32 desconectado; lo sigo buscando", flush=True)
            try:
                s.close()
            except Exception:
                pass

    def vigilar(self):
        while True:
            time.sleep(1)
            if self.s is None:
                self.abrir()
                if self.s is not None:
                    enviar_ajustes()

    def write(self, datos):
        s = self.s
        if s is None:
            return
        try:
            with self.turno:
                s.write(datos)
        except (serial.SerialException, OSError):
            self.cerrar()

    def read(self, n):
        s = self.s
        if s is None:
            time.sleep(0.1)
            return b""
        try:
            return s.read(n)
        except (serial.SerialException, OSError, TypeError):
            self.cerrar()
            return b""


ser = PuertoESP()


# --- correccion por inclinacion del dron (prealimentacion) ---
# La placa va montada sobre el dron: si el dron se inclina, la placa se inclina igual. Se suma al neutro
# de los servos  K * (roll, pitch) del dron [us por grado], antes de que la bola empiece a rodar; el PID
# sigue corrigiendo encima. En ajustes.json:
#   "ff_activo": 0/1, "ff_k": [x_por_roll, x_por_pitch, y_por_roll, y_por_pitch], "ff_max": us maximos
ff = {"x": 0.0, "y": 0.0, "roll": 0.0, "pitch": 0.0, "enviado": None, "t_envio": 0.0}


def neutro_actual():
    """Neutro de los servos con la correccion por inclinacion del dron (si esta activa y hay datos frescos)."""
    a = ajustes
    nx, ny = a["neutro_x"], a["neutro_y"]
    e = uav.estado
    fresco = uav.t_ultima and time.time() - uav.t_ultima < 0.4 and e.get("calibrado")
    if a.get("ff_activo") and fresco:
        k = a.get("ff_k", [0, 0, 0, 0])
        # filtro suave: la telemetria llega a 10 Hz y trae algo de vibracion
        ff["roll"] += 0.5 * (e["roll"] - ff["roll"])
        ff["pitch"] += 0.5 * (e["pitch"] - ff["pitch"])
        m = a.get("ff_max", 150)
        ff["x"] = max(-m, min(m, k[0] * ff["roll"] + k[1] * ff["pitch"]))
        ff["y"] = max(-m, min(m, k[2] * ff["roll"] + k[3] * ff["pitch"]))
    else:
        ff["x"] = ff["y"] = 0.0
    return int(round(nx + ff["x"])), int(round(ny + ff["y"]))


def enviar_neutro(forzar=False):
    """Manda el neutro corregido si cambio (como mucho 50 veces por segundo)."""
    n = neutro_actual()
    ahora = time.time()
    if forzar or (n != ff["enviado"] and ahora - ff["t_envio"] >= 0.02):
        ser.write(f"n{n[0]},{n[1]}\n".encode())
        ff["enviado"], ff["t_envio"] = n, ahora


def enviar_ajustes():
    a = ajustes
    if not a:
        return
    ser.write((f"k{a['kp']:.4f},{a['ki']:.4f},{a['kd']:.4f}\n"
               f"n{neutro_actual()[0]},{neutro_actual()[1]}\n"
               f"l{a['limite']}\n"
               f"d{a['signo_x']},{a['signo_y']}\n"
               f"f{a['factor_x']:.3f},{a['factor_y']:.3f}\n"
               f"v{a['filtro_v']:.2f}\n"
               f"w{a['reinicio_integral']}\n"
               f"m{a['friccion']}\n"
               f"i{a['vel_integral']},{a['autoridad_i']}\n"
               f"b{a['ab_alfa']},{a['ab_beta']}\n"
               f"z{a['holgura'][0]},{a['holgura'][1]},{a['holgura'][2]},{a['holgura'][3]},{a.get('holgura_rampa', 10)}\n"
               f"s{a['consigna_x']:.1f},{a['consigna_y']:.1f}\n"
               f"o{modo}\n").encode())


def leer_esp():
    """Lee las lineas de estado del ESP32 ("S...") y descarta el resto (imagenes de su camara)."""
    resto = b""
    while True:
        try:
            datos = ser.read(4096)
        except Exception as e:          # este hilo no debe morir nunca: sin el no hay estado del ESP32
            print(f"[pi] error leyendo el ESP32: {e}", flush=True)
            time.sleep(0.5)
            continue
        if not datos:
            continue
        resto += datos
        lineas = resto.split(b"\n")
        resto = lineas.pop()[-200:]
        for linea in lineas:
            i = linea.rfind(b"S")
            if i < 0:
                continue
            try:
                v = [int(c) for c in linea[i + 1:].decode().split(",")]
            except ValueError:
                continue
            if len(v) == 6:
                estado_esp.update(zip(("pulso_x", "pulso_y", "estado", "medidas_s", "int_x", "int_y"), v))


threading.Thread(target=leer_esp, daemon=True).start()


# ---------------------------------------------------------------- camara
def ctl(*ajustes_v4l2):
    for a in ajustes_v4l2:
        subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "-c", a], capture_output=True)


def aplicar_exposicion():
    ctl("auto_exposure=1", "gain_automatic=0", f"gain={config['ganancia']}", f"exposure={config['exposicion']}")


def abrir_camara():
    c = config
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, c["ancho"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c["alto"])
    cap.set(cv2.CAP_PROP_FPS, c["fps"])
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)   # se recibe el YUYV tal cual y se usa solo el brillo (Y)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    aplicar_exposicion()
    return cap


# ---------------------------------------------------------------- placa enderezada
class Placa:
    """Endereza la imagen de la camara a una vista de la placa desde arriba, en milimetros, con X a
    la derecha e Y hacia arriba segun los servos (lo deciden las esquinas)."""

    def __init__(self):
        c = config
        self.s = c["px_por_mm"]
        self.w, self.h = int(c["placa_mm"][0] * self.s), int(c["placa_mm"][1] * self.s)
        destino = np.float32([[0, 0], [self.w, 0], [self.w, self.h], [0, self.h]])
        self.H = cv2.getPerspectiveTransform(np.float32(c["esquinas"]), destino)
        self.H_inv = np.linalg.inv(self.H)
        # pixeles de camara por mm, en promedio (la perspectiva hace un lado de la placa mas grande)
        area_cam = cv2.contourArea(np.float32(c["esquinas"]))
        self.s_cam = float(np.sqrt(area_cam / (c["placa_mm"][0] * c["placa_mm"][1])))
        # zona de busqueda: la placa negra ampliada margen_mm hacia afuera (la bola tambien rueda por
        # la parte clara que la rodea; con margen negativo se deja fuera el canto). Se calcula en mm,
        # se lleva a la camara y se recorta al borde de la imagen. La deteccion se hace sobre la
        # imagen de la camara: enderezar cada imagen cuesta mucho en la Pi 3
        m = c["margen_mm"] * self.s
        zona = np.float32([[[-m, -m], [self.w + m, -m], [self.w + m, self.h + m], [-m, self.h + m]]])
        zona_cam = cv2.perspectiveTransform(zona, self.H_inv)[0]
        self.zona_cam = np.int32(zona_cam)
        self.mascara = np.zeros((c["alto"], c["ancho"]), np.uint8)
        cv2.fillPoly(self.mascara, [self.zona_cam], 1)
        self.mascara[:4, :] = self.mascara[-4:, :] = 0
        self.mascara[:, :4] = self.mascara[:, -4:] = 0
        ys, xs = np.nonzero(self.mascara)
        self.caja = (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)

    def enderezar(self, g):
        return cv2.warpPerspective(g, self.H, (self.w, self.h), flags=cv2.INTER_LINEAR)

    def camara_a_mm(self, x, y):
        p = self.H @ np.array([x, y, 1.0])
        return self.a_mm(p[0] / p[2], p[1] / p[2])

    def a_mm(self, u, v):
        """Pixel de la imagen enderezada -> mm desde el centro (X derecha, Y arriba)."""
        return u / self.s - config["placa_mm"][0] / 2, config["placa_mm"][1] / 2 - v / self.s


# ---------------------------------------------------------------- deteccion
class Detector:
    """En la placa enderezada: lo distinto de la placa vacia (o, sin ella, lo mas brillante), en
    manchas; los trozos cercanos se juntan; se elige el grupo con tamano y forma de bola mas cercano
    a la prediccion."""

    def __init__(self, placa, fondo):
        self.placa, self.fondo = placa, fondo
        self.ult = None          # ultima posicion valida (pixeles de la placa enderezada)
        self.vel = (0.0, 0.0)    # pixeles por imagen
        self.perdidas = 99

    def detectar(self, p, yuyv=None):
        """p: imagen de la camara (brillo); yuyv: la imagen en bruto, para el metodo "color".
        Devuelve la bola en pixeles de la camara (x, y, area en mm2, umbral)."""
        c, s = config, self.placa.s_cam
        d_px = c["diametro_bola_mm"] * s
        area = np.pi * d_px * d_px / 4
        # por la perspectiva la bola se ve mas grande en un lado de la placa (medido 830-2000 mm2 para
        # una de ~1250). Con un minimo de 10 % se colaban manchitas de ~150 mm2 en una esquina y, al
        # perder la bola un instante, el PID la "veia" ahi e inclinaba la placa a tope
        area_min, area_max = c["area_min_fraccion"] * area, 2.5 * area
        radio_union = 0.9 * d_px

        prediccion = None
        x0, y0, x1, y1 = self.placa.caja
        if self.ult is not None and self.perdidas < 5:
            prediccion = (self.ult[0] + self.vel[0] * (self.perdidas + 1),
                          self.ult[1] + self.vel[1] * (self.perdidas + 1))
            v = int(c["ventana_mm"] * s)
            x0, y0 = max(x0, int(prediccion[0]) - v), max(y0, int(prediccion[1]) - v)
            x1, y1 = min(x1, int(prediccion[0]) + v + 1), min(y1, int(prediccion[1]) + v + 1)
        if prediccion is None and c["metodo"] == "color" and yuyv is not None and c.get("busqueda_gruesa", True):
            # bola perdida: en vez de analizar toda la imagen al detalle (12 ms en la Pi), se busca
            # primero a un cuarto de resolucion donde hay mas color y se analiza solo esa ventana
            g4 = imagen_color(yuyv[y0:y1:2, 2 * (x0 // 2 * 2):2 * x1])[:, ::2]
            m4 = self.placa.mascara[y0:y1:2, x0 // 2 * 2:x1:2]
            n4 = min(g4.shape[1], m4.shape[1])
            g4 = g4[:, :n4] * m4[:, :n4]
            k4 = max(3, int(d_px / 2) | 1)
            _, pico, _, donde = cv2.minMaxLoc(cv2.blur(g4, (k4, k4)))
            if int(g4.max()) < c["color_min"]:
                return self.perder(0)
            cx, cy = x0 // 2 * 2 + 2 * donde[0], y0 + 2 * donde[1]
            v = int(c["ventana_mm"] * s)
            x0, y0 = max(x0, cx - v), max(y0, cy - v)
            x1, y1 = min(x1, cx + v + 1), min(y1, cy + v + 1)
        zona = p[y0:y1, x0:x1]
        mascara = self.placa.mascara[y0:y1, x0:x1]
        if zona.size == 0:
            return self.perder(0)

        if c["metodo"] == "color" and yuyv is not None:
            # la bola es lo que tiene su color: la placa negra y los reflejos blancos no tienen color.
            # Solo se calcula en la zona de busqueda (el color va de a 2 pixeles: se alinea a pares)
            xa, xb = x0 // 2 * 2, min((x1 + 1) // 2 * 2, yuyv.shape[1] // 2)
            col = imagen_color(yuyv[y0:y1, 2 * xa:2 * xb])[:, x0 - xa:x1 - xa]
            if col.shape != mascara.shape:      # en el borde de la imagen puede faltar una columna
                col = cv2.resize(col, (mascara.shape[1], mascara.shape[0]), interpolation=cv2.INTER_NEAREST)
            dif = col * mascara
            if int(dif.max()) < c["color_min"]:
                return self.perder(0)
            u = c["color_min"]
            binaria = dif > u
        elif c["metodo"] == "entorno":
            # la bola es lo que difiere de su entorno en la misma imagen (promedio de un cuadrado de
            # ~1.4 bolas): al inclinarse la placa cambia su sombreado entero, pero eso no afecta a
            # esta comparacion; con la placa vacia guardada si (se inundaba toda la placa)
            k = int(1.4 * d_px) | 1
            dif = cv2.absdiff(zona, cv2.blur(zona, (k, k), borderType=cv2.BORDER_REFLECT))
            dif = cv2.GaussianBlur(dif, (3, 3), 0) * mascara
            if int(dif.max()) < c["diferencia_min"]:
                return self.perder(0)
            u = c["diferencia_min"]
            binaria = dif > u
        elif self.fondo is not None:
            # la bola es lo que cambio respecto a la placa vacia (mas clara o mas oscura)
            dif = cv2.absdiff(zona, self.fondo[y0:y1, x0:x1])
            dif = cv2.GaussianBlur(dif, (3, 3), 0) * mascara
            pico = int(dif.max())
            if pico < c["diferencia_min"]:
                return self.perder(0)
            # umbral fijo: con la placa inclinada sus bordes se corren y el fondo blanco de la mesa da
            # diferencias de ~200; un umbral relativo al maximo subia tanto que la bola quedaba debajo
            u = c["diferencia_min"]
            binaria = dif > u
        else:
            # sin placa vacia: lo mas brillante, como en el ESP32
            bx0, by0, bx1, by1 = self.placa.caja
            fondo = int(np.median(p[by0:by1:4, bx0:bx1:4]))
            pico = int((zona * mascara).max())
            if pico - fondo < c["contraste_min"]:
                return self.perder(0)
            u = fondo + int(c["umbral_fraccion"] * (pico - fondo))
            binaria = (zona > u) & (mascara > 0)

        # una bola gris parecida a la placa solo destaca por su contorno (un anillo partido): un cierre
        # del tamano de un cuarto de bola une los trozos y rellena el anillo hasta dejar un disco
        k = max(3, int(d_px / 4) | 1)
        binaria = cv2.morphologyEx(binaria.astype(np.uint8), cv2.MORPH_CLOSE,
                                   cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
        n, _, st, cen = cv2.connectedComponentsWithStats(binaria, connectivity=4)
        trozos = [(cen[i][0] + x0, cen[i][1] + y0, st[i][0] + x0, st[i][1] + y0, st[i][2], st[i][3], st[i][4])
                  for i in range(1, n) if st[i][4] >= 3]
        trozos = sorted(trozos, key=lambda t: -t[6])[:30]   # con muchas manchas, solo las 30 mayores
        mejor = None
        if trozos:
            # mismo criterio que antes (grupo = trozos a menos de radio_union de cada trozo), pero con
            # matrices: el bucle en Python con ~30 trozos costaba mas de 2 ms por imagen
            T = np.array(trozos, dtype=np.float64)
            tx, ty, ar = T[:, 0], T[:, 1], T[:, 6]
            M = (tx[:, None] - tx[None, :]) ** 2 + (ty[:, None] - ty[None, :]) ** 2 <= radio_union ** 2
            A = M @ ar
            gw = np.where(M, T[:, 2] + T[:, 4], -1e9).max(1) - np.where(M, T[:, 2], 1e9).min(1)
            gh = np.where(M, T[:, 3] + T[:, 5], -1e9).max(1) - np.where(M, T[:, 3], 1e9).min(1)
            vale = ((A >= area_min) & (A <= area_max) & (np.maximum(gw, gh) <= 2 * np.minimum(gw, gh) + 1)
                    & (A * 20 >= 7 * gw * gh))
            if vale.any():
                gx, gy = (M @ (tx * ar)) / A, (M @ (ty * ar)) / A
                puntaje = ((gx - prediccion[0]) ** 2 + (gy - prediccion[1]) ** 2) if prediccion else -A
                i = int(np.argmin(np.where(vale, puntaje, np.inf)))
                mejor = (float(gx[i]), float(gy[i]), float(A[i]))
        if mejor is None:
            return self.perder(u)
        px, py, a = mejor
        if prediccion and np.hypot(px - prediccion[0], py - prediccion[1]) > c["salto_max_mm"] * s:
            return self.perder(u)
        if self.ult is not None and self.perdidas < 5:
            f = 1.0 / (self.perdidas + 1)
            self.vel = (0.5 * self.vel[0] + 0.5 * (px - self.ult[0]) * f,
                        0.5 * self.vel[1] + 0.5 * (py - self.ult[1]) * f)
        else:
            self.vel = (0.0, 0.0)
        self.ult, self.perdidas = (px, py), 0
        return (px, py, a / (s * s), u)

    def perder(self, u):
        self.perdidas += 1
        return (None, None, 0, u)


def cargar_fondo(placa):
    """La placa vacia se guarda enderezada (para verla) y se pasa a la vista de la camara para detectar."""
    f = cv2.imread(ARCHIVO_FONDO, cv2.IMREAD_GRAYSCALE)
    if f is None or f.shape != (placa.h, placa.w):
        return None
    return cv2.warpPerspective(f, placa.H_inv, (config["ancho"], config["alto"]), flags=cv2.INTER_LINEAR)


def imagen_color(yuyv):
    """Cuanto color de la bola tiene cada pixel (0 = nada). El YUYV trae, por cada 2 pixeles, Y U Y V:
    U = azul menos brillo, V = rojo menos brillo (128 = sin color). Se calcula a la mitad del ancho y
    se duplica cada columna para volver a w x h."""
    cb = config["color_bola"]
    if cb == "azul":
        # camino rapido (el habitual): restas y sumas con tope de OpenCV, sin pasar por enteros de 16 bits
        c = cv2.subtract(np.ascontiguousarray(yuyv[:, 1::4]), 128)
        return cv2.GaussianBlur(np.repeat(cv2.add(c, c), 2, axis=1), (3, 3), 0)
    u = yuyv[:, 1::4].astype(np.int16) - 128
    v = yuyv[:, 3::4].astype(np.int16) - 128
    if cb == "azul":
        c = u
    elif cb == "rojo":
        c = v
    elif cb == "amarillo":
        c = -u
    elif cb == "verde":
        c = -(u + v) // 2
    else:
        c = u
    c = np.clip(c * 2, 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(np.repeat(c, 2, axis=1), (3, 3), 0)


def velocidad(ahora):
    """Velocidad de la bola (mm/s) con una recta ajustada a las posiciones de los ultimos
    ventana_vel_s; None si hay muy pocas (recien encontrada o tras perderla)."""
    desde = ahora - config["ventana_vel_s"]
    pts = [p for p in list(historial)[-20:] if p[0] >= desde]
    if len(pts) < 4:
        return None
    # pendiente de la recta de minimos cuadrados, con la formula directa (np.polyfit daba lo mismo,
    # pero tardaba ~3 ms por imagen en la Pi: mas que toda la deteccion)
    n = len(pts)
    tm = sum(p[0] for p in pts) / n
    xm = sum(p[1] for p in pts) / n
    ym = sum(p[2] for p in pts) / n
    stt = sum((p[0] - tm) ** 2 for p in pts)
    if stt <= 0:
        return None
    return (sum((p[0] - tm) * (p[1] - xm) for p in pts) / stt,
            sum((p[0] - tm) * (p[2] - ym) for p in pts) / stt)


def nivelar():
    """Busca el neutro con el que la placa queda horizontal: enciende el PID unos segundos con la bola
    cerca del centro y toma el pulso medio de cada servo. La inclinacion que el PID necesita en
    promedio para sostener la bola es justo la que le falta al neutro. (Mirar hacia donde rueda la
    bola con la placa quieta no sirve: el rozamiento la frena y no ve inclinaciones pequenas.)
    Repite hasta que la correccion es pequena y guarda el neutro en ajustes.json."""
    global modo, ajustes
    estado_pi["nivelacion"] = "empezando"
    for vuelta in range(4):
        # la bola tiene que estar en la placa y lejos de los bordes
        espera = time.time()
        while True:
            p = historial[-1] if historial else None
            if p and time.time() - p[0] < 0.2 and abs(p[1]) < 70 and abs(p[2]) < 80:
                break
            estado_pi["nivelacion"] = "pon la pelota cerca del centro"
            if time.time() - espera > 60:
                modo = 2
                ser.write(b"o2\n")
                estado_pi["nivelacion"] = "cancelada: no vi la pelota cerca del centro en 60 s"
                return
            time.sleep(0.2)
        modo = 1
        ser.write(b"o1\n")
        estado_pi["nivelacion"] = f"vuelta {vuelta + 1}: PID encendido, midiendo 12 s"
        time.sleep(4)            # que el PID asiente la bola
        px, py, vistos = [], [], 0
        for _ in range(80):      # 8 s
            time.sleep(0.1)
            if estado_esp["estado"] & 3 == 3:      # PID con bola
                px.append(estado_esp["pulso_x"]); py.append(estado_esp["pulso_y"]); vistos += 1
        modo = 2
        ser.write(b"o2\n")
        if vistos < 50:
            estado_pi["nivelacion"] = f"vuelta {vuelta + 1}: se perdio la bola, repito"
            continue
        dx = int(round(np.mean(px))) - ajustes["neutro_x"]
        dy = int(round(np.mean(py))) - ajustes["neutro_y"]
        ajustes["neutro_x"] += dx
        ajustes["neutro_y"] += dy
        ser.write(f"n{ajustes['neutro_x']},{ajustes['neutro_y']}\n".encode())
        estado_pi["nivelacion"] = (f"vuelta {vuelta + 1}: correccion ({dx:+d}, {dy:+d}) us -> neutro "
                                   f"{ajustes['neutro_x']}/{ajustes['neutro_y']}")
        if abs(dx) <= 5 and abs(dy) <= 5:
            break
        time.sleep(1.5)
    json.dump(ajustes, open(ARCHIVO_AJUSTES, "w"), indent=2)
    estado_pi["nivelacion"] = f"lista: neutro {ajustes['neutro_x']}/{ajustes['neutro_y']} guardado"


# ---------------------------------------------------------------- sistema de comunicacion
def ejecutar_comando(n):
    """Ejecuta un comando numerico (1..11) venga de la pagina, del myRIO o de TCP.
    Devuelve None si se acepto, o el motivo por el que no."""
    global modo, pedir_fondo
    if 1 <= n <= 4:
        return uav.comando(n)
    if n in (5, 6, 7):
        if ser.s is None:
            return "ESP32 del ball and plate no conectado"
        modo = {5: 1, 6: 2, 7: 0}[n]
        ser.write(f"o{modo}\n".encode())
    elif n == 8:
        return "marcar esquinas solo se puede en la pagina web (clic en las 4 esquinas)"
    elif n == 9:
        pedir_fondo = True
    elif n == 10:
        if ser.s is None:
            return "ESP32 del ball and plate no conectado"
        if not any(t.name == "nivelar" and t.is_alive() for t in threading.enumerate()):
            threading.Thread(target=nivelar, name="nivelar", daemon=True).start()
    elif n == 11:
        if os.path.exists(ARCHIVO_FONDO):
            os.remove(ARCHIVO_FONDO)
    return None


def lectura_bp():
    """Tiempo B&P [ms desde que arranco el programa], posicion X e Y [mm] y si el PID esta activo."""
    p = historial[-1] if historial else None
    visto = p is not None and time.time() - p[0] < 0.1
    activo = int(modo == 1 and ser.s is not None)
    return (int(1000 * (time.time() - T_INICIO)), p[1] if visto else None, p[2] if visto else None, activo)


uav = comunicacion.Uav(PATRON_UAV, os.path.join(CARPETA, "registros"))
enlace = comunicacion.Enlace(ejecutar_comando, uav.leer, lectura_bp, PUERTO_MYRIO, BAUDIOS_MYRIO,
                             al_giro_base=uav.giro_base)


# ---------------------------------------------------------------- pagina web
PAGINA = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Ball and plate</title>
<style>body{font-family:sans-serif;background:#111;color:#eee;margin:16px}
.vistas{display:flex;flex-wrap:wrap;gap:8px}img{max-width:100%;border:1px solid #444}
#cam{width:640px;cursor:crosshair}#pl{height:480px}#col{width:480px}
button{font-size:16px;margin:4px;padding:8px 14px}h4{margin:14px 0 2px}.pel{background:#a22;color:#fff}
#uav{font-family:monospace;font-size:15px;margin:6px 4px}#e{font-family:monospace;white-space:pre;font-size:13px}
#ayuda{color:#fc6}</style></head>
<body><h3>Ball and plate (PS3 Eye + ESP32) &nbsp; <a href="/robot" style="color:#3ccf7a;font-size:15px">Robot completo (placa + dron) →</a> &nbsp; <a href="/dron" style="color:#6af;font-size:15px">Interfaz completa del dron →</a></h3>
<div class="vistas"><img id="cam" src="/video?v=camara"><img id="pl" src="/video?v=placa"><img id="col" src="/video?v=color" title="color de la bola"></div>
<h4>Ball and plate (ESP32_BALL AND PLATE)</h4>
<div><button onclick="c(5)">5 Inicio PID</button><button onclick="c(6)">6 Neutro (placa plana)</button>
<button onclick="c(7)">7 Apagar PID</button> |
<button onclick="esq()">8 Marcar esquinas</button><button onclick="c(9)">9 Guardar placa vacia</button>
<button onclick="c(10)">10 Nivelar placa</button><button onclick="c(11)">11 Borrar placa vacia</button></div>
<h4>Dron (ESP32_UAV)</h4>
<div><button onclick="c(1)">1 Calibrar</button><button onclick="c(2)">2 1er inicio (armar)</button>
<button class="pel" onclick="if(confirm('Se encienden los 8 motores del dron. Zona de helices despejada?'))c(3)">3 2do inicio (motores)</button>
<button onclick="c(4)">4 Parar</button></div>
<div id="uav"></div>
<div id="ayuda"></div><div id="e"></div>
<script>
const nombres=['X- Y+ (arriba izquierda)','X+ Y+ (arriba derecha)','X+ Y- (abajo derecha)','X- Y- (abajo izquierda)'];
let pts=null;
function m(k){fetch('/cmd?k='+k)}
function c(n){fetch('/comando?n='+n).then(r=>r.text()).then(t=>{ayuda.textContent='Comando '+n+': '+t})}
function esq(){pts=[];ayuda.textContent='Clic en la esquina '+nombres[0]}
cam.onclick=ev=>{if(!pts)return;const r=cam.getBoundingClientRect();
 pts.push([Math.round((ev.clientX-r.left)*320/r.width),Math.round((ev.clientY-r.top)*240/r.height)]);
 if(pts.length<4){ayuda.textContent='Clic en la esquina '+nombres[pts.length];return}
 fetch('/esquinas?p='+pts.flat().join(','));ayuda.textContent='Esquinas guardadas: '+JSON.stringify(pts);pts=null};
setInterval(()=>fetch('/estado').then(r=>r.json()).then(d=>{const u=d.uav;
 uav.textContent=u.conectado?('roll '+u.roll.toFixed(2)+'  pitch '+u.pitch.toFixed(2)+'  activo '+u.activo+'  '+u.fase+'  | '+u.mensaje):'ESP32_UAV no conectado';
 e.textContent=JSON.stringify(d,null,1)}),300);
</script></body></html>"""


class Web(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def responder(self, cuerpo, tipo="text/plain"):
        self.send_response(200)
        self.send_header("Content-Type", tipo)
        self.end_headers()
        self.wfile.write(cuerpo)

    def do_GET(self):
        global modo, pedir_fondo
        url = urlparse(self.path)
        q = parse_qs(url.query)
        if url.path == "/video":
            cual = q.get("v", ["camara"])[0]
            if cual not in espectadores:
                cual = "camara"
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            with bloqueo:
                espectadores[cual] += 1
            try:
                ultimo = None
                while True:
                    with bloqueo:
                        jpg = vistas.get(cual)
                    if jpg is not None and jpg is not ultimo:
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                        ultimo = jpg
                    time.sleep(VISTA_CADA_S / 2)
            except (BrokenPipeError, ConnectionResetError):
                return
            finally:
                with bloqueo:
                    espectadores[cual] -= 1
        elif url.path == "/estado":
            datos = {"pi": estado_pi, "modo_pedido": modo,
                     "esp32": dict(estado_esp, conectado=ser.s is not None,
                                   estado=ESTADO_TEXTO.get(estado_esp["estado"], estado_esp["estado"])),
                     "uav": uav.leer(), "comunicacion": enlace.estado(),
                     "ff": {k: (round(v, 1) if isinstance(v, float) else v) for k, v in ff.items() if k != "enviado"}}
            self.responder(json.dumps(datos).encode(), "application/json")
        elif url.path == "/robot":
            self.responder(PAGINA_ROBOT.encode(), "text/html; charset=utf-8")
        elif url.path == "/dron":
            self.responder(PAGINA_DRON.encode(), "text/html; charset=utf-8")
        elif url.path == "/dron/estado":
            try:
                desde = float(q.get("desde", ["0"])[0]); ultimo_msg = float(q.get("msg", ["0"])[0])
            except ValueError:
                desde = ultimo_msg = 0.0
            hist = [p for p in list(uav.historial) if p[0] > desde]
            if desde == 0:
                hist = [p for p in hist if p[0] > time.time() - 30]
            msgs = [m for m in list(uav.mensajes_t) if m[0] > ultimo_msg]
            self.responder(json.dumps({"estado": uav.leer(), "historial": hist, "mensajes": msgs}).encode(),
                           "application/json")
        elif url.path == "/dron/orden":
            o = q.get("o", [""])[0]
            if o == "cero_altura":          # no va al ESP32: solo pone el cero de la altura en la Pi
                uav.cero_altura()
                self.responder(b"OK")
                return
            error = uav.orden(o)
            self.responder(b"OK" if error is None else error.encode())
        elif url.path == "/dron/logs":
            nombres = sorted((os.path.basename(f) for f in glob.glob(os.path.join(CARPETA, "registros", "dron_log_*.csv"))),
                             reverse=True)[:20]
            self.responder(json.dumps(nombres).encode(), "application/json")
        elif url.path == "/dron/log":
            nombre = os.path.basename(q.get("f", [""])[0])
            ruta = os.path.join(CARPETA, "registros", nombre)
            if nombre.startswith("dron_log_") and nombre.endswith(".csv") and os.path.exists(ruta):
                self.send_response(200)
                self.send_header("Content-Type", "text/csv")
                self.send_header("Content-Disposition", f"attachment; filename={nombre}")
                self.end_headers()
                self.wfile.write(open(ruta, "rb").read())
            else:
                self.responder(b"no existe")
        elif url.path == "/comando":
            self.responder(enlace.atender(q.get("n", [""])[0], "pagina web").encode())
        elif url.path == "/cmd":
            modo = {"c": 1, "n": 2, "o": 0}.get(q.get("k", [""])[0], modo)
            ser.write(f"o{modo}\n".encode())
            self.responder(b"ok")
        elif url.path == "/seno":
            # ensayo E2: seno en el plato con el PID apagado. /seno?eje=0&amp=100&f=1 (eje 0=x, 1=y, 2=circulo)
            # /seno?eje=0&amp=100&barrido=1 -> 0,5-1-1,5-2-3-4 Hz, 10 s cada uno; /seno?amp=0 lo detiene
            try:
                eje, amp = int(q.get("eje", ["0"])[0]), float(q.get("amp", ["0"])[0])
                fr = float(q.get("f", ["1"])[0])
            except ValueError:
                eje, amp, fr = 0, 0.0, 1.0
            modo = 2                       # el plato queda en neutro (el reenvio de ajustes manda o2 y no corta el seno)
            ser.write(b"o2\n")
            if q.get("barrido", ["0"])[0] == "1":
                ser.write(f"G{eje},{amp:.0f}\n".encode())
            else:
                ser.write(f"g{eje},{amp:.0f},{fr:.2f}\n".encode())
            print(f"[pi] seno eje={eje} amp={amp:.0f} us f={fr} barrido={q.get('barrido', ['0'])[0]}", flush=True)
            self.responder(b"ok")
        elif url.path == "/nivelar":
            if not any(t.name == "nivelar" and t.is_alive() for t in threading.enumerate()):
                threading.Thread(target=nivelar, name="nivelar", daemon=True).start()
            self.responder(b"ok")
        elif url.path == "/neutro":
            # neutro temporal sin tocar ajustes.json (para pruebas cortas): /neutro?x=1400&y=1380;
            # a los 2 s se vuelve a mandar el de ajustes.json
            try:
                ser.write(f"n{int(q['x'][0])},{int(q['y'][0])}\n".encode())
            except (KeyError, ValueError):
                pass
            self.responder(b"ok")
        elif url.path == "/esquinas":
            v = [int(x) for x in q.get("p", [""])[0].split(",") if x]
            if len(v) == 8:
                config["esquinas"] = [v[0:2], v[2:4], v[4:6], v[6:8]]
                json.dump(config, open(ARCHIVO_CONFIG, "w"), indent=2)   # el bucle lo recarga
                if os.path.exists(ARCHIVO_FONDO):
                    os.remove(ARCHIVO_FONDO)   # la placa vacia de antes ya no coincide: hay que guardarla de nuevo
            self.responder(b"ok")
        elif url.path == "/fondo":
            if q.get("borrar"):
                if os.path.exists(ARCHIVO_FONDO):
                    os.remove(ARCHIVO_FONDO)
            else:
                pedir_fondo = True
            self.responder(b"ok")
        else:
            self.responder(PAGINA.encode(), "text/html")


threading.Thread(target=ThreadingHTTPServer(("", 8080), Web).serve_forever, daemon=True).start()


def dibujar(g, p, placa, det):
    """Vista de la camara con la placa marcada y vista de la placa enderezada con la bola."""
    cam = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    esq = np.int32(config["esquinas"])
    cv2.polylines(cam, [esq], True, (255, 160, 0), 1)
    cv2.polylines(cam, [placa.zona_cam], True, (0, 160, 0), 1)
    for i, (x, y) in enumerate(esq):
        cv2.putText(cam, str(i + 1), (int(x) + 3, int(y) - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 160, 0), 1)
    pl = cv2.cvtColor(p, cv2.COLOR_GRAY2BGR)
    cv2.drawMarker(pl, (placa.w // 2, placa.h // 2), (255, 255, 255), cv2.MARKER_CROSS, 12, 1)
    cv2.arrowedLine(pl, (placa.w // 2, placa.h // 2), (placa.w // 2 + 30, placa.h // 2), (0, 0, 255), 1, tipLength=0.3)
    cv2.arrowedLine(pl, (placa.w // 2, placa.h // 2), (placa.w // 2, placa.h // 2 - 30), (0, 255, 0), 1, tipLength=0.3)
    if det[0] is not None:
        r = int(config["diametro_bola_mm"] * placa.s / 2) + 2
        q = placa.H @ np.array([det[0], det[1], 1.0])
        cv2.circle(pl, (int(q[0] / q[2]), int(q[1] / q[2])), r, (0, 0, 255), 2)
        cv2.circle(cam, (int(det[0]), int(det[1])), int(config["diametro_bola_mm"] * placa.s_cam / 2) + 2, (0, 0, 255), 2)
    e = estado_pi
    texto = f"{e['fps']:.0f} FPS  det {e['detectada']:.0f}%  {config['metodo'] if config['metodo'] in ('entorno', 'color') or e['con_fondo'] else 'brillo'}"
    cv2.putText(cam, texto, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.putText(cam, f"ESP32: {ESTADO_TEXTO.get(estado_esp['estado'], '?')}  servos {estado_esp['pulso_x']}/{estado_esp['pulso_y']}",
                (4, g.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
    if e["x_mm"] is not None:
        cv2.putText(pl, f"X={e['x_mm']:+.0f} Y={e['y_mm']:+.0f} mm", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    calidad = [cv2.IMWRITE_JPEG_QUALITY, 70]
    return cv2.imencode(".jpg", cam, calidad)[1].tobytes(), cv2.imencode(".jpg", pl, calidad)[1].tobytes()


# ---------------------------------------------------------------- bucle principal
def main():
    global config, ajustes, pedir_fondo
    cap = abrir_camara()
    placa = Placa()
    det = Detector(placa, cargar_fondo(placa))
    os.makedirs(os.path.join(CARPETA, "registros"), exist_ok=True)
    nombre = os.path.join(CARPETA, "registros", time.strftime("registro_%Y%m%d_%H%M%S.csv"))
    archivo_registro = open(nombre, "w", newline="")
    registro = csv.writer(archivo_registro)
    registro.writerow(["tiempo_unix", "x_mm", "y_mm", "area_mm2", "pulso_x_us", "pulso_y_us", "estado", "umbral"])
    print(f"[pi] camara {config['ancho']}x{config['alto']} a {config['fps']} fps, ESP32 {'en ' + PUERTO if PUERTO else 'por USB (se busca solo)'}, "
          f"registro {nombre}, web en el puerto 8080", flush=True)
    enviar_ajustes()
    w, h = config["ancho"], config["alto"]
    t_fps, n_fps, n_det, t_det = time.time(), 0, 0, 0.0
    t_vista = t_cfg = 0.0
    fondo_acum, fondo_n = None, 0
    while True:
        ok, raw = cap.read()
        if not ok:
            # sin camara (no conectada o se desenchufo): se vuelve a buscar cada 2 s
            cap.release()
            time.sleep(2)
            cap = abrir_camara()
            continue
        a = time.perf_counter()
        yuyv = raw.reshape(h, w * 2)
        g = yuyv[:, ::2]
        px, py, area, u = det.detectar(g, yuyv)
        if px is not None:
            x_mm, y_mm = placa.camara_a_mm(px, py)
            ahora_det = time.time()
            historial.append((ahora_det, x_mm, y_mm))
            k = config["mm_por_unidad"]
            v = velocidad(ahora_det) if config["velocidad_pi"] else None
            if v is None:
                ser.write(f"p{x_mm / k:.2f},{y_mm / k:.2f}\n".encode())
            else:
                ser.write(f"p{x_mm / k:.2f},{y_mm / k:.2f},{v[0] / k:.1f},{v[1] / k:.1f}\n".encode())
            enviar_neutro()
            n_det += 1
        else:
            x_mm = y_mm = None
            ser.write(b"p\n")
        t_det += time.perf_counter() - a
        n_fps += 1
        ahora = time.time()

        # placa vacia: promedio de 30 imagenes enderezadas
        if pedir_fondo:
            p = placa.enderezar(g)
            fondo_acum = p.astype(np.float32) if fondo_acum is None else fondo_acum + p
            fondo_n += 1
            if fondo_n >= 30:
                cv2.imwrite(ARCHIVO_FONDO, (fondo_acum / fondo_n).astype(np.uint8))
                fondo_acum, fondo_n, pedir_fondo = None, 0, False
                det.fondo = cargar_fondo(placa)
                print("[pi] placa vacia guardada", flush=True)

        estado_pi.update(x_mm=None if x_mm is None else round(x_mm, 1), y_mm=None if y_mm is None else round(y_mm, 1),
                         area_mm2=int(area), umbral=int(u), con_fondo=det.fondo is not None)
        registro.writerow([f"{ahora:.4f}", "" if x_mm is None else f"{x_mm:.1f}", "" if y_mm is None else f"{y_mm:.1f}",
                           int(area), estado_esp["pulso_x"], estado_esp["pulso_y"], estado_esp["estado"], u])
        if ahora - t_fps >= 1:
            estado_pi.update(fps=round(n_fps / (ahora - t_fps), 1), det_ms=round(1000 * t_det / max(n_fps, 1), 2),
                             detectada=round(100 * n_det / max(n_fps, 1)))
            t_fps, n_fps, n_det, t_det = ahora, 0, 0, 0.0
            archivo_registro.flush()
        if ahora - t_vista >= VISTA_CADA_S and any(espectadores.values()):
            t_vista = ahora
            nuevas = {}
            if espectadores["camara"] or espectadores["placa"]:
                nuevas["camara"], nuevas["placa"] = dibujar(g, placa.enderezar(g), placa, (px, py))
            if espectadores["color"]:
                col = imagen_color(yuyv)
                nuevas["color"] = cv2.imencode(".jpg", cv2.applyColorMap(col, cv2.COLORMAP_JET),
                                               [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
            with bloqueo:
                vistas.update(nuevas)
        if ahora - t_cfg >= 2:
            # se reenvian los ajustes cada 2 s (por si el ESP32 se reinicio) y se recargan si cambiaron
            t_cfg = ahora
            if mtime(ARCHIVO_CONFIG) != mtimes[ARCHIVO_CONFIG]:
                mtimes[ARCHIVO_CONFIG] = mtime(ARCHIVO_CONFIG)
                config = cargar(ARCHIVO_CONFIG, CONFIG_DEFECTO)
                aplicar_exposicion()
                placa = Placa()
                det = Detector(placa, cargar_fondo(placa))   # con esquinas nuevas el fondo ya no sirve
            if mtime(ARCHIVO_AJUSTES) != mtimes[ARCHIVO_AJUSTES]:
                mtimes[ARCHIVO_AJUSTES] = mtime(ARCHIVO_AJUSTES)
                ajustes = cargar(ARCHIVO_AJUSTES, {})
            enviar_ajustes()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        ser.write(b"o2\n")  # al salir: placa plana
