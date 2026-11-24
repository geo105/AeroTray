#!/usr/bin/env python3
"""Visor en la laptop: muestra la imagen del ESP32-S3, la bola detectada y los FPS.

Teclas de camara: + / - umbral de deteccion fijo, t = umbral automatico, a = exposicion automatica,
1..9 = exposicion fija (corta a larga), q = salir.
Teclas de control:
  c        encender / apagar el control PID (arranca apagado; apagado = servos sin senal)
  n        servos fijos en el neutro (90 grados, placa plana), sin PID; n otra vez = apagar
  g        elegir ganancia a ajustar (Kp, Ki, Kd);  , / .  bajarla / subirla
  h / k    neutro del servo X -10 / +10 us        j / u   neutro del servo Y -10 / +10 us
  x / y    invertir el sentido del servo X / Y    [ / ]   limite de inclinacion -25 / +25 us
  z        consigna al centro
  v        filtro de velocidad siguiente (0.6 / 0.7 / 0.75 / 0.8 / 0.85)
  p        medir el pulso que sale de verdad por el pin de cada servo
Raton: arrastrar con el boton izquierdo = zona de interes (la placa);
       clic derecho = consigna (donde mantener la bola).
Los ajustes de control se guardan en ajustes.json.
"""
import json
import os
import struct
import sys
import time

import cv2
import numpy as np
import serial

PUERTO = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
ESCALA = 4
CAPTURAS = os.environ.get("VISOR_CAPTURAS")  # carpeta donde guardar la ultima imagen (opcional)
COMANDOS = os.environ.get("VISOR_COMANDOS")  # archivo con teclas a ejecutar como si se pulsaran (opcional)
teclas_externas = []
# registro CSV con posicion y servos de cada imagen recibida (para graficar con graficar.py):
# por defecto registros/registro_<fecha>_<hora>.csv junto al visor; VISOR_LOG permite otra ruta
CABECERA_REGISTRO = ("tiempo_unix,cx_px,cy_px,px_bola,pulso_x_us,pulso_y_us,estado,corruptas_s,descartadas_s,"
                     "pin_x_us,pin_y_us,compensa_x_us,compensa_y_us\n")
REGISTRO = os.environ.get("VISOR_LOG") or os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "registros", time.strftime("registro_%Y%m%d_%H%M%S.csv"))
os.makedirs(os.path.dirname(REGISTRO) or ".", exist_ok=True)
registro_nuevo = not os.path.exists(REGISTRO) or os.path.getsize(REGISTRO) == 0
registro = open(REGISTRO, "a")
if registro_nuevo:
    registro.write(CABECERA_REGISTRO)
print(f"[visor] guardando registro en {REGISTRO}", flush=True)
ultima_captura = 0.0
ARCHIVO_ROI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "roi.txt")
ARCHIVO_AJUSTES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ajustes.json")
CAMPOS = 20
TAM_CABECERA = 4 + 2 * CAMPOS

def abrir_puerto():
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PUERTO, 2000000, 0.5
    s.dtr = False  # lineas de reinicio en reposo antes de abrir el puerto
    s.rts = False
    s.open()
    return s


ser = abrir_puerto()
ultimo_cuadro = time.time()
buf = b""
umbral = 200
ultimo_envio_cfg = 0.0
imagenes_rotas = 0  # imagenes que llegaron incompletas por el USB (no afectan al ESP32)
pulso_medido = (0, 0)  # pulso de cada servo medido en su pin por el ESP32 (tecla p)
compensacion = (0, 0)  # termino integral de cada eje en us: lo que compensa una base inclinada

# zona de interes en coordenadas de la camara (160x120): x0, y0, x1, y1
try:
    roi = [int(v) for v in open(ARCHIVO_ROI).read().split(",")]
except (OSError, ValueError):
    roi = [0, 0, 160, 120]
arrastre = None  # punto inicial mientras se dibuja con el raton

# ajustes del control (se guardan en ajustes.json; el control siempre arranca apagado)
ajustes = {"kp": 1.5, "ki": 0.2, "kd": 0.8, "neutro_x": 1500, "neutro_y": 1500, "limite": 250,
           "signo_x": 1, "signo_y": 1, "consigna_x": 0.0, "consigna_y": 0.0,
           "factor_x": 1.0, "factor_y": 1.0, "filtro_v": 0.6, "exposicion": -1, "umbral": 0,
           "reinicio_integral": 0, "friccion": 0,
           "vel_integral": 0, "autoridad_i": 0.5, "ab_alfa": 0, "ab_beta": 0,
           "holgura": [0, 0, 0, 0], "holgura_rampa": 10}
try:
    ajustes.update(json.load(open(ARCHIVO_AJUSTES)))
except (OSError, ValueError):
    pass
control = False
try:
    mtime_ajustes = os.path.getmtime(ARCHIVO_AJUSTES)
except OSError:
    mtime_ajustes = 0
sostener = True  # servos fijos en el neutro, sin PID: al abrir el visor la placa queda plana y la pelota no se cae


def grados(us):
    """Angulo del servo para el pulso, con la escala habitual 500-2500 us = 0-180 grados."""
    return (us - 500) * 180 / 2000
GANANCIAS = ["kp", "ki", "kd"]
ganancia_sel = 0


def guardar_ajustes():
    global mtime_ajustes
    json.dump(ajustes, open(ARCHIVO_AJUSTES, "w"), indent=2)
    mtime_ajustes = os.path.getmtime(ARCHIVO_AJUSTES)


def recargar_si_cambio():
    """Si ajustes.json se edito desde fuera del visor, se recarga y se manda al ESP32."""
    global mtime_ajustes
    try:
        m = os.path.getmtime(ARCHIVO_AJUSTES)
        if m != mtime_ajustes:
            mtime_ajustes = m
            ajustes.update(json.load(open(ARCHIVO_AJUSTES)))
            return True
    except (OSError, ValueError):
        pass
    return False


def enviar_ajustes():
    a = ajustes
    ser.write((f"k{a['kp']:.4f},{a['ki']:.4f},{a['kd']:.4f}\n"
               f"n{a['neutro_x']},{a['neutro_y']}\n"
               f"l{a['limite']}\n"
               f"d{a['signo_x']},{a['signo_y']}\n"
               f"f{a['factor_x']:.3f},{a['factor_y']:.3f}\n"
               f"v{a['filtro_v']:.2f}\n"
               f"e{a['exposicion']}\n"
               f"u{a['umbral']}\n"
               f"w{a['reinicio_integral']}\n"
               f"m{a['friccion']}\n"
               f"i{a['vel_integral']},{a['autoridad_i']}\n"
               f"b{a['ab_alfa']},{a['ab_beta']}\n"
               f"z{a['holgura'][0]},{a['holgura'][1]},{a['holgura'][2]},{a['holgura'][3]},{a.get('holgura_rampa', 10)}\n"
               f"s{a['consigna_x']:.1f},{a['consigna_y']:.1f}\n"
               f"o{1 if control else 2 if sostener else 0}\n").encode())

# centro de la placa y zona de control (igual que en el firmware)
CENTRO_X, CENTRO_Y = 89, 57
ALCANCE_X, ALCANCE_Y = 24, 39


def dibujar_ejes(vista):
    """Centro, ejes X (rojo, derecha +) e Y (verde, arriba +) y zona de control (amarillo)."""
    c = (CENTRO_X * ESCALA, CENTRO_Y * ESCALA)
    z0 = ((CENTRO_X - ALCANCE_X) * ESCALA, (CENTRO_Y - ALCANCE_Y) * ESCALA)
    z1 = ((CENTRO_X + ALCANCE_X) * ESCALA, (CENTRO_Y + ALCANCE_Y) * ESCALA)
    cv2.rectangle(vista, z0, z1, (0, 220, 255), 1)
    cv2.putText(vista, "zona de control", (z0[0] + 4, z1[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1)
    largo_x, largo_y = ALCANCE_X * ESCALA, ALCANCE_Y * ESCALA
    cv2.arrowedLine(vista, (c[0] - largo_x, c[1]), (c[0] + largo_x + 20, c[1]), (60, 60, 255), 2, tipLength=0.08)
    cv2.arrowedLine(vista, (c[0], c[1] + largo_y), (c[0], c[1] - largo_y - 20), (60, 220, 60), 2, tipLength=0.06)
    cv2.putText(vista, "+X", (c[0] + largo_x + 6, c[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 255), 2)
    cv2.putText(vista, "+Y", (c[0] + 8, c[1] - largo_y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 220, 60), 2)
    cv2.circle(vista, c, 4, (255, 255, 255), -1)


def raton(evento, x, y, flags, _):
    global arrastre, roi
    x = x % (160 * ESCALA)  # la ventana tiene dos imagenes lado a lado; las dos usan las mismas coordenadas
    px, py = min(max(x // ESCALA, 0), 160), min(max(y // ESCALA, 0), 120)
    if evento == cv2.EVENT_RBUTTONDOWN:
        cxp = max(-ALCANCE_X, min(ALCANCE_X, x / ESCALA - CENTRO_X))
        cyp = max(-ALCANCE_Y, min(ALCANCE_Y, CENTRO_Y - y / ESCALA))
        ajustes["consigna_x"], ajustes["consigna_y"] = round(cxp, 1), round(cyp, 1)
        guardar_ajustes()
        enviar_ajustes()
        return
    if evento == cv2.EVENT_LBUTTONDOWN:
        arrastre = (px, py)
    elif evento == cv2.EVENT_LBUTTONUP and arrastre:
        x0, x1 = sorted((arrastre[0], px))
        y0, y1 = sorted((arrastre[1], py))
        arrastre = None
        if x1 - x0 >= 4 and y1 - y0 >= 4:
            roi = [x0, y0, x1, y1]
            open(ARCHIVO_ROI, "w").write(",".join(map(str, roi)))
            ser.write(f"r{x0},{y0},{x1},{y1}\n".encode())


VENTANA = "ESP32-S3 OV2640 - ball and plate"
cv2.namedWindow(VENTANA)
cv2.setMouseCallback(VENTANA, raton)


def reconectar():
    global ser, buf, ultimo_cuadro
    print("\n[visor] sin datos del ESP32: reconectando el puerto...", flush=True)
    try:
        ser.close()
    except Exception:
        pass
    while True:
        try:
            ser = abrir_puerto()
            break
        except (serial.SerialException, OSError):
            time.sleep(1)
    buf = b""
    ultimo_cuadro = time.time()
    time.sleep(1.5)  # al abrir el puerto el ESP32 se reinicia
    ser.write(f"r{roi[0]},{roi[1]},{roi[2]},{roi[3]}\n".encode())
    enviar_ajustes()


def leer_cuadro():
    global buf, ultimo_cuadro
    while True:
        i = buf.find(b"FRM!")
        if i >= 0 and len(buf) >= i + TAM_CABECERA:
            (w, h, cx, cy, n, fps10, umb, t_desc, t_det, saltos, max_len, brillo, descartados,
             pulso_x, pulso_y, estado, medido_x, medido_y, int_x, int_y) = struct.unpack(f"<{CAMPOS}H", buf[i + 4:i + TAM_CABECERA])
            fin = i + TAM_CABECERA + w * h
            if len(buf) >= fin + 2:
                img = np.frombuffer(buf[i + TAM_CABECERA:fin], np.uint8).reshape(h, w)
                suma = struct.unpack("<H", buf[fin:fin + 2])[0]
                buf = buf[fin + 2:]
                if int(img.sum(dtype=np.uint32)) & 0xFFFF != suma:
                    # llegaron bytes de menos por el USB: se descarta en vez de mostrarla corrida
                    global imagenes_rotas
                    imagenes_rotas += 1
                    continue
                cx = cx if cx != 0xFFFF else -1
                cy = cy if cy != 0xFFFF else -1
                global pulso_medido
                pulso_medido = (medido_x, medido_y)
                global compensacion
                compensacion = (int_x - 32768, int_y - 32768)
                ultimo_cuadro = time.time()
                return (img, cx, cy, n, fps10 / 10, umb, t_desc, t_det, descartados, max_len, brillo, saltos,
                        pulso_x, pulso_y, estado)
        elif i < 0 and len(buf) > 4:
            buf = buf[-4:]
        try:
            datos = ser.read(max(1, ser.in_waiting))
        except (serial.SerialException, OSError):
            datos = b""
            ultimo_cuadro = 0  # fuerza la reconexion
        buf += datos
        # si el USB se desconecto (p. ej. por ruido de los servos) se vuelve a abrir el puerto
        # y se reenvian todos los ajustes, incluido si el control estaba encendido
        if time.time() - ultimo_cuadro > 3:
            reconectar()


while True:
    (img, cx, cy, n, fps, umb, t_desc, t_det, descartados, max_len, brillo, saltos,
     pulso_x, pulso_y, estado) = leer_cuadro()
    # izquierda: camara completa; derecha: solo el tablero (lo que analiza el ESP32), con las marcas
    completa = cv2.cvtColor(cv2.resize(img, None, fx=ESCALA, fy=ESCALA, interpolation=cv2.INTER_NEAREST),
                            cv2.COLOR_GRAY2BGR)
    dentro = np.zeros_like(img, dtype=bool)
    dentro[roi[1]:roi[3], roi[0]:roi[2]] = True
    vista = cv2.cvtColor(cv2.resize(np.where(dentro, img, 0).astype(np.uint8), None, fx=ESCALA, fy=ESCALA,
                                    interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
    cv2.rectangle(completa, (roi[0] * ESCALA, roi[1] * ESCALA), (roi[2] * ESCALA - 1, roi[3] * ESCALA - 1),
                  (255, 160, 0), 1)
    if cx >= 0:
        cv2.circle(completa, (cx * ESCALA, cy * ESCALA), 12, (0, 0, 255), 2)
    cv2.putText(completa, "camara completa", (8, completa.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1)
    cv2.putText(vista, "tablero (lo que analiza el ESP32)", (8, vista.shape[0] - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    mascara = cv2.resize(((img > umb) & dentro).astype(np.uint8) * 255, None, fx=ESCALA, fy=ESCALA,
                         interpolation=cv2.INTER_NEAREST)
    vista[mascara > 0] = (0.5 * vista[mascara > 0] + (0, 90, 0)).astype(np.uint8)
    cv2.rectangle(vista, (roi[0] * ESCALA, roi[1] * ESCALA), (roi[2] * ESCALA - 1, roi[3] * ESCALA - 1),
                  (255, 160, 0), 1)
    dibujar_ejes(vista)
    if cx >= 0:
        cv2.circle(vista, (cx * ESCALA, cy * ESCALA), 12, (0, 0, 255), 2)
        cv2.drawMarker(vista, (cx * ESCALA, cy * ESCALA), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    if cx >= 0:
        xp, yp = cx - CENTRO_X, CENTRO_Y - cy
        fuera = abs(xp) > ALCANCE_X or abs(yp) > ALCANCE_Y
        pos = f"X={xp:+d} Y={yp:+d}" + (" FUERA" if fuera else "")
    else:
        pos = "sin bola"
    texto = f"ESP32: {fps:.1f} FPS  {pos}  px={n}  umbral={umb}{' auto' if ajustes['umbral'] == 0 else ''}"
    vista = np.hstack([completa, vista])
    cv2.putText(vista, texto, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    tiempos = f"descomprimir {t_desc} us  detectar {t_det} us  descartados {descartados}/s  jpeg max {max_len} B  brillo {brillo & 255}-{brillo >> 8}  corruptas {saltos >> 8}/s (dif max {saltos & 255})  usb rotas {imagenes_rotas}  pin X {pulso_medido[0]}us Y {pulso_medido[1]}us  compensa X {compensacion[0]:+d}us Y {compensacion[1]:+d}us"
    cv2.putText(vista, tiempos, (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    if registro:
        registro.write(f"{time.time():.3f},{cx},{cy},{n},{pulso_x},{pulso_y},{estado},{saltos >> 8},{descartados},"
                       f"{pulso_medido[0]},{pulso_medido[1]},{compensacion[0]},{compensacion[1]}\n")
        registro.flush()

    # consigna (circulo blanco) y estado del control
    cons = (int((CENTRO_X + ajustes["consigna_x"]) * ESCALA) + 160 * ESCALA,
            int((CENTRO_Y - ajustes["consigna_y"]) * ESCALA))
    cv2.circle(vista, cons, 8, (255, 255, 255), 2)
    a = ajustes
    gan = "  ".join((">" if g == GANANCIAS[ganancia_sel] else " ") + f"{g.upper()}={a[g]:.3g}" for g in GANANCIAS)
    est = "CONTROL ON" if estado & 1 else "NEUTRO FIJO" if estado & 4 else "control off"
    lin3 = (f"{est}  {gan}  X={pulso_x}us {grados(pulso_x):.0f}g (n {a['neutro_x']}{'+' if a['signo_x'] > 0 else '-'})"
            f"  Y={pulso_y}us {grados(pulso_y):.0f}g (n {a['neutro_y']}{'+' if a['signo_y'] > 0 else '-'})  lim {a['limite']}  filtro {a['filtro_v']}"
            f"  reinicioI {a['reinicio_integral']}  friccion {a['friccion']}")
    cv2.putText(vista, lin3, (8, vista.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (0, 0, 255) if estado & 1 else (0, 200, 255) if estado & 4 else (200, 200, 200), 1)
    print(f"\r{texto}  {lin3}   ", end="", flush=True)
    cv2.imshow(VENTANA, vista)

    # guardar la imagen cada segundo (sin y con marcas) para revisarla fuera del visor
    if CAPTURAS and time.time() - ultima_captura >= 1:
        ultima_captura = time.time()
        cv2.imwrite(os.path.join(CAPTURAS, "ultima_cruda.png"), img)
        cv2.imwrite(os.path.join(CAPTURAS, "ultima_vista.png"), vista)

    # reenviar zona de interes y ajustes cada 2 s por si el ESP32 se reinicio
    if recargar_si_cambio():
        enviar_ajustes()
    if time.time() - ultimo_envio_cfg >= 2:
        ultimo_envio_cfg = time.time()
        ser.write(f"r{roi[0]},{roi[1]},{roi[2]},{roi[3]}\n".encode())
        enviar_ajustes()

    k = cv2.waitKey(1) & 0xFF
    if ord("A") <= k <= ord("Z"):
        k += 32  # igual con Bloq Mayus activado
    # teclas enviadas desde fuera: se escriben en el archivo VISOR_COMANDOS y se procesan una por cuadro
    if COMANDOS and k == 255:
        try:
            pendientes = open(COMANDOS).read()
            os.remove(COMANDOS)
            teclas_externas.extend(pendientes.strip())
        except OSError:
            pass
        if teclas_externas:
            k = ord(teclas_externas.pop(0).lower())
    if k == ord("q"):
        break
    if k in (ord("+"), ord("=")):
        ajustes["umbral"] = min(254, umb + 5)
        guardar_ajustes()
        enviar_ajustes()
    if k == ord("-"):
        ajustes["umbral"] = max(1, umb - 5)
        guardar_ajustes()
        enviar_ajustes()
    if k == ord("t"):
        # umbral automatico: el ESP32 lo calcula en cada imagen segun la luz
        ajustes["umbral"] = 0
        guardar_ajustes()
        enviar_ajustes()
    if k == ord("a"):
        ajustes["exposicion"] = -1
        guardar_ajustes()
        enviar_ajustes()
    if ord("1") <= k <= ord("9"):
        # exposicion fija: con la automatica, si cambia la luz la placa se aclara y pasa el umbral
        ajustes["exposicion"] = (k - ord("0")) * 60
        guardar_ajustes()
        enviar_ajustes()

    # control
    cambio = True
    if k == ord("c"):
        control = not control
        sostener = False
    elif k == ord("n"):
        sostener = not sostener
        control = False
    elif k == ord("g"):
        ganancia_sel = (ganancia_sel + 1) % len(GANANCIAS)
    elif k in (ord(","), ord(".")):
        g = GANANCIAS[ganancia_sel]
        valor = ajustes[g] * (1.25 if k == ord(".") else 0.8)
        ajustes[g] = round(max(valor, 0.01), 4)
    elif k in (ord("h"), ord("k")):
        ajustes["neutro_x"] += 10 if k == ord("k") else -10
    elif k in (ord("j"), ord("u")):
        ajustes["neutro_y"] += 10 if k == ord("u") else -10
    elif k == ord("x"):
        ajustes["signo_x"] *= -1
    elif k == ord("y"):
        ajustes["signo_y"] *= -1
    elif k in (ord("["), ord("]")):
        ajustes["limite"] = max(25, min(600, ajustes["limite"] + (25 if k == ord("]") else -25)))
    elif k == ord("v"):
        opciones = [0.6, 0.7, 0.75, 0.8, 0.85]
        actual = min(opciones, key=lambda o: abs(o - ajustes["filtro_v"]))
        ajustes["filtro_v"] = opciones[(opciones.index(actual) + 1) % len(opciones)]
    elif k == ord("p"):
        ser.write(b"q\n")
        cambio = False
    elif k == ord("z"):
        ajustes["consigna_x"] = ajustes["consigna_y"] = 0.0
    else:
        cambio = False
    if cambio:
        guardar_ajustes()
        enviar_ajustes()

ser.write(b"o0\n")  # al cerrar el visor, control apagado y placa horizontal
ser.close()
cv2.destroyAllWindows()
