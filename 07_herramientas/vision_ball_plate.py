"""Visor ArUco + pelota. Mide X/Y sobre el plano, no altura ni inclinacion 3D.

Usa IDs unicos 0,1,2,3 alrededor de la bandeja, en ese orden. Para cm hay que
medir las distancias entre CENTROS de los marcadores. Solo envia ver/foto a
la camara DSM; nunca manda ordenes a los servos. Negocia USB hasta 921600 con
diagnostico 0.3. La frecuencia de vista no valida un lazo de equilibrio.
"""
import argparse
from collections import Counter
import csv
from datetime import datetime
import io
import json
from pathlib import Path
import threading
import time
import urllib.request

import cv2
import numpy as np
from PIL import Image

RAIZ = Path(__file__).resolve().parent.parent


def crear_detector(umbral_adaptativo=7):
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    moderno = hasattr(cv2.aruco, "ArucoDetector")
    p = cv2.aruco.DetectorParameters() if moderno else cv2.aruco.DetectorParameters_create()
    p.adaptiveThreshWinSizeMin, p.adaptiveThreshWinSizeMax, p.adaptiveThreshWinSizeStep = 3, 53, 4
    p.adaptiveThreshConstant = umbral_adaptativo
    p.minMarkerPerimeterRate = 0.015
    # Margen blanco estrecho sobre fondo negro: no descartar el contorno interior
    # en favor del contorno exterior del papel (OpenCV 4.6 usa 0.05 por defecto).
    p.minMarkerDistanceRate = 0.02
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if moderno:
        return cv2.aruco.ArucoDetector(dic, p).detectMarkers
    return lambda gris: cv2.aruco.detectMarkers(gris, dic, parameters=p)


def detectar_marcadores(frame, detector):
    escala = max(1.0, 960 / frame.shape[1])
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    grande = cv2.resize(gris, None, fx=escala, fy=escala, interpolation=cv2.INTER_CUBIC)
    esquinas, ids, _ = detector(grande)
    cs = [] if ids is None else list(esquinas)
    numeros = [] if ids is None else ids.ravel().tolist()
    cs, numeros = unificar_contornos(cs, numeros)
    if not all(i in numeros for i in range(4)):
        # JPEG pequeno + reflejos producen bordes fragmentados. Una segunda
        # pasada suave recupera esquinas sin reutilizar coordenadas antiguas.
        extras, ids_extra, _ = detector(cv2.GaussianBlur(grande, (3, 3), 0))
        if ids_extra is not None:
            cs, numeros = unificar_contornos([*cs, *extras], [*numeros, *ids_extra.ravel().tolist()])
    if not all(i in numeros for i in range(4)):
        # QVGA desenfocado: recuperar bordes débiles sin subir la tolerancia a
        # errores del diccionario. Solo añadir IDs todavía ausentes.
        nitida = cv2.addWeighted(gris, 1.8, cv2.GaussianBlur(gris, (3, 3), 0), -.8, 0)
        nitida = cv2.resize(nitida, None, fx=escala, fy=escala, interpolation=cv2.INTER_CUBIC)
        extras, ids_extra, _ = detector(nitida)
        if ids_extra is not None:
            faltan = set(range(4)) - set(numeros)
            nuevos = [(c, i) for c, i in zip(extras, ids_extra.ravel().tolist()) if i in faltan]
            cs, numeros = unificar_contornos([*cs, *[c for c, _ in nuevos]], [*numeros, *[i for _, i in nuevos]])
    if not all(i in numeros for i in range(4)):
        # Sombras junto a los marcadores pequeños: ventanas de realce más
        # amplias y dos escalas. Se mantiene la corrección del diccionario.
        nitida = cv2.addWeighted(gris, 1.8, cv2.GaussianBlur(gris, (5, 5), 0), -.8, 0)
        for factor, detector_extra in ((escala, crear_detector(11)), (escala * 4 / 3, detector)):
            grande_extra = cv2.resize(nitida, None, fx=factor, fy=factor, interpolation=cv2.INTER_CUBIC)
            extras, ids_extra, _ = detector_extra(grande_extra)
            if ids_extra is not None:
                faltan = set(range(4)) - set(numeros)
                nuevos = [(c / factor, i) for c, i in zip(extras, ids_extra.ravel().tolist()) if i in faltan]
                cs, numeros = unificar_contornos([*cs, *[c * escala for c, _ in nuevos]], [*numeros, *[i for _, i in nuevos]])
            if all(i in numeros for i in range(4)):
                break
    return [c.astype(np.float32) / escala for c in cs], numeros


def unificar_contornos(esquinas, ids):
    """Un papel puede producir dos contornos anidados; dos papeles siguen siendo ambiguos."""
    salida, numeros = [], []
    # El borde exterior del cuadrado negro da centros mas estables que el
    # contorno pequeno del payload que a veces acepta OpenCV con poco contraste.
    for c, idx in sorted(zip(esquinas, ids), key=lambda par: cv2.contourArea(par[0]), reverse=True):
        pts = c.reshape(4, 2)
        centro = pts.mean(axis=0)
        lado = np.linalg.norm(pts-np.roll(pts, 1, axis=0), axis=1).mean()
        misma = False
        for prev, i in zip(salida, numeros):
            if i != idx:
                continue
            p = prev.reshape(4, 2)
            lado_prev = np.linalg.norm(p-np.roll(p, 1, axis=0), axis=1).mean()
            if np.linalg.norm(centro-p.mean(axis=0)) < 0.25*min(lado, lado_prev):
                misma = True
                break
        if not misma:
            salida.append(c)
            numeros.append(idx)
    return salida, numeros


def centro_proyectivo(corners):
    """El promedio de vertices NO es el centro bajo perspectiva: usar diagonales."""
    p = np.column_stack((np.asarray(corners).reshape(4, 2), np.ones(4)))
    c = np.cross(np.cross(p[0], p[2]), np.cross(p[1], p[3]))
    if abs(c[2]) < 1e-8:
        raise ValueError("Marcador degenerado")
    return (c[:2] / c[2]).astype(np.float32)


def geometria(esquinas, ids, ancho=None, alto=None):
    counts = Counter(ids)
    repetidos = [i for i in range(4) if counts[i] > 1]
    faltan = [i for i in range(4) if counts[i] == 0]
    if repetidos:
        return None, None, f"IDs repetidos: {repetidos}. Usa uno de cada ID 0,1,2,3."
    if faltan:
        return None, None, f"Faltan IDs: {faltan}. Mostrar los cuatro completos."
    try:
        centros = {i: centro_proyectivo(c) for c, i in zip(esquinas, ids) if i in range(4)}
    except ValueError:
        return None, None, "Marcador degenerado; revisar imagen."
    pts = np.array([centros[i] for i in range(4)], np.float32)
    if not cv2.isContourConvex(pts) or abs(cv2.contourArea(pts)) < 200:
        return None, None, "Revisar orden 0 -> 1 -> 2 -> 3 alrededor del plato."
    w, h = (ancho, alto) if ancho is not None else (2.0, 2.0)
    mundo = np.array([[-w/2, h/2], [w/2, h/2], [w/2, -h/2], [-w/2, -h/2]], np.float32)
    H = cv2.getPerspectiveTransform(pts, mundo)
    if not np.isfinite(H).all() or abs(np.linalg.det(H)) < 1e-12:
        return None, None, "Perspectiva no valida."
    return H, pts, "Cuatro esquinas OK"


def poligono_busqueda(pts, H, margen_cm, ancho=None, alto=None):
    """Zona de busqueda de la pelota en la imagen.

    Los ArUco estan en las esquinas, pero la superficie util llega mas alla
    de la linea que une sus CENTROS. Con margen > 0 se amplia el rectangulo
    en el plano (cm reales, via homografia inversa) para no perder la pelota
    cuando rueda junto a un borde.
    """
    if not margen_cm or H is None:
        return np.round(pts).astype(np.int32)
    w, h = (ancho, alto) if ancho is not None else (2.0, 2.0)
    escala = 1.0 if ancho is not None else 2.0 / 13.6  # normalizada: aproximar
    m = margen_cm * escala
    mundo = np.array([[[-w/2-m, h/2+m], [w/2+m, h/2+m], [w/2+m, -h/2-m], [-w/2-m, -h/2-m]]], np.float32)
    img = cv2.perspectiveTransform(mundo, np.linalg.inv(H))[0]
    return np.round(img).astype(np.int32)


def detectar_bola(frame, esquinas, pts, H, args):
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    roi = np.zeros(gris.shape, np.uint8)
    cv2.fillConvexPoly(roi, poligono_busqueda(pts, H, getattr(args, "margen_cm", 0.0),
                                              getattr(args, "ancho_cm", None), getattr(args, "alto_cm", None)), 255)
    roi = cv2.erode(roi, np.ones((5, 5), np.uint8))
    # Excluir codigos Y bordes blancos, que el programa anterior confundia con bola.
    for c in esquinas:
        p = c.reshape(4, 2)
        p = (p-p.mean(axis=0))*1.65+p.mean(axis=0)
        cv2.fillConvexPoly(roi, np.round(p).astype(np.int32), 0)
    if args.modo == "metal":
        # Suprimir iluminacion que cambia lentamente y conservar la esfera
        # brillante. Cerrar pequenos huecos entre reflejos de su superficie.
        k = args.ventana_local
        local = cv2.morphologyEx(gris, cv2.MORPH_TOPHAT,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
        _, mask = cv2.threshold(local, args.contraste, 255, cv2.THRESH_BINARY)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    elif args.modo == "color":
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), np.array(args.hsv_min), np.array(args.hsv_max))
    else:
        tipo = cv2.THRESH_BINARY if args.modo == "clara" else cv2.THRESH_BINARY_INV
        _, mask = cv2.threshold(gris, args.umbral, 255, tipo)
    mask = cv2.bitwise_and(mask, roi)
    if getattr(args, "excluir_rojo", True):
        # El marco rojo de la bandeja es brillante tras el top-hat y se fusiona
        # con la pelota cuando esta rueda junto al borde (deja de ser circular).
        # Pelota metalica: H~70-80, S<40. Marco: H<15 o >165, S>40.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        rojo = cv2.inRange(hsv, (0, 42, 60), (15, 255, 255)) | cv2.inRange(hsv, (165, 42, 60), (179, 255, 255))
        rojo = cv2.dilate(rojo, np.ones((3, 3), np.uint8))
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(rojo))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidatos = []
    for c in contornos:
        a, per = cv2.contourArea(c), cv2.arcLength(c, True)
        if not args.area_min <= a <= args.area_max or per <= 0:
            continue
        circularidad = 4*np.pi*a/per**2
        if circularidad < args.circularidad:
            continue
        borde = cv2.drawContours(np.zeros_like(roi), [c], -1, 255, 1)
        borde = cv2.dilate(borde, np.ones((3, 3), np.uint8))
        if np.any((borde > 0) & (roi == 0)):
            continue
        m = cv2.moments(c)
        if m["m00"] <= 0:
            continue
        uv = (m["m10"]/m["m00"], m["m01"]/m["m00"])
        xy = cv2.perspectiveTransform(np.array([[uv]], np.float32), H)[0, 0]
        candidatos.append({"uv": uv, "xy": xy.tolist(), "area": float(a)})
    args._diagnostico_bola = {
        "contraste_usado": args.contraste if args.modo == "metal" else None,
        "contornos": len(contornos), "candidatos": len(candidatos),
        "areas_px": [round(float(cv2.contourArea(c)), 1) for c in contornos],
    }
    if len(candidatos) != 1:
        return None, mask, "Sin pelota" if not candidatos else f"Ambigua: {len(candidatos)} posibles pelotas"
    return candidatos[0], mask, "Pelota detectada"


def detectar_bola_adaptativa(frame, esquinas, pts, H, args):
    """Primero entre los centros ArUco; solo si ahi no hay pelota, ampliar
    la busqueda --margen-cm hacia los bordes (pelota pegada al marco)."""
    margen = getattr(args, "margen_cm", 0.0)
    args.margen_cm = 0.0
    try:
        bola, mask, estado = _detectar_bola_contraste(frame, esquinas, pts, H, args)
    finally:
        args.margen_cm = margen
    if bola is None and margen and "Ambigua" not in estado:
        diag = args._diagnostico_bola
        b2, m2, e2 = _detectar_bola_contraste(frame, esquinas, pts, H, args)
        if b2 is not None:
            return b2, m2, e2 + " (borde)"
        args._diagnostico_bola = diag
    return bola, mask, estado


def _detectar_bola_contraste(frame, esquinas, pts, H, args):
    """Recupera reflejos unidos al fondo; nunca elige entre varias pelotas.

    La recuperación exige una única esfera consistente en dos umbrales
    consecutivos. No reutiliza una posición antigua ni prefiere el centro.
    """
    bola, mask, estado = detectar_bola(frame, esquinas, pts, H, args)
    if (bola is not None or args.modo != "metal" or
            not getattr(args, "auto_contraste", False) or "Ambigua" in estado):
        return bola, mask, estado
    original = args.contraste
    diagnostico = args._diagnostico_bola.copy()
    previa = None
    try:
        for contraste in range(original + 5, min(original + 20, 254) + 1, 5):
            args.contraste = contraste
            candidata, mascara, mensaje = detectar_bola(frame, esquinas, pts, H, args)
            if candidata is None:
                previa = None
                continue
            if previa is not None:
                distancia = np.linalg.norm(np.subtract(candidata["uv"], previa["uv"]))
                razon = candidata["area"] / previa["area"]
                if distancia <= 3 and 0.65 <= razon <= 1.5:
                    return candidata, mascara, f"Pelota detectada (contraste {contraste})"
            previa = candidata
    finally:
        args.contraste = original
    args._diagnostico_bola = diagnostico
    return bola, mask, estado



class FuenteUSB:
    def __init__(self, puerto, baudrate=2000000):
        import serial
        self.port = serial.Serial(port=None, baudrate=115200, timeout=0.15, write_timeout=1, exclusive=True)
        self.port.dtr = self.port.rts = False
        self.port.port = puerto
        self.rapido = False
        try:
            self.port.open()
            state = {}
            for intento in range(4):
                self.port.baudrate = 115200
                self.rapido = False
                deadline = time.monotonic() + 2.8
                while time.monotonic() < deadline:
                    self.port.read(2048)
                state = self.leer_estado(5)
                if baudrate != 115200 and state.get("baud_max", 115200) >= baudrate:
                    state = self._negociar_baud(baudrate, state)
                elif baudrate != 115200:
                    print("# Firmware anterior: conserva 115200; requiere diagnostico 0.3 para USB rapido.", flush=True)
                try:
                    self.read()
                    break
                except Exception:
                    if intento == 3:
                        raise
                    print(f"# Re-sincronizando sensor OV2640 (intento {intento+1}/3)...", flush=True)
                    self.port.dtr = False
                    self.port.rts = True
                    time.sleep(0.12)
                    self.port.rts = False
                    self.port.dtr = False
                    self.port.reset_input_buffer()
            print("# Camara:", json.dumps(state), flush=True)
            print(f"# USB a {self.port.baudrate} baud", flush=True)
        except Exception:
            self.close()
            raise

    def leer_estado(self, timeout):
        self.port.write(b"ver\n")
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            line = self.port.readline()
            if not line.startswith(b"{"):
                continue
            try:
                state = json.loads(line)
            except (ValueError, UnicodeDecodeError):
                continue
            if state.get("version") in ("camara-diag-0.2", "camara-diag-0.3", "camara-diag-0.4", "camara-diag-0.5"):
                return state
        raise RuntimeError("No responde camara DSM. Revisa puerto y cierra otros monitores.")

    def cambiar_baud(self, baudrate):
        self.port.write(f"baud {baudrate}\n".encode())
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            if self.port.readline().strip() == f"# BAUD {baudrate}".encode():
                self.port.baudrate = baudrate
                self.rapido = baudrate != 115200
                time.sleep(0.12)
                return
        raise RuntimeError("No se confirmo el cambio de velocidad USB")

    def _reset_hardware(self):
        """Pulso EN por RTS: el ESP32-S3 vuelve a 115200 y reinicia el OV2640."""
        self.port.baudrate = 115200
        self.rapido = False
        self.port.dtr = False
        self.port.rts = True
        time.sleep(0.12)
        self.port.rts = False
        deadline = time.monotonic() + 2.8
        while time.monotonic() < deadline:
            self.port.read(2048)

    def _negociar_baud(self, pedido, state):
        """Prueba 2 M -> 1.5 M -> 921600 y se queda con la primera que entregue
        3 JPEG validos seguidos. Con 2 Mbaud el JPEG QVGA tarda ~35 ms en vez de ~80 ms."""
        maximo = min(pedido, state.get("baud_max", 921600))
        for b in [v for v in (2000000, 1500000, 921600, 460800) if v <= maximo]:
            try:
                self.cambiar_baud(b)
                state = self.leer_estado(2)
                if state.get("baudrate") != b:
                    raise RuntimeError("baud no confirmado")
                for _ in range(3):
                    self.read()
                return state
            except Exception as e:
                print(f"# USB a {b} baud no estable ({e}); probando menor...", flush=True)
                self._reset_hardware()
                state = self.leer_estado(5)
        return state

    def _drenar_fin_foto(self, timeout=0.35):
        fin = time.monotonic() + timeout
        while time.monotonic() < fin:
            l = self.port.readline()
            if not l or b"# FIN_FOTO" in l:
                break
        self.port.reset_input_buffer()

    def read(self):
        if self.port.in_waiting:
            self.port.reset_input_buffer()
        self.port.write(b"foto\n")
        deadline, size = time.monotonic()+0.6, 0
        while time.monotonic() < deadline:
            line = self.port.readline()
            if line.startswith(b"# FOTO_JPEG "):
                size = int(line.split()[2])
                break
            if line.startswith(b"# ERROR_FOTO"):
                raise RuntimeError(line.decode(errors="replace").strip())
        if not 1 <= size <= 128*1024:
            self.port.reset_input_buffer()
            raise RuntimeError("No llego cabecera JPEG valida")
        # El CH340 pierde bytes de vez en cuando (~2 %). No esperar a bytes que
        # nunca llegaran: leer lo disponible hasta ver el marcador FIN_FOTO y
        # comparar longitudes. Un fallo cuesta ~1 foto (35 ms), no ~1.1 s.
        fin = b"\n# FIN_FOTO"
        data = bytearray()
        deadline = time.monotonic() + 0.08 + 2.5*size*10/max(self.port.baudrate, 1)
        pos = -1
        while time.monotonic() < deadline:
            n = self.port.in_waiting
            chunk = self.port.read(n if n else 1)
            if chunk:
                inicio = max(0, len(data) - len(fin))
                data.extend(chunk)
                pos = data.find(fin, inicio)
                if pos >= 0:
                    break
        if pos < 0:
            self.port.reset_input_buffer()
            raise RuntimeError("JPEG incompleto; revisar USB")
        jpeg = bytes(data[:pos])
        if len(jpeg) != size or jpeg[:2] != b"\xff\xd8" or jpeg[-2:] != b"\xff\xd9":
            raise RuntimeError("JPEG incompleto; revisar USB")
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("JPEG no decodificable")
        return frame

    def close(self):
        if self.port.is_open:
            if self.rapido:
                try:
                    self.cambiar_baud(115200)
                except (OSError, RuntimeError):
                    pass  # Un reset fisico siempre recupera 115200.
            self.port.close()


class Captura:
    """Mantiene el visor receptivo durante la transferencia USB, sin cola de fotos."""
    def __init__(self, leer, cerrar):
        self.leer, self.cerrar = leer, cerrar
        self.lock, self.parar = threading.Lock(), threading.Event()
        self.ultimo = (0, None, 0.0, 0.0, "Esperando camara")
        self.hilo = threading.Thread(target=self.run, daemon=True)
        self.hilo.start()

    def run(self):
        seq = 0
        try:
            while not self.parar.is_set():
                t0 = time.monotonic()
                try:
                    frame = self.leer()
                    if frame is None:
                        raise RuntimeError("Sin imagen")
                    seq += 1
                    nuevo = (seq, frame, time.monotonic(), time.monotonic()-t0, "")
                except Exception as error:
                    nuevo = (seq, None, 0.0, 0.0, str(error))
                with self.lock:
                    self.ultimo = nuevo
                self.parar.wait(0.002 if nuevo[1] is not None else 0.5)
        finally:
            self.cerrar()

    def get(self):
        with self.lock:
            return self.ultimo

    def close(self):
        self.parar.set()
        self.hilo.join(timeout=22)


def procesar(frame, detector, args, dibujar=True):
    esquinas, ids = detectar_marcadores(frame, detector)
    H, pts, estado = geometria(esquinas, ids, args.ancho_cm, args.alto_cm)
    ahora = time.monotonic()
    observada = H is not None
    origen = "observada" if observada else "ausente"
    if observada and getattr(args, "memoria_aruco", False):
        args._geom_previa = (H, pts.copy(), ahora, [c.copy() for c in esquinas], list(ids))
    elif not observada and getattr(args, "memoria_aruco", False):
        # No extrapolar un paralelogramo bajo perspectiva: reutilizar la
        # homografia previa SOLO si los ArUco visibles no se han movido.
        previa = getattr(args, "_geom_previa", None)
        vistos = [i for i in ids if i in range(4)]
        if previa and 2 <= len(vistos) == len(set(vistos)) < 4:
            H_p, pts_p, t_p, esquinas_p, ids_p = previa
            desplazamientos = [np.linalg.norm(centro_proyectivo(c) - pts_p[i])
                                for c, i in zip(esquinas, ids) if i in range(4)]
            if ahora - t_p <= .25 and max(desplazamientos) <= 2:
                H, pts = H_p, pts_p
                origen = "memoria"
                estado = "Referencias en memoria (<0.25 s); sin medida para control"
                # Excluir también los papeles de las esquinas que ahora faltan.
                esquinas_roi = esquinas_p
                if len(vistos) == 3:
                    # Bandeja rigida inclinada por el centro: ninguna inclinacion
                    # deja 3 esquinas quietas. 3 ArUco inmoviles (<=2 px) => la
                    # homografia sigue vigente; se renueva y SI vale para control.
                    origen = "memoria3"
                    estado = "3 ArUco inmóviles (4.º tapado): geometría vigente"
                    args._geom_previa = (H_p, pts_p, ahora, esquinas_p, ids_p)
    if origen not in ("memoria", "memoria3"):
        esquinas_roi = [c for c, i in zip(esquinas, ids) if i in range(4)]
    args._diagnostico_bola = {}
    foto, bola, mask = frame.copy() if dibujar else None, None, np.zeros(frame.shape[:2], np.uint8)
    if ids and dibujar:
        cv2.aruco.drawDetectedMarkers(foto, esquinas, np.array(ids, np.int32).reshape(-1, 1))
    if H is not None:
        if dibujar:
            cv2.polylines(foto, [pts.astype(np.int32)], True, (255, 210, 0), 2)
        bola, mask, estado_bola = detectar_bola_adaptativa(frame, esquinas_roi, pts, H, args)
        estado += " | "+estado_bola
        if bola and dibujar:
            cv2.circle(foto, tuple(np.round(bola["uv"]).astype(int)), 8, (0, 255, 255), 2)
    return foto, mask, {"ids": ids, "geometria_valida": H is not None, "estado": estado,
                        "geometria_observada": observada, "origen_geometria": origen,
                        "control_valido": (observada or origen == "memoria3") and bola is not None and args.ancho_cm is not None,
                        "diagnostico_bola": args._diagnostico_bola,
                        "centros_cm": [args.ancho_cm, args.alto_cm] if args.ancho_cm else None,
                        "unidad": "cm" if args.ancho_cm else "normalizada", "bola": bola}


def panel(foto, resultado, duracion, edad, error=""):
    vista = cv2.resize(foto, (960, round(foto.shape[0]*960/foto.shape[1])), interpolation=cv2.INTER_NEAREST)
    pie = np.full((180, 960, 3), (30, 24, 20), np.uint8)
    texto = f"IDs: {resultado['ids']} | {resultado['estado']}"
    medida = "Sin medida de pelota"
    if resultado["bola"] and edad < 2 and not error:
        x, y = resultado["bola"]["xy"]
        medida = f"Pelota X={x:+.2f} Y={y:+.2f} ({resultado['unidad']})"
    if edad >= 2 or error:
        medida = "IMAGEN ANTIGUA / SIN CONEXION: no hay medida actual"
    escala = (f"Escala entre centros ArUco: {resultado['centros_cm'][0]:g} x {resultado['centros_cm'][1]:g} cm"
              if resultado.get("centros_cm") else "Para cm: medir distancia entre centros y usar --ancho-cm y --alto-cm")
    fps = resultado.get("fps_recibidos", 1/max(duracion, 1e-6)) if duracion else 0
    lineas = [texto, medida, f"Transferencia: {duracion*1000:.0f} ms | Recepcion: {fps:.1f} FPS | Edad: {edad:.1f} s",
              "q: salir | s: guardar | Observacion sin accionar servos",
              error[:110] if error else escala]
    for k, linea in enumerate(lineas):
        cv2.putText(pie, linea, (12, 27+33*k), cv2.FONT_HERSHEY_SIMPLEX, 0.51, (80, 215, 255), 1, cv2.LINE_AA)
    return np.vstack((vista, pie))


def argumentos(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    fuente = ap.add_mutually_exclusive_group()
    fuente.add_argument("--usb", help="Puerto de camara DSM: /dev/ttyUSB0")
    fuente.add_argument("--url", help="JPEG WiFi: http://192.168.4.1/foto.jpg")
    fuente.add_argument("--imagen", type=Path, help="Foto guardada, sin hardware")
    fuente.add_argument("--camara", type=int, default=0, help="Webcam, predeterminada 0")
    ap.add_argument("--ancho-cm", type=float, help="Distancia real entre centros 0-1 y 3-2")
    ap.add_argument("--alto-cm", type=float, help="Distancia real entre centros 0-3 y 1-2")
    ap.add_argument("--baudrate", type=int, choices=(115200, 460800, 921600, 1500000, 2000000), default=2000000,
                    help="Negociado con diagnostico 0.3; firmware anterior conserva 115200")
    ap.add_argument("--modo", choices=("clara", "oscura", "color", "metal"), default="clara")
    ap.add_argument("--auto-contraste", action="store_true", help="Metal: recuperar contraste con dos umbrales consistentes")
    ap.add_argument("--calibracion", type=Path, help="Perfil guardado del panel; reemplaza los parámetros de visión")
    ap.add_argument("--contraste", type=int, default=25, help="Modo metal: contraste local minimo 1..254")
    ap.add_argument("--ventana-local", type=int, default=41, help="Modo metal: diametro impar mayor que la pelota en pixeles")
    ap.add_argument("--umbral", type=int, default=180)
    ap.add_argument("--hsv-min", type=int, nargs=3, default=[0, 100, 80])
    ap.add_argument("--hsv-max", type=int, nargs=3, default=[179, 255, 255])
    ap.add_argument("--area-min", type=float, default=20)
    ap.add_argument("--area-max", type=float, default=2500)
    ap.add_argument("--circularidad", type=float, default=0.55)
    ap.add_argument("--margen-cm", type=float, default=2.0,
                    help="Ampliar la busqueda de la pelota mas alla de los centros ArUco (cm)")
    ap.add_argument("--sin-ventana", action="store_true")
    ap.add_argument("--frames", type=int, default=0, help="Limite de imagenes, 0 hasta q/Ctrl+C")
    ap.add_argument("--salida", type=Path, default=RAIZ / "estado_camara" / "visor")
    args = ap.parse_args(argv)
    if args.calibracion:
        from calibracion_ball_plate import cargar
        try:
            if not args.calibracion.is_file():
                raise ValueError(f'No existe el perfil: {args.calibracion}')
            config = cargar(args.calibracion)
        except (OSError, ValueError) as e:
            ap.error(str(e))
        for key, value in config.items():
            setattr(args, key, value)
    if (args.ancho_cm is None) != (args.alto_cm is None):
        ap.error("Se necesitan --ancho-cm y --alto-cm juntos")
    if args.ancho_cm is not None and (not np.isfinite([args.ancho_cm, args.alto_cm]).all() or min(args.ancho_cm, args.alto_cm) <= 0):
        ap.error("Las distancias deben ser positivas y finitas")
    if not 0 <= args.umbral <= 255 or not 0 < args.area_min < args.area_max or not 0 <= args.circularidad <= 1 or args.frames < 0:
        ap.error("Revisar umbral 0..255, areas, circularidad 0..1 y frames >= 0")
    if not 1 <= args.contraste <= 254 or not 7 <= args.ventana_local <= 151 or args.ventana_local % 2 == 0:
        ap.error("Contraste 1..254 y ventana-local impar entre 7 y 151")
    limites = [179, 255, 255]
    if any(not 0 <= low <= high <= limit for low, high, limit in zip(args.hsv_min, args.hsv_max, limites)):
        ap.error("HSV requiere 0 <= minimo <= maximo, H <= 179 y S/V <= 255")
    return args


def ejecutar(args):
    detector, captura = crear_detector(), None
    args.salida.mkdir(parents=True, exist_ok=True)
    if args.imagen:
        frame = cv2.imread(str(args.imagen))
        if frame is None:
            raise RuntimeError(f"No se puede leer {args.imagen}")
    else:
        if args.usb:
            src = FuenteUSB(args.usb, args.baudrate)
            leer, cerrar = src.read, src.close
        elif args.url:
            def leer():
                with urllib.request.urlopen(args.url, timeout=3) as response:
                    return cv2.imdecode(np.frombuffer(response.read(128*1024), np.uint8), cv2.IMREAD_COLOR)
            cerrar = lambda: None
        else:
            cap = cv2.VideoCapture(args.camara)
            if not cap.isOpened():
                raise RuntimeError("No se puede abrir webcam")
            leer, cerrar = lambda: cap.read()[1], cap.release
        captura = Captura(leer, cerrar)
    ventana = "DSM - Camara y Ball and Plate"
    prev, n, foto, resultado = -1, 0, None, None
    ultimo_log, ultimo_log_t = None, 0.0
    ultima_foto_guardada = 0.0
    previa_recepcion = None
    t0 = time.monotonic()
    ultima_valida, ultimo_error = t0, ""
    csv_path = args.salida / datetime.now().strftime("medidas_%Y%m%d_%H%M%S.csv")
    print(f"# Registro: {csv_path}. Medicion planar, no altura 3D.", flush=True)
    try:
        if not args.sin_ventana:
            cv2.namedWindow(ventana, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(ventana, 960, 800)
        with csv_path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_recepcion_s", "transferencia_ms", "ids", "geometria_valida", "bola_valida", "x", "y", "unidad", "estado"])
            f.flush()
            while True:
                seq, img, recibida, duracion, error = captura.get() if captura else (0, frame, time.monotonic(), 0, "")
                if error and error != ultimo_error:
                    print(f"# Esperando imagen valida: {error}", flush=True)
                ultimo_error = error
                if img is not None and seq != prev:
                    ultima_valida = time.monotonic()
                    prev = seq
                    t_proceso = time.monotonic()
                    foto, mask, resultado = procesar(img, detector, args)
                    resultado["proceso_ms"] = round((time.monotonic()-t_proceso)*1000, 2)
                    resultado["recibida_local"] = datetime.now().astimezone().isoformat()
                    resultado["transferencia_ms"] = round(duracion*1000, 1)
                    resultado["fps_recibidos"] = round(1/max(recibida-previa_recepcion, 1e-6), 2) if previa_recepcion else 0.0
                    previa_recepcion = recibida
                    bola = resultado["bola"]
                    w.writerow([f"{recibida-t0:.3f}", f"{duracion*1000:.1f}", ";".join(map(str, resultado["ids"])),
                                int(resultado["geometria_valida"]), int(bola is not None),
                                bola["xy"][0] if bola else "", bola["xy"][1] if bola else "", resultado["unidad"], resultado["estado"]])
                    f.flush()
                    mostrado = panel(foto, resultado, duracion, 0)
                    # La ventana recibe cada frame; escribir imagenes solo 2 Hz
                    # evita trabajo de JPEG/disco innecesario en el modo rapido.
                    if time.monotonic()-ultima_foto_guardada >= 0.5 or args.imagen:
                        cv2.imwrite(str(args.salida / "actual.jpg"), mostrado)
                        cv2.imwrite(str(args.salida / "original.jpg"), img)
                        cv2.imwrite(str(args.salida / "mascara.png"), mask)
                        ultima_foto_guardada = time.monotonic()
                    (args.salida / "estado.json").write_text(json.dumps(resultado, indent=2, ensure_ascii=False)+"\n")
                    n += 1
                    resumen = (tuple(sorted(resultado["ids"])), resultado["estado"])
                    if resumen != ultimo_log and time.monotonic()-ultimo_log_t > 1:
                        print(f"# Foto {n}: {resultado['ids']} | {resultado['estado']}", flush=True)
                        ultimo_log, ultimo_log_t = resumen, time.monotonic()
                if foto is not None and not args.sin_ventana:
                    edad = time.monotonic()-recibida if img is not None else float("inf")
                    mostrado = panel(foto, resultado, duracion, edad, error)
                    cv2.imshow(ventana, mostrado)
                if (args.frames and n >= args.frames) or (args.imagen and args.sin_ventana):
                    break
                if args.frames and time.monotonic()-ultima_valida > 25:
                    raise RuntimeError(f"25 segundos sin imagen valida: {error or 'sin respuesta'}")
                if not args.sin_ventana:
                    key = cv2.waitKey(30) & 0xFF
                    if key == ord("q") or cv2.getWindowProperty(ventana, cv2.WND_PROP_VISIBLE) < 1:
                        break
                    if key == ord("s") and foto is not None:
                        cv2.imwrite(str(args.salida / datetime.now().strftime("foto_%Y%m%d_%H%M%S.jpg")), mostrado)
                else:
                    time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if captura:
            captura.close()
        if not args.sin_ventana:
            cv2.destroyAllWindows()


def main():
    try:
        ejecutar(argumentos())
    except (RuntimeError, OSError) as error:
        raise SystemExit(f"ERROR: {error}")
