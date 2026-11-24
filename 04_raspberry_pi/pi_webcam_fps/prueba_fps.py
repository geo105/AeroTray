#!/usr/bin/env python3
"""Mide FPS de la webcam en la Raspberry Pi: solo captura, y captura + deteccion de bola.

La deteccion es la misma que en el ESP32: escala de grises, reducir a 160x120,
umbral y centroide de los pixeles brillantes.
"""
import time

import cv2
import numpy as np

DISPOSITIVO = 0
UMBRAL = 200


def abrir(formato, ancho, alto):
    cap = cv2.VideoCapture(DISPOSITIVO, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*formato))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, ancho)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def detectar(frame):
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    peq = cv2.resize(gris, (160, 120), interpolation=cv2.INTER_AREA)
    ys, xs = np.nonzero(peq > UMBRAL)
    if len(xs):
        return xs.mean(), ys.mean()
    return -1, -1


def medir(formato, ancho, alto, con_deteccion, segundos=5):
    cap = abrir(formato, ancho, alto)
    real = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    for _ in range(10):
        cap.read()
    n = 0
    t_proc = 0.0
    t0 = time.time()
    while time.time() - t0 < segundos:
        ok, frame = cap.read()
        if not ok:
            continue
        if con_deteccion:
            a = time.perf_counter()
            detectar(frame)
            t_proc += time.perf_counter() - a
        n += 1
    seg = time.time() - t0
    cap.release()
    modo = "captura + deteccion" if con_deteccion else "solo captura"
    extra = f" | deteccion {1000 * t_proc / max(n, 1):.1f} ms/cuadro" if con_deteccion else ""
    print(f"{formato} {real[0]}x{real[1]} {modo:20s} -> {n / seg:5.1f} FPS{extra}", flush=True)


if __name__ == "__main__":
    print("CPU:", open("/proc/device-tree/model").read().strip("\x00"), flush=True)
    for formato, w, h in [("MJPG", 640, 480), ("YUYV", 640, 480), ("MJPG", 1280, 720)]:
        medir(formato, w, h, False)
        medir(formato, w, h, True)
