#!/usr/bin/env python3
"""Ver en la laptop la camara PS3 Eye de la Raspberry, sin programas en la Pi (solo v4l2-ctl), para
orientar la camara. Muestra la imagen con una cuadricula y una cruz en el centro.

Uso:  python3 ver_camara.py [ip_de_la_pi]      (q o Esc para salir)
"""
import os
import subprocess
import sys

import cv2
import numpy as np

IP = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("PI_HOST", "raspberrypi.local")
USER = os.environ.get("PI_USER", "pi")
W, H = 320, 240
TAM = W * H * 2  # YUYV: 2 bytes por pixel

orden = ("v4l2-ctl -d /dev/video0 --set-fmt-video=width=320,height=240,pixelformat=YUYV --set-parm=30 "
         "-c auto_exposure=0 && v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=0 --stream-to=-")
ssh = subprocess.Popen(["ssh", "-o", "ConnectTimeout=8", f"{USER}@{IP}", orden],
                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
print("Conectando a la camara de la Pi... (q o Esc para salir)")
buf = b""
while True:
    datos = ssh.stdout.read(65536)
    if not datos:
        print("Se corto la conexion con la Pi")
        break
    buf += datos
    if len(buf) < TAM:
        continue
    # solo la imagen mas reciente (si la red va lenta se descartan las viejas)
    n = len(buf) // TAM
    cuadro = buf[(n - 1) * TAM:n * TAM]
    buf = buf[n * TAM:]
    yuyv = np.frombuffer(cuadro, np.uint8).reshape(H, W, 2)
    img = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUYV)
    img = cv2.resize(img, (W * 2, H * 2), interpolation=cv2.INTER_NEAREST)
    for x in range(0, W * 2, 80):
        cv2.line(img, (x, 0), (x, H * 2), (0, 90, 0), 1)
    for y in range(0, H * 2, 80):
        cv2.line(img, (0, y), (W * 2, y), (0, 90, 0), 1)
    cv2.drawMarker(img, (W, H), (0, 0, 255), cv2.MARKER_CROSS, 40, 2)
    cv2.putText(img, "Placa centrada en la cruz, bordes paralelos a la cuadricula", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.imshow("Camara PS3 Eye (Raspberry)", img)
    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
        break
ssh.terminate()
cv2.destroyAllWindows()
