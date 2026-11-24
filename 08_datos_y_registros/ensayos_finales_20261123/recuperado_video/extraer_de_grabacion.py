"""Recupera la telemetria de la pagina /robot desde la grabacion de pantalla del 23/11 (18:36:50).
La Pi no estuvo disponible despues para bajar sus CSV: la pagina dibujaba en vivo la bola (mapa de la placa,
250x220 mm) y la inclinacion del dron (circulos de 2, 5 y 12 grados), asi que se miden esos puntos cuadro a cuadro.
Escalas sacadas del codigo de pagina_robot.py: placa e = 210/220 px/mm; dron esc = (230/2-12)/14 px/grado.
Salida: telemetria_video.csv (t_unix, bola_x_mm, bola_y_mm, dron_roll_deg, dron_pitch_deg, pid, dron_on)."""
import cv2, numpy as np, csv, sys, time
VID = sys.argv[1]; SAL = sys.argv[2]; T_INI = float(sys.argv[3]) if len(sys.argv) > 3 else 100; T_FIN = float(sys.argv[4]) if len(sys.argv) > 4 else 2400
import os
t0 = time.mktime(time.strptime("2026-11-23 " + os.environ.get("INICIO", "18:36:50"), "%Y-%m-%d %H:%M:%S"))   # hora de inicio de la grabacion (nombre del archivo)
PC = (993.0, 528.0); EP = 210 / 220                     # centro del mapa de la placa y px/mm
DC = (1359.0, 528.0); ED = (230 / 2 - 12) / 14          # centro del nivel del dron y px/grado

def blob(reg, cols, tol, amin):
    m = np.zeros(reg.shape[:2], bool)
    for c in cols:
        m |= np.all(np.abs(reg.astype(int) - np.array(c[::-1])) <= tol, axis=2)
    n, lab, st, cen = cv2.connectedComponentsWithStats(m.astype(np.uint8))
    if n <= 1: return None
    i = 1 + np.argmax(st[1:, cv2.CC_STAT_AREA])
    return cen[i] if st[i, cv2.CC_STAT_AREA] >= amin else None

c = cv2.VideoCapture(VID); out = []
while True:
    if not c.grab(): break
    t = c.get(cv2.CAP_PROP_POS_MSEC) / 1000
    if t < T_INI: continue
    if t > T_FIN: break
    ok, im = c.retrieve()
    if not ok: continue
    barra = im[190:230, 300:1700].reshape(-1, 3).mean(0)          # barra roja PARAR TODO = pagina /robot visible
    if not (barra[2] > 150 and barra[1] < 90): out.append((t0 + t, "", "", "", "", "", "", 0)); continue
    b = blob(im[413:643, 829:1156], [(60, 207, 122)], 30, 120)
    d = blob(im[413:643, 1196:1523], [(60, 207, 122), (242, 182, 64), (255, 90, 90)], 30, 120)
    bx = by = rr = pp = ""
    if b is not None: bx = round((b[0] + 829 - PC[0]) / EP, 1); by = round(-(b[1] + 413 - PC[1]) / EP, 1)
    if d is not None: rr = round((d[0] + 1196 - DC[0]) / ED, 2); pp = round(-(d[1] + 413 - DC[1]) / ED, 2)
    chip_p = im[132:146, 200:260].reshape(-1, 3).mean(0); chip_d = im[132:146, 362:428].reshape(-1, 3).mean(0)
    pid = int(chip_p[1] > chip_p[2] + 25 and chip_p[1] > chip_p[0])      # verde = placa en PID
    don = int(chip_d[1] > chip_d[2] + 25 and chip_d[1] > chip_d[0])      # verde = dron con motores activos
    out.append((round(t0 + t, 2), bx, by, rr, pp, pid, don, 1))
w = csv.writer(open(SAL, "w", newline="")); w.writerow(["t_unix", "bola_x_mm", "bola_y_mm", "dron_roll_deg", "dron_pitch_deg", "pid", "dron_on", "pagina_visible"]); w.writerows(out)
print(len(out), "cuadros")
