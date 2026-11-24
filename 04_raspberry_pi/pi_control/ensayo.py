# Ensayo del PID en la Pi: aplica ajustes, enciende el PID N s (con apagado automatico) y resume.
# Uso: python3 ensayo.py nombre clave=valor ...   (ej: limite=220 kd=2.0)
import sys, time, urllib.request, csv, glob, os, json
import numpy as np
def get(u): return urllib.request.urlopen("http://localhost:8080" + u).read()
def est(): return json.loads(get("/estado"))
nombre, cambios = sys.argv[1], dict(a.split("=") for a in sys.argv[2:])
a = json.load(open("ajustes.json"))
for k, v in cambios.items(): a[k] = [int(n) for n in v.split(",")] if "," in v else (float(v) if "." in v else int(v))
json.dump(a, open("ajustes.json", "w"), indent=2)
time.sleep(2.5)
R = max(glob.glob("registros/*.csv"), key=os.path.getmtime)
# espera hasta 40 s a que la bola este quieta cerca del centro (alguien la pone ahi)
listo = 0
for _ in range(200):
    p = est()["pi"]
    listo = listo + 1 if p["x_mm"] is not None and abs(p["x_mm"]) < 40 and abs(p["y_mm"]) < 40 else 0
    if listo >= 5: break
    time.sleep(0.2)
else:
    print(f"{nombre:22s} sin bola en el centro: no se ensayo", flush=True); sys.exit()
get("/cmd?k=c"); t0 = time.time(); perdida = 0; motivo = "ok"
while time.time() - t0 < 20:
    time.sleep(0.1); p = est()["pi"]
    if p["x_mm"] is None: perdida += 0.1
    else:
        perdida = 0
        if abs(p["x_mm"]) > 140 or abs(p["y_mm"]) > 175: motivo = "borde"; break
    if perdida >= 1.0: motivo = "perdida"; break
get("/cmd?k=n"); dur = time.time() - t0; time.sleep(0.3)
r = [x for x in csv.DictReader(open(R)) if x["tiempo_unix"][:1].isdigit() and t0 <= float(x["tiempo_unix"]) <= t0 + dur]
t = np.array([float(x["tiempo_unix"]) for x in r]) - t0
X = np.array([float(x["x_mm"]) if x["x_mm"] else np.nan for x in r]); Y = np.array([float(x["y_mm"]) if x["y_mm"] else np.nan for x in r])
e = np.hypot(X, Y); px = np.array([int(x["pulso_x_us"]) for x in r]); py = np.array([int(x["pulso_y_us"]) for x in r])
est_esp = np.array([int(x["estado"]) for x in r])
lim = a["limite"] + 40
sat = np.mean((abs(px - a["neutro_x"]) >= lim) | (abs(py - a["neutro_y"]) >= lim)) * 100
fin = t >= t.max() - 10
llega = t[np.nanargmax(e < 20)] if np.any(e < 20) else float("nan")
print(f"{nombre:22s} {motivo:7s} {t.max():4.1f}s det {np.isfinite(X).mean()*100:3.0f}% | ultimos 10 s: error medio {np.nanmean(e[fin]):5.1f} mm, <=10mm {np.nanmean(e[fin] <= 10)*100:3.0f}%, <=20mm {np.nanmean(e[fin] <= 20)*100:3.0f}% | llega a 20mm en {llega:4.1f}s | servos al tope {sat:3.0f}% | reinicios ESP {int(np.sum(np.diff((est_esp == 0).astype(int)) == 1))}", flush=True)
