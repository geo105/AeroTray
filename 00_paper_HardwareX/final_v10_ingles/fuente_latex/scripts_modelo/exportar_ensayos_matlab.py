"""Datos de las figuras experimentales para MATLAB (figuras_matlab/datos/):
  bp_cajas.csv, bp_rampa_serie.csv      -> fig_bp_resultados.m    (fig_bp_results.pdf, 17-18/11)
  ensayos23_cajas.csv                    -> fig_ensayos_23nov.m    (fig_ensayos_23nov.pdf)
  bp23_despegue.csv, bp23_marcas.csv     -> fig_bp23_despegue.m    (fig_bp23_despegue.pdf)
Python calcula; MATLAB solo dibuja.
Uso: AEROTRAY_ROOT=<carpeta AeroTray_Repositorio_Completo> python3 exportar_ensayos_matlab.py"""
import os, sys, json, time, csv
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
ROOT = os.environ.get("AEROTRAY_ROOT", os.path.abspath(os.path.join(AQUI, "..", "..", "..", "..", "AeroTray_Repositorio_Completo")))
OUT = os.environ.get("DATOS", os.path.join(AQUI, "..", "figuras_matlab", "datos"))
ENS = os.path.join(ROOT, "08_datos_y_registros", "ensayos_finales_20261123")
os.makedirs(OUT, exist_ok=True)


def W(nombre, cols, filas):
    with open(os.path.join(OUT, nombre), "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(cols)
        for f in filas:
            w.writerow([("%.6g" % v) if isinstance(v, (float, np.floating)) else v for v in f])
    print("  ", nombre, len(filas))


def cajas(d):
    return [float(np.percentile(d, q)) for q in (5, 25, 50, 75, 95)]


# ---------------------------------------------------------------- 17-18/11 (bola con la base quieta y en movimiento)
sys.path.insert(0, os.path.join(ROOT, "00_paper_HardwareX", "scripts_repositorio"))
import matplotlib; matplotlib.use("Agg")
import make_figures as MF                         # reutiliza la seleccion de tramos y videos del paper
met = json.load(open(os.path.join(MF.DAT, "bp_metrics.json")))
D = os.path.join(ROOT, "08_datos_y_registros", "copia_raspberry_pi_20261120", "pi", "registros")
grupos = []
for nom, (f, a) in (("base quieta - dron apagado", ("registro_20261117_162914.csv", "19:21:15")),
                    ("base quieta - dron en ralenti", ("registro_20261117_162914.csv", "19:29:18"))):
    t, x, y, e, px, py = MF.load(os.path.join(D, f))
    t0 = time.mktime(time.strptime("2026-11-17 " + a, "%Y-%m-%d %H:%M:%S"))
    act = (t >= t0) & ((e == 1) | (e == 3)); ta = t[act]; br = np.where(np.diff(ta) > 1)[0]
    t1 = ta[br[0]] if len(br) else ta[-1]
    s = MF.series("20261117", t0, t1); grupos.append((nom, np.hypot(s[1], s[2])))
for sc, nom in (("flat", "en movimiento - llano"), ("ramp", "en movimiento - rampa 10 grados")):
    ds = []
    for v in met["videos_20261118"]:
        if MF.VIDEOS.get(v["video"]) == sc and v["dur_s"] > 20:
            tv = time.mktime(time.strptime(v["video"][4:19], "%Y%m%d_%H%M%S"))
            s = MF.series("20261118", tv, tv + v["dur_s"])
            if s: ds.append(np.hypot(s[1], s[2]))
    grupos.append((nom, np.concatenate(ds)))
W("bp_cajas.csv", ["condicion", "n", "p5", "q1", "mediana", "q3", "p95"], [[n, len(d)] + cajas(d) for n, d in grupos])
tv = time.mktime(time.strptime("20261118_165747", "%Y%m%d_%H%M%S"))
t, x, y = MF.series("20261118", tv, tv + 69.0)
k = np.arange(0, len(t), 3)                       # ~60 Hz es suficiente para dibujar
W("bp_rampa_serie.csv", ["t_s", "x_mm", "y_mm"], np.column_stack([t[k], x[k], y[k]]).tolist())

# ---------------------------------------------------------------- 23/11 (13 ensayos del robot completo)
sys.path.insert(0, os.path.join(ENS, "recuperado_video"))
import analizar_23nov as A
A.REG = ENS
res = json.load(open(os.path.join(ENS, "recuperado_video", "resultados", "ensayos_23nov.json")))
pi = A.leer_pi(); V = {"1729": A.leer_video("telemetria_video_1729.csv"), "1836": A.leer_video("telemetria_video_1836.csv")}
hm = lambda h: time.mktime(time.strptime("2026-11-23 " + h, "%Y-%m-%d %H:%M:%S"))
T_V1, T_V2 = (hm("18:26:55"), hm("18:28:30")), hm("18:36:52")   # motores encendidos segun vuelo_*.csv
COMPLETOS = {"BP23-10", "BP23-11", "BP23-12", "BP23-13"}
filas = []
for e in res["ensayos"]:
    d = pi if e["fuente"].startswith("Pi") else V[e["fuente"].split()[1]]
    k = (d["t"] >= e["t0"] + 3) & (d["t"] <= e["t1"]) & ~np.isnan(d["x"]) & (np.hypot(d["x"], d["y"]) <= 130) & (d["pid"] == 1)
    on = e.get("dron_motores_on_pct")
    if on is None:
        on = 100 if (e["t1"] > T_V1[0] and e["t0"] < T_V1[1]) or e["t1"] > T_V2 else 0
    filas.append([e["ensayo"], int(on > 50), int(e["ensayo"] in COMPLETOS)] + cajas(np.hypot(d["x"][k], d["y"][k]))
                 + [e["x_std_mm"], e["y_std_mm"]])
W("ensayos23_cajas.csv", ["ensayo", "rotores", "completo", "p5", "q1", "mediana", "q3", "p95", "sx_mm", "sy_mm"], filas)

e = [x for x in res["ensayos"] if x["inicio"] == "18:26:29"][0]
k = np.where((pi["t"] >= e["t0"] - 2) & (pi["t"] <= e["t0"] + 100))[0][::3]
W("bp23_despegue.csv", ["t_s", "x_mm", "y_mm", "servo_x_us", "servo_y_us"],
  np.column_stack([pi["t"][k] - e["t0"], pi["x"][k], pi["y"][k], pi["px"][k], pi["py"][k]]).tolist())
W("bp23_marcas.csv", ["t_flotar_s", "t_despegue_s", "neutro_x_us", "tope_x_us"],
  [[hm("18:27:39") - e["t0"], hm("18:28:04") - e["t0"], 1386, 1612]])
