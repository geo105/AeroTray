"""Ensayos del 23/11 en el robot completo: une el CSV de la Pi (18:23-18:39) con la telemetria recuperada de
las grabaciones de pantalla (17:29-18:31 y 18:36-19:15), valida la extraccion contra la Pi donde se solapan,
separa los ensayos de ball and plate (PID activo y bola vista sin huecos > 1 s) y calcula sus metricas.
Uso: python3 analizar_23nov.py   (desde pruebas_20261123/recuperado_video)"""
import csv, json, os, time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AQUI = os.path.dirname(os.path.abspath(__file__))
REG = os.path.join(AQUI, "..", "registros")
SAL = os.path.join(AQUI, "resultados"); os.makedirs(SAL, exist_ok=True)
R_UTIL = 110.0 - 13.5
hhmmss = lambda t: time.strftime("%H:%M:%S", time.localtime(t))


def leer_video(nombre):
    f = os.path.join(AQUI, nombre)
    if not os.path.exists(f):
        return None
    T, X, Y, RO, PI, PID, DON = [], [], [], [], [], [], []
    for r in csv.DictReader(open(f)):
        if r["pagina_visible"] != "1":
            continue
        T.append(float(r["t_unix"]))
        X.append(float(r["bola_x_mm"]) if r["bola_x_mm"] else np.nan)
        Y.append(float(r["bola_y_mm"]) if r["bola_y_mm"] else np.nan)
        RO.append(float(r["dron_roll_deg"]) if r["dron_roll_deg"] else np.nan)
        PI.append(float(r["dron_pitch_deg"]) if r["dron_pitch_deg"] else np.nan)
        PID.append(int(r["pid"])); DON.append(int(r["dron_on"]))
    return {k: np.array(v) for k, v in zip(("t", "x", "y", "roll", "pitch", "pid", "don"), (T, X, Y, RO, PI, PID, DON))}


def leer_pi():
    r = list(csv.reader(open(os.path.join(REG, "registro_20261123_182316.csv"))))[1:]
    t = np.array([float(a[0]) for a in r]); st = np.array([int(float(a[6])) for a in r])
    x = np.array([float(a[1]) if a[1] not in ("", "nan") else np.nan for a in r])
    y = np.array([float(a[2]) if a[2] not in ("", "nan") else np.nan for a in r])
    return {"t": t, "x": x, "y": y, "pid": ((st & 1) == 1).astype(int), "px": np.array([int(a[4]) for a in r]),
            "py": np.array([int(a[5]) for a in r])}


def validar(v, p):
    """Compara la bola del video con la de la Pi en los instantes comunes (interpolando la Pi)."""
    m = (v["t"] >= p["t"][0]) & (v["t"] <= p["t"][-1]) & ~np.isnan(v["x"])
    if m.sum() < 50:
        return None
    best = None
    for lag in np.arange(-1.5, 1.51, 0.05):           # retardo de la pagina (sondeo + dibujo) respecto a la Pi
        ok = ~np.isnan(p["x"])
        xi = np.interp(v["t"][m] - lag, p["t"][ok], p["x"][ok]); yi = np.interp(v["t"][m] - lag, p["t"][ok], p["y"][ok])
        err = np.hypot(v["x"][m] - xi, v["y"][m] - yi)
        if best is None or np.median(err) < best[1]:
            best = (lag, np.median(err), np.percentile(err, 90), m.sum())
    return {"retardo_pagina_s": round(float(best[0]), 2), "error_mediano_mm": round(float(best[1]), 1),
            "error_p90_mm": round(float(best[2]), 1), "muestras": int(best[3])}


def ensayos(d, fuente, min_dur=20.0, hueco=1.5):
    # control: PID activo y bola vista dentro del plato (a mas de 130 mm del centro la bola ya cayo o esta en el marco)
    ctrl = (d["pid"] == 1) & ~np.isnan(d["x"]) & (np.hypot(d["x"], d["y"]) <= 130)
    idx = np.where(ctrl)[0]
    out = []
    if len(idx) == 0:
        return out
    a = p = idx[0]
    for j in list(idx[1:]) + [None]:
        if j is None or d["t"][j] - d["t"][p] > hueco:
            t0, t1 = d["t"][a], d["t"][p]
            if t1 - t0 >= min_dur:
                k = (d["t"] >= t0 + 3) & (d["t"] <= t1) & ctrl          # se descartan los 3 s de reposicion
                e = np.hypot(d["x"][k], d["y"][k])
                if len(e) > 20 and np.median(e) < 120 and np.std(e) > 0.5:     # bola real (no fija en el borde ni sin servos)
                    fila = {"fuente": fuente, "inicio": hhmmss(t0), "fin": hhmmss(t1), "duracion_s": round(float(t1 - t0), 1),
                            "e_mediana_mm": round(float(np.median(e)), 1), "e_rms_mm": round(float(np.sqrt(np.mean(e ** 2))), 1),
                            "e_p95_mm": round(float(np.percentile(e, 95)), 1),
                            "error_rel_pct": round(100 * float(np.sqrt(np.mean(e ** 2))) / R_UTIL, 1),
                            "efectividad_30mm_pct": round(100 * float(np.mean(e <= 30)), 1),
                            "efectividad_50mm_pct": round(100 * float(np.mean(e <= 50)), 1),
                            "x_std_mm": round(float(np.nanstd(d["x"][k])), 1), "y_std_mm": round(float(np.nanstd(d["y"][k])), 1),
                            "t0": float(t0), "t1": float(t1)}
                    if "roll" in d:
                        kk = (d["t"] >= t0) & (d["t"] <= t1) & ~np.isnan(d["roll"])
                        if kk.sum() > 5:
                            inc = np.hypot(d["roll"][kk], d["pitch"][kk])
                            fila.update(dron_motores_on_pct=round(100 * float(np.mean(d["don"][kk])), 0),
                                        dron_incl_rms_deg=round(float(np.sqrt(np.mean(inc ** 2))), 2),
                                        dron_incl_max_deg=round(float(inc.max()), 1))
                    out.append(fila)
            if j is not None:
                a = j
        if j is not None:
            p = j
    return out


def main():
    pi = leer_pi()
    res = {"validacion": {}, "ensayos": []}
    vids = {"1729": leer_video("telemetria_video_1729.csv"), "1836": leer_video("telemetria_video_1836.csv")}
    for k, v in vids.items():
        if v is not None:
            res["validacion"][k] = validar(v, pi)
    res["ensayos"] += ensayos(pi, "Pi (CSV 187 Hz)")
    for k, v in vids.items():
        if v is None:
            continue
        ts = ensayos(v, f"grabacion {k} (~13 Hz)")
        # no repetir los tramos que ya estan en el CSV de la Pi
        res["ensayos"] += [e for e in ts if e["t1"] < pi["t"][0] or e["t0"] > pi["t"][-1]]
    res["ensayos"].sort(key=lambda e: e["t0"])
    for i, e in enumerate(res["ensayos"], 1):
        e["ensayo"] = f"BP23-{i:02d}"
    json.dump(res, open(os.path.join(SAL, "ensayos_23nov.json"), "w"), indent=1, ensure_ascii=False)
    claves = ["ensayo", "fuente", "inicio", "fin", "duracion_s", "e_mediana_mm", "e_rms_mm", "e_p95_mm", "error_rel_pct",
              "efectividad_30mm_pct", "efectividad_50mm_pct", "x_std_mm", "y_std_mm", "dron_motores_on_pct",
              "dron_incl_rms_deg", "dron_incl_max_deg"]
    with open(os.path.join(SAL, "ensayos_23nov.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=claves, extrasaction="ignore"); w.writeheader(); w.writerows(res["ensayos"])
    print("validacion:", res["validacion"])
    for e in res["ensayos"]:
        print({k: e.get(k) for k in claves})
    # figura: cada ensayo, |e|(t)
    n = len(res["ensayos"])
    if n:
        fig, ax = plt.subplots(n, 1, figsize=(9, 1.6 * n + 0.6), sharex=False)
        ax = np.atleast_1d(ax)
        for a, e in zip(ax, res["ensayos"]):
            d = pi if e["fuente"].startswith("Pi") else vids[e["fuente"].split()[1]]
            k = (d["t"] >= e["t0"]) & (d["t"] <= e["t1"])
            a.plot(d["t"][k] - e["t0"], d["x"][k], lw=.6, label="x"); a.plot(d["t"][k] - e["t0"], d["y"][k], lw=.6, label="y")
            a.axhline(0, c="k", lw=.4); a.set_ylim(-110, 110); a.grid(alpha=.3)
            a.set_ylabel("mm"); a.set_title(f"{e['ensayo']}  {e['inicio']}  ({e['fuente']})  mediana {e['e_mediana_mm']} mm", fontsize=8)
        ax[0].legend(loc="upper right", fontsize=7); ax[-1].set_xlabel("tiempo desde el inicio del ensayo (s)")
        fig.tight_layout(); fig.savefig(os.path.join(SAL, "ensayos_23nov.png"), dpi=120); plt.close(fig)


if __name__ == "__main__":
    main()
