"""Metricas de desempeno de AeroTray a partir de los registros de la Raspberry Pi.

Entrada:
  hoja_ensayos.csv  una fila por ensayo:  codigo,inicio,fin,tipo,notas
                    inicio/fin = hora HH:MM:SS del reloj de la LAPTOP/celular (la de los videos)
                    tipo = equilibrio | recuperacion20 | recuperacion50 | (cualquier otro: solo bola)
  registros/        registro_*.csv de la Pi (bola, 187 Hz) y vuelo_*.csv (dron, 10 Hz)
Opciones:
  --desfase S       segundos que hay que SUMAR a la hora de la hoja para llegar a la hora de la Pi
                    (anotar al inicio: hora de la Pi con `date` y hora del celular en el mismo instante)
  --fecha AAAA-MM-DD  (por defecto 2026-11-23)
Salida (en la carpeta resultados/):
  metricas.csv      una fila por ensayo con todas las metricas definidas en METRICAS_Y_PRUEBAS.md
  <codigo>.png      x(t), y(t), |e|(t) con las bandas, y el uso del recorrido de los servos
Uso:
  python3 metricas_pruebas.py hoja_ensayos.csv ruta/a/registros --desfase 0
"""
import argparse, csv, glob, json, math, os, time
import numpy as np

R_UTIL = 110.0 - 13.5       # mm: semiancho corto del plato (220/2) menos el radio de la bola
LIM_US = 220.0              # limite del PID alrededor del neutro (ajustes.json)
NEUTRO = (1386, 1392)       # neutros de los servos (ajustes.json)


def leer_bola(carpeta):
    T, X, Y, E, PX, PY = [], [], [], [], [], []
    for f in sorted(glob.glob(os.path.join(carpeta, "registro_*.csv"))):
        with open(f) as fh:
            r = csv.reader(fh); next(r, None)
            for fila in r:
                try:
                    t = float(fila[0]); e = int(float(fila[6]))
                except (ValueError, IndexError):
                    continue
                T.append(t); E.append(e)
                X.append(float(fila[1]) if fila[1] not in ("", "nan") else np.nan)
                Y.append(float(fila[2]) if fila[2] not in ("", "nan") else np.nan)
                try:
                    PX.append(int(fila[4])); PY.append(int(fila[5]))
                except (ValueError, IndexError):
                    PX.append(0); PY.append(0)
    o = np.argsort(T)
    return [np.array(v)[o] for v in (T, X, Y, E, PX, PY)]


def leer_dron(carpeta):
    filas = []
    for f in sorted(glob.glob(os.path.join(carpeta, "vuelo_*.csv"))):
        with open(f) as fh:
            for x in csv.DictReader(fh):
                try:
                    M = [float(x[f"M{i}_us"]) for i in range(1, 9)]
                    filas.append((float(x["t_pc"]), float(x["roll_deg"]), float(x["pitch_deg"]),
                                  np.mean(M), np.std(M)))
                except (KeyError, ValueError, TypeError):
                    pass
    return np.array(sorted(filas)) if filas else np.zeros((0, 5))


def a_unix(fecha, hhmmss, desfase):
    return time.mktime(time.strptime(f"{fecha} {hhmmss}", "%Y-%m-%d %H:%M:%S")) + desfase


def tiempo_recuperacion(t, e, banda, sostener=2.0):
    """Primer instante desde el inicio en que |e| entra en la banda y se queda `sostener` s."""
    dentro = e <= banda
    for i in range(len(t)):
        if dentro[i]:
            j = np.searchsorted(t, t[i] + sostener)
            if j <= len(t) and np.all(dentro[i:j]):
                return t[i] - t[0]
    return np.nan


def metricas(T, X, Y, E, PX, PY, t0, t1, tipo):
    m = (T >= t0) & (T <= t1)
    act = m & ((E & 1) == 1)                     # PID activo
    vis = act & ((E & 3) == 3) & ~np.isnan(X)    # PID activo con la bola vista
    out = {"muestras_pid": int(act.sum())}
    if vis.sum() < 20:
        out["nota"] = "sin datos de bola en la ventana (revisar horas o --desfase)"
        return out, None
    t = T[vis]; x = X[vis]; y = Y[vis]; e = np.hypot(x, y)
    # perdidas de la bola: tramos > 1 s con el PID activo y sin bola (caida o la retiraron)
    ta = T[act]; va = vis[act]
    perdidas, k = 0, 0
    while k < len(ta):
        if not va[k]:
            j = k
            while j < len(ta) and not va[j]:
                j += 1
            if ta[min(j, len(ta) - 1)] - ta[k] > 1.0:
                perdidas += 1
            k = j
        else:
            k += 1
    ux = np.abs(PX[act] - NEUTRO[0]) / LIM_US; uy = np.abs(PY[act] - NEUTRO[1]) / LIM_US
    u = np.maximum(ux, uy)
    # esfuerzo de control de la bola: RMS de la desviacion del pulso respecto al neutro y actividad del servo
    dux = (PX[act] - NEUTRO[0]).astype(float); duy = (PY[act] - NEUTRO[1]).astype(float)
    tt = T[act]; dur = max(tt[-1] - tt[0], 1e-6)
    esfuerzo_rms = float(np.sqrt(np.mean(dux ** 2 + duy ** 2)))
    actividad = float((np.abs(np.diff(dux)).sum() + np.abs(np.diff(duy)).sum()) / dur)
    out.update({
        "duracion_s": round(float(t1 - t0), 1),
        "bola_vista_pct": round(100 * vis.sum() / act.sum(), 1),
        "e_mediana_mm": round(float(np.median(e)), 1),
        "e_media_mm": round(float(e.mean()), 1),
        "e_rms_mm": round(float(np.sqrt(np.mean(e ** 2))), 1),
        "varianza_mm2": round(float(np.var(e)), 1),
        "e_p95_mm": round(float(np.percentile(e, 95)), 1),
        "e_max_mm": round(float(e.max()), 1),
        "error_rel_pct": round(100 * float(np.sqrt(np.mean(e ** 2))) / R_UTIL, 1),
        "efectividad_10mm_pct": round(100 * float(np.mean(e <= 10)), 1),
        "efectividad_30mm_pct": round(100 * float(np.mean(e <= 30)), 1),
        "efectividad_50mm_pct": round(100 * float(np.mean(e <= 50)), 1),
        "perdidas_bola": perdidas,
        "uso_servo_medio_pct": round(100 * float(np.mean(u)), 1),
        "saturacion_servo_pct": round(100 * float(np.mean(u >= 0.95)), 1),
        "esfuerzo_servo_rms_us": round(esfuerzo_rms, 1),
        "actividad_servo_us_por_s": round(actividad, 0),
    })
    if tipo.startswith("recuperacion"):
        banda = 5.0 if "OFF" in tipo.upper() else 20.0
        e0 = float(np.median(e[:max(3, int(0.05 * len(e)))]))
        ts = tiempo_recuperacion(t, e, banda)
        out.update({"banda_mm": banda, "e_inicial_mm": round(e0, 1),
                    "t_recuperacion_s": None if np.isnan(ts) else round(float(ts), 2),
                    "recupero": bool(not np.isnan(ts))})
        # sobrepaso: cuanto cruza al otro lado del centro en la direccion inicial
        d0 = np.array([x[0], y[0]]); n0 = d0 / (np.linalg.norm(d0) + 1e-9)
        proy = x * n0[0] + y * n0[1]
        out["sobrepaso_mm"] = round(float(max(0.0, -proy.min())), 1)
    return out, (t, x, y, e, T[act], u)


def dron(D, t0, t1):
    if len(D) == 0:
        return {}
    m = (D[:, 0] >= t0) & (D[:, 0] <= t1)
    if m.sum() < 5:
        return {}
    ro, pi = D[m, 1], D[m, 2]
    tilt = np.degrees(np.arccos(np.cos(np.radians(ro)) * np.cos(np.radians(pi))))
    return {"dron_alabeo_rms": round(float(np.sqrt(np.mean(ro ** 2))), 2),
            "dron_cabeceo_rms": round(float(np.sqrt(np.mean(pi ** 2))), 2),
            "dron_incl_max": round(float(tilt.max()), 1),
            "dron_motor_medio_us": round(float(D[m, 3].mean())),
            "dron_esfuerzo_dif_us": round(float(D[m, 4].mean()), 1)}   # dispersion entre motores = par de actitud


def graficar(cod, serie, t0, ruta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t, x, y, e, ta, u = serie
    fig, ax = plt.subplots(3, 1, figsize=(8, 6.5), sharex=True)
    ax[0].plot(t - t0, x, lw=.7, label="x"); ax[0].plot(t - t0, y, lw=.7, label="y")
    ax[0].set_ylabel("posicion (mm)"); ax[0].legend(loc="upper right"); ax[0].axhline(0, c="k", lw=.5)
    ax[1].plot(t - t0, e, lw=.7, c="tab:red")
    for b, c in ((10, "tab:green"), (30, "tab:orange"), (50, "tab:gray")):
        ax[1].axhline(b, ls="--", lw=.7, c=c)
    ax[1].axhline(R_UTIL, ls=":", c="k", lw=.8)
    ax[1].set_ylabel("|e| (mm)")
    ax[2].plot(ta - t0, 100 * u, lw=.6, c="tab:purple"); ax[2].axhline(95, ls="--", c="k", lw=.7)
    ax[2].set_ylabel("uso servo (%)"); ax[2].set_xlabel("tiempo (s)"); ax[2].set_ylim(0, 105)
    ax[0].set_title(cod)
    for a in ax:
        a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(ruta, dpi=150); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hoja"); ap.add_argument("registros")
    ap.add_argument("--desfase", type=float, default=0.0)
    ap.add_argument("--fecha", default="2026-11-23")
    ap.add_argument("--salida", default="resultados")
    a = ap.parse_args()
    os.makedirs(a.salida, exist_ok=True)
    T, X, Y, E, PX, PY = leer_bola(a.registros)
    D = leer_dron(a.registros)
    filas = []
    with open(a.hoja) as fh:
        for f in csv.DictReader(fh):
            if not f.get("codigo") or not f.get("inicio"):
                continue
            t0 = a_unix(a.fecha, f["inicio"].strip(), a.desfase); t1 = a_unix(a.fecha, f["fin"].strip(), a.desfase)
            tipo = (f.get("tipo") or "equilibrio").strip()
            m, serie = metricas(T, X, Y, E, PX, PY, t0, t1, tipo)
            m.update(dron(D, t0, t1))
            fila = {"codigo": f["codigo"], "tipo": tipo, "inicio": f["inicio"], "fin": f["fin"], **m,
                    "notas": f.get("notas", "")}
            filas.append(fila)
            if serie is not None:
                graficar(f["codigo"], serie, t0, os.path.join(a.salida, f"{f['codigo']}.png"))
            print(f["codigo"], {k: v for k, v in m.items() if k in ("e_rms_mm", "error_rel_pct", "efectividad_30mm_pct",
                                                                    "perdidas_bola", "saturacion_servo_pct",
                                                                    "t_recuperacion_s", "nota", "dron_cabeceo_rms")})
    claves = []
    for f in filas:
        for k in f:
            if k not in claves:
                claves.append(k)
    with open(os.path.join(a.salida, "metricas.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=claves); w.writeheader(); w.writerows(filas)
    json.dump(filas, open(os.path.join(a.salida, "metricas.json"), "w"), indent=1, ensure_ascii=False)
    print(f"\n{len(filas)} ensayos -> {a.salida}/metricas.csv")


if __name__ == "__main__":
    main()
