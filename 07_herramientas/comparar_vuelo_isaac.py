"""Compara un ensayo de vuelo en la guía con lo que predijo el modelo de Isaac Lab (y las otras estimaciones).

Uso:
    python3 comparar_vuelo_isaac.py --prediccion                 # solo la tabla de predicciones (antes de volar)
    python3 comparar_vuelo_isaac.py vuelo_AAAAMMDD_HHMMSS.csv     # registro de la Pi (10 Hz, todo el encendido)
    python3 comparar_vuelo_isaac.py vuelo_*.csv --masa 3.6 --log dron_log_*.csv
Genera <registro>_comparacion.md y <registro>_comparacion.png junto al CSV.

Despegue: se detecta con el láser (subida de la altura vertical > 5 mm sostenida 0,5 s respecto a los
primeros segundos en CONTROL). Si el láser no da medidas válidas, se usa el colectivo en el que se pulsó
«mantener» (objetivo = actual) como estimación, y se avisa.
"""
import argparse
import csv
import json
import math
import os
import statistics as st

G = 9.80665
SIM = "/home/george/DSM/simulacion"
# Curva provisional del firmware (MATLAB): F = 5.3955 N * ((PWM-1140)/860)^1.6 por motor
F_MAX_CURVA, SPIN, SPAN, EXP = 5.3955, 1140, 860, 1.6
X = [0.24749, -0.24749, -0.24749, 0.24749, 0, -0.35, 0, 0.35]       # +X hacia M8 (m), del firmware
Y = [-0.24749, -0.24749, 0.24749, 0.24749, -0.35, 0, 0.35, 0]       # +Y hacia M7


def fuerza(pwm, fmax=F_MAX_CURVA):
    return fmax * max(0.0, (pwm - SPIN) / SPAN) ** EXP


def pwm_para(f, fmax=F_MAX_CURVA):
    return SPIN + SPAN * (f / fmax) ** (1 / EXP)


def isaac():
    """Lo que usó y lo que dio el entrenamiento de Isaac Lab."""
    s = json.load(open(os.path.join(SIM, "supuestos_conjunto_cad.json")))
    ev = {k: json.load(open(os.path.join(SIM, "isaaclab_aerotray", f"evaluacion_{k}.json")))
          for k in ("policy_nominal_v2", "policy_perturbaciones_v2", "referencia")}
    return {
        "masa_kg": s["masa_total_kg"],
        "trim_N": sum(s["trim_empuje_ideal_N"]) / 8,
        "trim_dispersion_N": max(s["trim_empuje_ideal_N"]) - min(s["trim_empuje_ideal_N"]),
        "limite_N": s["limite_empuje_ficticio_N"],
        "recorrido_mm": s["recorrido_simulacion_m"] * 1000,
        "rms_incl_ppo": ev["policy_nominal_v2"]["rms_inclinacion_ultimos_2s_deg"],
        "rms_incl_ppo_pert": ev["policy_perturbaciones_v2"]["rms_inclinacion_ultimos_2s_deg"],
        "rms_incl_clasico": ev["referencia"]["rms_inclinacion_ultimos_2s_deg"],
        "rms_alt_ppo_mm": ev["policy_nominal_v2"]["rms_altura_ultimos_2s_mm"],
        "rms_alt_ppo_pert_mm": ev["policy_perturbaciones_v2"]["rms_altura_ultimos_2s_mm"],
    }


def predicciones(masa):
    i = isaac()
    peso_motor = masa * G / 8
    filas = [
        ("Isaac Lab (trim del modelo, 3,6 kg)", f"{i['trim_N']:.2f} N/motor = {i['trim_N'] / G * 1000:.0f} gf",
         f"{i['trim_N'] / i['limite_N'] * 100:.0f} % de su tope ficticio de {i['limite_N']:.0f} N",
         f"{pwm_para(i['trim_N']):.0f} µs con la curva del firmware"),
        (f"Curva provisional del firmware ({masa:.2f} kg)", f"{peso_motor:.2f} N/motor",
         f"{peso_motor / F_MAX_CURVA * 100:.0f} % de 5,40 N (2000 µs)", f"{pwm_para(peso_motor):.0f} µs"),
        ("Curva del firmware a 1800 µs", f"{fuerza(1800):.2f} N/motor", f"levanta {8 * fuerza(1800) / G:.2f} kg",
         "—"),
        ("Curva del firmware a 1950 µs (máximo)", f"{fuerza(1950):.2f} N/motor", f"levanta {8 * fuerza(1950) / G:.2f} kg",
         "—"),
        ("Auditoría de alimentación (A2212 + 9450, 3,10 kg)", "3,80 N/motor", "≈ 77 % de acelerador", "≈ 1770 µs"),
        ("Ensayo histórico (4 motores, solo ball and plate)", "—", "empezó a subir", "≈ 1850 µs"),
    ]
    return i, filas


def leer_registro(ruta):
    filas = []
    with open(ruta) as f:
        for r in csv.DictReader(f):
            try:
                m = [int(float(r[f"M{k}_us"])) for k in range(1, 9)]
                d = {"t": float(r["t_pc"]), "modo": r["modo"], "roll": float(r["roll_deg"]),
                     "pitch": float(r["pitch_deg"]), "m": m, "col": sum(m) / 8, "accel": float(r["accel_g"]),
                     "laser": int(float(r["laser_mm"])) if r["laser_estado"] == "0" else None,
                     "obj": int(r["colectivo_obj"]) if r.get("colectivo_obj") else None}
            except (KeyError, ValueError):
                continue
            if d["laser"] is not None:
                d["alt"] = d["laser"] * math.cos(math.radians(d["roll"])) * math.cos(math.radians(d["pitch"]))
            else:
                d["alt"] = None
            filas.append(d)
    t0 = filas[0]["t"] if filas else 0
    for d in filas:
        d["t"] -= t0
    return filas


def despegue(filas):
    ctrl = [d for d in filas if d["modo"] == "CONTROL"]
    alts = [d for d in ctrl if d["alt"] is not None]
    if len(alts) >= 20:
        base = st.median(d["alt"] for d in alts[:20])
        seguidos = 0
        for d in ctrl:
            if d["alt"] is not None and d["alt"] - base > 5:
                seguidos += 1
                if seguidos >= 5:                      # 0,5 s a 10 Hz
                    return d, base, "láser"
            else:
                seguidos = 0
        return None, base, "láser (no se detectó subida)"
    # sin láser: primer «mantener» (objetivo constante = colectivo actual) sobre 1500 µs
    for a, b in zip(ctrl, ctrl[1:]):
        if b["obj"] and a["obj"] and b["obj"] < a["obj"] and b["col"] > 1500:
            return b, None, "orden mantener (sin láser válido)"
    return None, None, "sin láser válido ni mantener"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("registro", nargs="?")
    ap.add_argument("--masa", type=float, default=3.6, help="masa que sube por la guía (kg)")
    ap.add_argument("--prediccion", action="store_true")
    a = ap.parse_args()
    i, filas_pred = predicciones(a.masa)
    md = ["# Vuelo en la guía frente a Isaac Lab", "", "## Predicciones antes de volar", "",
          "| Fuente | Empuje por motor | Margen | Hover en PWM |", "|---|---|---|---|"]
    md += [f"| {f[0]} | {f[1]} | {f[2]} | {f[3]} |" for f in filas_pred]
    md += ["", f"Isaac Lab también predijo, con control: inclinación RMS {i['rms_incl_ppo']:.3f}° (PPO), "
           f"{i['rms_incl_ppo_pert']:.3f}° con perturbaciones y {i['rms_incl_clasico']:.3f}° con el control clásico; "
           f"altura RMS {i['rms_alt_ppo_mm']:.2f} mm (PPO) en {i['recorrido_mm']:.0f} mm de guía. "
           "En el modelo el reparto es simétrico: los 8 motores necesitan el mismo empuje "
           f"(diferencia máxima {i['trim_dispersion_N'] * 1000:.0f} mN).", ""]
    if a.prediccion or not a.registro:
        print("\n".join(md))
        return
    filas = leer_registro(a.registro)
    ev, base, metodo = despegue(filas)
    md += ["## Ensayo real", "", f"Registro: `{os.path.basename(a.registro)}` · {len(filas)} muestras · método de despegue: {metodo}.", ""]
    res = {}
    if ev:
        p = ev["col"]
        f_motor = a.masa * G / 8
        fmax_real = f_motor / max(1e-6, ((p - SPIN) / SPAN) ** EXP)
        res.update(pwm_despegue=p, t_despegue=ev["t"], fmax_real=fmax_real, tw_real=8 * fmax_real / (a.masa * G))
        md += ["| Magnitud | Real | Isaac Lab / modelo | Diferencia |", "|---|---|---|---|",
               f"| PWM medio al despegar | {p:.0f} µs | {pwm_para(i['trim_N']):.0f} µs (trim de Isaac con la curva) | {p - pwm_para(i['trim_N']):+.0f} µs |",
               f"| Empuje máximo por motor (2000 µs) | {fmax_real:.2f} N | {i['limite_N']:.1f} N (tope de Isaac) · 5,40 N (curva) | {(fmax_real / i['limite_N'] - 1) * 100:+.0f} % vs Isaac |",
               f"| Relación empuje/peso a 2000 µs | {8 * fmax_real / (a.masa * G):.2f} | {8 * i['limite_N'] / (i['masa_kg'] * G):.2f} | |"]
        # tramo estable: 5 s después del despegue o hasta que cambie el objetivo
        tramo = [d for d in filas if d["modo"] == "CONTROL" and ev["t"] <= d["t"] <= ev["t"] + 5]
    else:
        md += ["No se detectó despegue: el colectivo máximo no alcanzó a levantar el conjunto o el láser no midió.", ""]
        tramo = [d for d in filas if d["modo"] == "CONTROL"][-50:]
    if tramo:
        incl = [math.hypot(d["roll"], d["pitch"]) for d in tramo]
        rms = math.sqrt(st.mean(x * x for x in incl))
        res["rms_incl"] = rms
        md += [f"| Inclinación RMS en el tramo ({len(tramo) / 10:.0f} s) | {rms:.2f}° | {i['rms_incl_ppo']:.3f}° PPO · {i['rms_incl_clasico']:.3f}° clásico · {i['rms_incl_ppo_pert']:.3f}° con perturbaciones | ×{rms / i['rms_incl_ppo_pert']:.0f} respecto al caso con perturbaciones |"]
        alts = [d["alt"] for d in tramo if d["alt"] is not None]
        if len(alts) > 5:
            ra = st.pstdev(alts)
            md += [f"| Altura: variación en el tramo | {ra:.1f} mm (sin control de altura) | {i['rms_alt_ppo_mm']:.2f} mm (PPO con control de altura) | |"]
        # reparto de los motores: desbalance -> centro de masa desplazado
        prom = [st.mean(d["m"][k] for d in tramo) for k in range(8)]
        fz = [fuerza(p_) for p_ in prom]
        tot = sum(fz)
        if tot > 0:
            cx = sum(f * x for f, x in zip(fz, X)) / tot * 1000
            cy = sum(f * y for f, y in zip(fz, Y)) / tot * 1000
            md += [f"| Reparto de los 8 motores | de {min(prom):.0f} a {max(prom):.0f} µs | todos iguales (simétrico) | centro de empuje en x={cx:+.0f} mm, y={cy:+.0f} mm |"]
    md += ["", "Notas: el firmware real no controla la altura (solo el ángulo); Isaac Lab sí lo hacía. "
           "El empuje real sale de suponer que al despegar el empuje iguala al peso (sin rozamiento en la guía), "
           "así que es un valor mínimo."]
    base_ruta = os.path.splitext(a.registro)[0] + "_comparacion"
    open(base_ruta + ".md", "w").write("\n".join(md) + "\n")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        t = [d["t"] for d in filas]
        fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        ax[0].plot(t, [d["col"] for d in filas], label="PWM medio")
        ax[0].plot(t, [d["obj"] if d["obj"] else float("nan") for d in filas], "--", label="objetivo")
        ax[0].axhline(pwm_para(i["trim_N"]), color="tab:red", ls=":", label="hover según Isaac + curva")
        ax[0].set_ylabel("µs"); ax[0].legend()
        ax[1].plot(t, [d["alt"] if d["alt"] is not None else float("nan") for d in filas], color="tab:green")
        ax[1].set_ylabel("altura láser (mm)")
        ax[2].plot(t, [d["roll"] for d in filas], label="roll"); ax[2].plot(t, [d["pitch"] for d in filas], label="pitch")
        ax[2].set_ylabel("°"); ax[2].set_xlabel("tiempo (s)"); ax[2].legend()
        if ev:
            for x_ in ax:
                x_.axvline(ev["t"], color="k", lw=0.8)
        fig.tight_layout(); fig.savefig(base_ruta + ".png", dpi=130)
    except ImportError:
        pass
    print("\n".join(md))
    print(f"\nGuardado: {base_ruta}.md y .png")


if __name__ == "__main__":
    main()
