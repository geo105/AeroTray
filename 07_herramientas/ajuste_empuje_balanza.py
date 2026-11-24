#!/usr/bin/env python3
"""Ajuste de la curva PWM -> empuje del octo a partir de lecturas de balanza.

Montaje: dron amarrado a un lastre (balde con agua/arena) que descansa sobre
una balanza. Con los 8 motores al mismo PWM, la balanza baja: empuje = W0 - lectura.

CSV de entrada (ver registros/empuje_balanza_plantilla.csv):
    pwm_us,lectura_g[,vbat_a,vbat_b]
La fila con pwm_us = 1000 (ESC armados, motores quietos) es la referencia W0.

Uso:
    python3 herramientas/ajuste_empuje_balanza.py registros/empuje_balanza_AAAAMMDD.csv --masa 3.2
"""
import argparse
import csv
import sys

import numpy as np

G = 9.80665


def leer(ruta):
    filas = []
    with open(ruta, newline="") as f:
        for r in csv.DictReader(f):
            if not r.get("pwm_us") or not r.get("lectura_g"):
                continue
            filas.append((float(r["pwm_us"]), float(r["lectura_g"])))
    if not filas:
        sys.exit("CSV sin datos (columnas pwm_us, lectura_g)")
    filas.sort()
    return np.array(filas)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--masa", type=float, default=3.2, help="masa de vuelo del dron [kg]")
    ap.add_argument("--motores", type=int, default=8)
    ap.add_argument("--png", default=None, help="guardar grafica (opcional)")
    a = ap.parse_args()

    d = leer(a.csv)
    ref = d[d[:, 0] <= 1000.5]
    if len(ref) == 0:
        sys.exit("Falta la fila de referencia pwm_us = 1000 (motores quietos)")
    w0 = ref[:, 1].mean()
    u = d[d[:, 0] > 1000.5, 0]
    t_g = w0 - d[d[:, 0] > 1000.5, 1]          # empuje total [gf]
    peso_g = a.masa * 1000.0

    print(f"Referencia W0 = {w0:.0f} g  |  masa de vuelo = {peso_g:.0f} g  |  hover = {peso_g/a.motores:.0f} gf por motor")
    print(f"{'PWM us':>7} {'total gf':>9} {'por motor gf':>13} {'por motor N':>12} {'T/W':>6}")
    for ui, ti in zip(u, t_g):
        print(f"{ui:7.0f} {ti:9.0f} {ti/a.motores:13.1f} {ti/a.motores*G/1000:12.3f} {ti/peso_g:6.2f}")

    # Modelo T = k (u - u0)^2  ->  sqrt(T) = sqrt(k) (u - u0), recta en los puntos con empuje claro
    m = t_g > max(30.0, 0.03 * t_g.max())
    if m.sum() < 3:
        print("\nPocos puntos con empuje medible: no se ajusta la curva.")
        return
    p = np.polyfit(u[m], np.sqrt(t_g[m]), 1)
    sk, u0 = p[0], -p[1] / p[0]
    k = sk ** 2
    pred = k * np.clip(u - u0, 0, None) ** 2
    rms = np.sqrt(np.mean((pred[m] - t_g[m]) ** 2))
    print(f"\nAjuste T_total = {k:.5f} (u - {u0:.0f})^2  [gf, us]   error RMS = {rms:.0f} gf")

    u_hover = u0 + np.sqrt(peso_g / k)
    t_1950 = k * (1950 - u0) ** 2
    medido_max = u.max()
    print(f"PWM de hover estimado: {u_hover:.0f} us"
          + ("" if u_hover <= medido_max else "  (EXTRAPOLADO: fuera de lo medido)"))
    print(f"Empuje total a 1950 us: {t_1950:.0f} gf -> T/W = {t_1950/peso_g:.2f}"
          + ("" if medido_max >= 1940 else "  (EXTRAPOLADO)"))
    tw = t_1950 / peso_g
    if tw < 1.3:
        print("AVISO: T/W < 1.3 -> no intentar hover libre; reducir masa o cambiar propulsion.")
    elif tw < 1.6:
        print("AVISO: T/W entre 1.3 y 1.6 -> hover posible pero con poco margen de control.")

    # Tabla para firmware/dron_octo_esp32/actuadores_reales.h (promedio colectivo, solo puntos medidos)
    pts = [(ui, ti / a.motores * G / 1000) for ui, ti in zip(u, t_g) if ti > 0]
    mono = []
    for ui, fi in pts:
        if not mono or (ui > mono[-1][0] and fi > mono[-1][1]):
            mono.append((ui, fi))
    print("\n// Curva promedio por motor (colectiva, 8 motores iguales). Solo puntos medidos.")
    print(f"// Fuente: {a.csv}")
    print("{" + ",".join(f"{{{ui:.0f},{fi:.3f}f}}" for ui, fi in mono) + "}")

    if a.png:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib no disponible: sin grafica")
            return
        uu = np.linspace(min(u0, u.min()), 1950, 200)
        fig, ax = plt.subplots(figsize=(7, 4.5))
        ax.plot(u, t_g / a.motores, "o", label="medido (por motor)")
        ax.plot(uu, k * np.clip(uu - u0, 0, None) ** 2 / a.motores, "-", label="ajuste k(u-u0)^2")
        ax.axhline(peso_g / a.motores, ls="--", color="gray", label=f"hover {peso_g/a.motores:.0f} gf")
        ax.set_xlabel("PWM [us]")
        ax.set_ylabel("Empuje por motor [gf]")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(a.png, dpi=150)
        print(f"Grafica: {a.png}")


if __name__ == "__main__":
    main()
