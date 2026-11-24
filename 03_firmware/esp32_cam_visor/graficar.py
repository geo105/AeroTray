#!/usr/bin/env python3
"""Graficas de un registro del visor: posicion de la pelota, pulsos de los servos y trayectoria.

Uso:
  python3 graficar.py registros/registro_20261102_210413.csv
  python3 graficar.py registro.csv --desde 60 --hasta 120     # solo ese tramo (segundos)
  python3 graficar.py registro.csv --mm-por-px 3               # ejes en milimetros
  python3 graficar.py registro.csv --mostrar                   # abrir la ventana ademas de guardar

Guarda <registro>.png al lado del CSV e imprime un resumen del tiempo que la pelota pasa
cerca del centro mientras el PID esta encendido.
"""
import argparse
import os

import matplotlib
import numpy as np

# centro de la placa en la imagen (igual que en el firmware y en visor.py)
CENTRO_X, CENTRO_Y = 89, 57
ALCANCE_X, ALCANCE_Y = 24, 39
# tamano aceptado de la pelota en pixeles (igual que en el firmware)
BOLA_PX_MIN, BOLA_PX_MAX = 15, 200

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("registro", help="CSV guardado por visor.py")
ap.add_argument("--desde", type=float, default=0, help="segundo inicial del tramo")
ap.add_argument("--hasta", type=float, default=None, help="segundo final del tramo")
ap.add_argument("--mm-por-px", type=float, default=None, help="convertir posiciones a milimetros")
ap.add_argument("--mostrar", action="store_true", help="abrir la ventana con las graficas")
args = ap.parse_args()
if not args.mostrar:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

d = np.genfromtxt(args.registro, delimiter=",", names=True, filling_values=np.nan)
t = d["tiempo_unix"] - d["tiempo_unix"][0]
tramo = (t >= args.desde) & ((t <= args.hasta) if args.hasta is not None else True)
d, t = d[tramo], t[tramo]
if len(d) == 0:
    raise SystemExit("El tramo elegido no tiene datos")

estado = d["estado"].astype(int)
pid = (estado & 1) == 1
neutro = (estado & 4) == 4
valida = (d["cx_px"] >= 0) & (d["cx_px"] < 1000) & (d["px_bola"] >= BOLA_PX_MIN) & (d["px_bola"] <= BOLA_PX_MAX)
escala = args.mm_por_px or 1.0
unidad = "mm" if args.mm_por_px else "px"
X = np.where(valida, (d["cx_px"] - CENTRO_X) * escala, np.nan)
Y = np.where(valida, (CENTRO_Y - d["cy_px"]) * escala, np.nan)

fig = plt.figure(figsize=(13, 9))
fig.suptitle(os.path.basename(args.registro), fontsize=11)
rejilla = fig.add_gridspec(3, 2, width_ratios=[2.2, 1])


def sombrear(ax):
    """Fondo verde claro donde el PID estaba encendido, naranja donde estaba en neutro fijo."""
    for mascara, color in ((pid, "#d8f0d8"), (neutro, "#fbe6cc")):
        inicio = None
        for i, m in enumerate(np.append(mascara, False)):
            if m and inicio is None:
                inicio = t[i]
            elif not m and inicio is not None:
                ax.axvspan(inicio, t[i - 1], color=color, zorder=0)
                inicio = None


ax1 = fig.add_subplot(rejilla[0, 0])
sombrear(ax1)
ax1.plot(t, X, lw=0.9, color="#c0392b", label="X")
ax1.plot(t, Y, lw=0.9, color="#27ae60", label="Y")
ax1.axhspan(-4 * escala, 4 * escala, color="#999", alpha=0.15, label=f"±{4 * escala:g} {unidad}")
ax1.axhline(0, color="k", lw=0.6)
ax1.set_ylabel(f"posicion ({unidad})")
ax1.set_title("Posicion de la pelota respecto al centro (verde: PID encendido, naranja: neutro fijo)", fontsize=9)
ax1.legend(loc="upper right", fontsize=8, ncol=3)

ax2 = fig.add_subplot(rejilla[1, 0], sharex=ax1)
sombrear(ax2)
ax2.plot(t, d["pulso_x_us"], lw=0.9, color="#c0392b", label="servo X")
ax2.plot(t, d["pulso_y_us"], lw=0.9, color="#27ae60", label="servo Y")
ax2.set_ylabel("pulso (us)")
ax2.set_title("Pulsos enviados a los servos", fontsize=9)
ax2.legend(loc="upper right", fontsize=8)

ax3 = fig.add_subplot(rejilla[2, 0], sharex=ax1)
ax3.plot(t, d["px_bola"], lw=0.8, color="#555", label="pixeles de la pelota")
ax3.axhspan(BOLA_PX_MIN, BOLA_PX_MAX, color="#3498db", alpha=0.1, label="tamano aceptado")
if "corruptas_s" in d.dtype.names and np.isfinite(d["corruptas_s"]).any():
    ax3b = ax3.twinx()
    ax3b.plot(t, d["corruptas_s"], lw=0.8, color="#8e44ad", label="imagenes corruptas/s")
    ax3b.set_ylabel("corruptas/s", color="#8e44ad")
ax3.set_ylim(0, max(250, np.nanpercentile(d["px_bola"], 99)))
ax3.set_xlabel("tiempo (s)")
ax3.set_ylabel("pixeles")
ax3.set_title("Deteccion (tamano de la mancha) y calidad de imagen", fontsize=9)
ax3.legend(loc="upper left", fontsize=8)

ax4 = fig.add_subplot(rejilla[:, 1])
sel = valida & pid
ax4.add_patch(plt.Rectangle((-ALCANCE_X * escala, -ALCANCE_Y * escala), 2 * ALCANCE_X * escala,
                            2 * ALCANCE_Y * escala, fill=False, color="#e6b800", lw=1, label="zona de control"))
puntos = ax4.scatter(X[sel], Y[sel], c=t[sel], s=4, cmap="viridis")
ax4.plot(0, 0, "k+", ms=12)
ax4.set_aspect("equal")
ax4.set_xlabel(f"X ({unidad})")
ax4.set_ylabel(f"Y ({unidad})")
ax4.set_title("Trayectoria con el PID encendido", fontsize=9)
fig.colorbar(puntos, ax=ax4, label="tiempo (s)", shrink=0.6)
ax4.legend(loc="upper right", fontsize=8)

fig.tight_layout()
salida = os.path.splitext(args.registro)[0] + (f"_{args.desde:g}-{args.hasta:g}s" if args.hasta else "") + ".png"
fig.savefig(salida, dpi=110)
print(f"Grafica guardada en {salida}")

# resumen numerico
duracion = t[-1] - t[0]
print(f"Duracion: {duracion:.1f} s, {len(d)} muestras ({len(d) / max(duracion, 1e-9):.1f} por segundo)")
print(f"PID encendido: {pid.mean() * 100:.0f} % del tiempo; pelota detectada: {valida.mean() * 100:.0f} %")
if sel.any():
    Xs, Ys = X[sel] / escala, Y[sel] / escala
    for lim in (2, 4, 8):
        dentro = ((np.abs(Xs) <= lim) & (np.abs(Ys) <= lim)).mean() * 100
        print(f"  con PID, a <= {lim} px del centro: {dentro:.0f} % del tiempo")
    print(f"  error medio |X| {np.abs(Xs).mean():.1f} px, |Y| {np.abs(Ys).mean():.1f} px; "
          f"rango X {Xs.min():+.0f}..{Xs.max():+.0f}, Y {Ys.min():+.0f}..{Ys.max():+.0f}")
if args.mostrar:
    plt.show()
