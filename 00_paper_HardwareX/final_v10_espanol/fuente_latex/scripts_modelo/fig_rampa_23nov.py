"""Figura de la rampa con los parametros medidos el 23/11 (masa 10,4 kg, L = 0,22 m, CM por vuelco).
(a) pendiente de vuelco subiendo y bajando de frente frente a f; (b) margen de aceleracion en 10 y 20 grados;
(c) friccion requerida en las ruedas motrices al subir. Salida: ../figures/fig_rampa.pdf (13,7 cm de ancho)."""
import os, json, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from params import P, g
Mb, xb, hb = P['M_base'], P['xG_base'], P['hG_base']; Mm = P['m'] + P['mc']; lJ, hJ, L = P['lJ'], P['hJ0'], P['Lwb']; Crr = P['Crr']
def cg(f): me = Mm * (1 - f); M = Mb + me; return M, (Mb * xb + me * lJ) / M, (Mb * hb + me * hJ) / M
def sub(f): M, x, h = cg(f); return np.degrees(np.arctan2(x, h))
def baj(f): M, x, h = cg(f); return np.degrees(np.arctan2(L - x, h))
def amax(f, b):
    M, x, h = cg(f); b = np.radians(b); I = Mb * hb + Mm * hJ
    return g * (M * x * np.cos(b) - M * h * np.sin(b)) / I
def mu(f, b):
    M, x, h = cg(f); b = np.radians(b); Nr = M * g * ((L - x) * np.cos(b) + h * np.sin(b)) / L
    return M * g * (np.sin(b) + Crr * np.cos(b)) / Nr
plt.rcParams.update({"font.family": "serif", "font.size": 8, "axes.grid": True, "grid.alpha": .3, "axes.spines.top": False,
                     "axes.spines.right": False, "legend.fontsize": 6.5, "savefig.bbox": "tight", "mathtext.fontset": "cm",
                     "axes.unicode_minus": False})
coma = FuncFormatter(lambda v, p: f"{v:g}".replace(".", ","))
f = np.linspace(0, 1, 101); F = 100 * f
fig, ax = plt.subplots(1, 3, figsize=(13.7 / 2.54, 4.6 / 2.54))
ax[0].plot(F, sub(f), c="#0072B2", lw=1.3, label="subiendo de frente")
ax[0].plot(F, baj(f), c="#D55E00", lw=1.3, ls="--", label="bajando de frente")
ax[0].plot([0], [P['tip_climb_meas']], "o", c="#0072B2", ms=3.5); ax[0].plot([0], [P['tip_desc_meas']], "s", c="#D55E00", ms=3.5)
ax[0].axhline(10, c="k", lw=.7, ls=":"); ax[0].text(55, 11, "rampa de 10°", fontsize=6)
ax[0].set_ylabel("Pendiente de vuelco (°)"); ax[0].set_ylim(0, 65); ax[0].legend(loc="upper left", frameon=False)
for b, c in ((10, "#009E73"), (20, "#CC79A7")):
    ax[1].plot(F, amax(f, b), c=c, lw=1.3, label=f"{b}°"); ax[2].plot(F, mu(f, b), c=c, lw=1.3, label=f"{b}°")
ax[1].set_ylabel("Acel. sin vuelco (m/s²)"); ax[1].set_ylim(0, 3); ax[1].legend(frameon=False)
ax[2].axhspan(0.5, 0.8, color="0.88", zorder=0); ax[2].text(3, 0.52, r"$\mu$ disponible", fontsize=6, color="0.35")
ax[2].set_ylabel(r"$\mu$ requerido al subir"); ax[2].set_ylim(0, 0.9); ax[2].legend(loc="upper left", frameon=False)
for a, t in zip(ax, "abc"):
    a.axvspan(35, 55, color="#E69F00", alpha=.15, lw=0); a.set_xlabel("Peso sostenido, $f$ (%)")
    a.text(0.02, 1.02, f"({t})", transform=a.transAxes, fontsize=8); a.yaxis.set_major_formatter(coma)
fig.tight_layout(w_pad=0.6)
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures", "fig_rampa.pdf"))
tab = {}
for ff in (0, 0.12, 0.25, 0.5, 0.75, 1.0):
    M, x, h = cg(ff)
    tab[str(ff)] = dict(sub=round(float(sub(ff)), 1), baj=round(float(baj(ff)), 1), amax10=round(float(amax(ff, 10)), 2),
                        amax20=round(float(amax(ff, 20)), 2), mu10=round(float(mu(ff, 10)), 2), mu20=round(float(mu(ff, 20)), 2))
json.dump(tab, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultados", "rampa_23nov.json"), "w"), indent=1)
for k, v in tab.items(): print(k, v)
