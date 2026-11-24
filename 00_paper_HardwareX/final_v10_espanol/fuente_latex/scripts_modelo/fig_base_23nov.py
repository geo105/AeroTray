"""Figura de la base con los parametros medidos el 23/11: (a) velocidad estacionaria frente a la pendiente
(modelo del TorqueNADO, 10,4 kg) para tres ciclos de trabajo y la velocidad medida al 15 % en llano;
(b) margen de aceleracion antes de volcar hacia atras con el modulo libre en la rotula (modelo calibrado con
las pendientes de vuelco medidas). Salida: ../figures/fig_base_limits.pdf"""
import os, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from params import P, g
Mb, xb, hb = P['M_base'], P['xG_base'], P['hG_base']; Mm = P['m'] + P['mc']; lJ, hJ = P['lJ'], P['hJ0']
r, w0, ts, Crr = P['r'], P['w0'], P['tau_s'], P['Crr']; M = Mb + Mm
x = (Mb * xb + Mm * lJ) / M; h = (Mb * hb + Mm * hJ) / M
be = np.linspace(0, 25, 101); b = np.radians(be)
def v(u): return 100 * r * w0 * (u - r * M * g * (np.sin(b) + Crr * np.cos(b)) / (2 * ts))
amax = g * (M * x * np.cos(b) - M * h * np.sin(b)) / (Mb * hb + Mm * hJ)
plt.rcParams.update({"font.family": "serif", "font.size": 8, "axes.grid": True, "grid.alpha": .3, "axes.spines.top": False,
                     "axes.spines.right": False, "legend.fontsize": 6.5, "savefig.bbox": "tight", "mathtext.fontset": "cm",
                     "axes.unicode_minus": False})
coma = FuncFormatter(lambda val, p: f"{val:g}".replace(".", ","))
fig, ax = plt.subplots(1, 2, figsize=(13.7 / 2.54, 5.0 / 2.54))
for u, c, ls in ((0.15, "#0072B2", "-"), (0.30, "#009E73", "--"), (0.50, "#E69F00", "-.")):
    ax[0].plot(be, np.clip(v(u), 0, None), c=c, ls=ls, lw=1.3, label=f"{int(u*100)} %")
ax[0].errorbar([0], [7.0], yerr=[[1.0], [1.0]], fmt="o", c="k", ms=3, capsize=2, label="medido al 15 %")
ax[0].set_xlabel(r"Pendiente $\beta$ (°)"); ax[0].set_ylabel("Velocidad estacionaria (cm/s)"); ax[0].legend(frameon=False)
for bb in (10, 20): ax[1].axvline(bb, c="0.6", ls=":", lw=.8)
ax[1].plot(be, amax, c="k", lw=1.3, label="módulo en la rótula ($f=0$)")
ax[1].axhline(0, c="#D55E00", lw=.8)
ax[1].plot([P['tip_climb_meas']], [0], "o", c="#0072B2", ms=4, label="vuelco medido (21,5°)")
ax[1].set_xlabel(r"Pendiente $\beta$ (°)"); ax[1].set_ylabel("Acel. sin vuelco (m/s²)"); ax[1].legend(frameon=False, loc="upper right")
for a, t in zip(ax, "ab"):
    a.text(0.02, 1.02, f"({t})", transform=a.transAxes, fontsize=8); a.yaxis.set_major_formatter(coma)
fig.tight_layout(w_pad=1.0)
fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures", "fig_base_limits.pdf"))
i10, i20 = np.argmin(abs(be - 10)), np.argmin(abs(be - 20))
print(f"amax 0 deg {amax[0]:.2f}, 10 deg {amax[i10]:.2f}, 20 deg {amax[i20]:.2f}; v15 llano {v(0.15)[0]:.1f} cm/s, 10 deg {v(0.15)[i10]:.1f}")
