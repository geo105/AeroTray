"""Figuras del paper para los ensayos del 23/11 (robot completo)."""
import json, os, csv, time, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
AQUI = os.path.dirname(os.path.abspath(__file__)); FIG = os.path.join(AQUI, "..", "..", "laptop2", "AeroTray_Paper_Final_ES_Overleaf", "AeroTray_Paper_ES", "figures")
import analizar_23nov as A
res = json.load(open(os.path.join(AQUI, "resultados", "ensayos_23nov.json")))
pi = A.leer_pi(); V = {"1729": A.leer_video("telemetria_video_1729.csv"), "1836": A.leer_video("telemetria_video_1836.csv")}
plt.rcParams.update({"font.family": "serif", "font.size": 8, "mathtext.fontset": "cm", "axes.unicode_minus": False})
hm = lambda h: time.mktime(time.strptime("2026-11-23 " + h, "%Y-%m-%d %H:%M:%S"))
T_V1 = (hm("18:26:55"), hm("18:28:30")); T_V2 = hm("18:36:52")
# --- Fig 1: distribucion del error por ensayo
fig, ax = plt.subplots(1, 2, figsize=(7.2, 2.8), gridspec_kw={"width_ratios": [3, 1.2]})
datos, cols, nombres = [], [], []
for e in res["ensayos"]:
    d = pi if e["fuente"].startswith("Pi") else V[e["fuente"].split()[1]]
    k = (d["t"] >= e["t0"] + 3) & (d["t"] <= e["t1"]) & ~np.isnan(d["x"]) & (np.hypot(d["x"], d["y"]) <= 130) & (d["pid"] == 1)
    datos.append(np.hypot(d["x"][k], d["y"][k])); nombres.append(e["ensayo"][-2:])
    on = e.get("dron_motores_on_pct")
    if on is None:   # ensayos del CSV de la Pi: motores segun los registros vuelo_20261123_182655 y _183652
        on = 100 if (e["t1"] > T_V1[0] and e["t0"] < T_V1[1]) or e["t1"] > T_V2 else 0
    cols.append("tab:orange" if on > 50 else "tab:blue")
b = ax[0].boxplot(datos, whis=(5, 95), showfliers=False, patch_artist=True, widths=.6)
for p, c in zip(b["boxes"], cols): p.set_facecolor(c); p.set_alpha(.55)
ax[0].set_xticklabels(nombres); ax[0].set_xlabel("ensayo BP23-"); ax[0].set_ylabel(r"$|e|$ (mm)"); ax[0].axhline(30, ls="--", c="k", lw=.6); ax[0].axhline(96.5, ls=":", c="r", lw=.7)
ax[0].text(13.4, 98, "$R_u$", color="r", fontsize=7, ha="right", va="bottom")
ax[0].set_title("(a) error por ensayo (caja: cuartiles; bigotes: P5-P95)"); ax[0].grid(alpha=.3, axis="y")
from matplotlib.patches import Patch
ax[0].legend(handles=[Patch(color="tab:blue", alpha=.55, label="rotores apagados"), Patch(color="tab:orange", alpha=.55, label="rotores encendidos")], loc="upper left", fontsize=7)
xs = np.concatenate([pi["x"][(pi["pid"] == 1)]]); 
sx = [e["x_std_mm"] for e in res["ensayos"]]; sy = [e["y_std_mm"] for e in res["ensayos"]]
ax[1].bar([0, 1], [np.median(sx), np.median(sy)], color=["tab:purple", "tab:green"], alpha=.7)
ax[1].errorbar([0, 1], [np.median(sx), np.median(sy)], yerr=[[np.median(sx) - min(sx), np.median(sy) - min(sy)], [max(sx) - np.median(sx), max(sy) - np.median(sy)]], fmt="none", c="k", capsize=3)
ax[1].set_xticks([0, 1]); ax[1].set_xticklabels(["eje $x$", "eje $y$"]); ax[1].set_ylabel("desv. estándar (mm)"); ax[1].set_title("(b) dispersión por eje"); ax[1].grid(alpha=.3, axis="y")
fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig_ensayos_23nov.pdf")); plt.close(fig)
# --- Fig 2: ensayo BP23-05 (Pi) con el despegue del dron (datos del vuelo impresos del registro vuelo_20261123_182655)
e = [x for x in res["ensayos"] if x["inicio"] == "18:26:29"][0]
k = (pi["t"] >= e["t0"] - 2) & (pi["t"] <= e["t0"] + 100)
t = pi["t"][k] - e["t0"]
fig, ax = plt.subplots(2, 1, figsize=(7.2, 3.6), sharex=True)
ax[0].plot(t, pi["x"][k], lw=.5, label="$x$"); ax[0].plot(t, pi["y"][k], lw=.5, label="$y$"); ax[0].set_ylim(-160, 160); ax[0].set_ylabel("bola (mm)"); ax[0].axhline(0, c="k", lw=.4); ax[0].legend(loc="lower left", ncol=2)
ax[1].plot(t, pi["px"][k], lw=.4, label="servo $x$"); ax[1].plot(t, pi["py"][k], lw=.4, label="servo $y$"); ax[1].axhline(1386, c="tab:blue", ls=":", lw=.7); ax[1].axhline(1612, c="k", ls="--", lw=.6)
ax[1].set_ylabel(r"pulso ($\mu$s)"); ax[1].set_xlabel("tiempo desde el inicio del ensayo (s)"); ax[1].legend(loc="lower left", ncol=2)
t_flot = time.mktime(time.strptime("2026-11-23 18:27:39", "%Y-%m-%d %H:%M:%S")) - e["t0"]; t_desp = time.mktime(time.strptime("2026-11-23 18:28:04", "%Y-%m-%d %H:%M:%S")) - e["t0"]
for a in ax:
    a.axvspan(t_flot, t_desp, color="tab:orange", alpha=.12); a.axvline(t_desp, c="tab:red", lw=.8); a.grid(alpha=.3)
ax[0].text(t_desp - 1, 125, r"orden Flotar: sube a 1750 $\mu$s", fontsize=6.5, ha="right"); ax[0].text(t_desp + 1, -150, "despegue", color="tab:red", fontsize=7)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig_bp23_despegue.pdf")); plt.close(fig)
print("ok")
