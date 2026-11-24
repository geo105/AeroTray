"""Generates the data figures of the HardwareX manuscript from the raw logs.

Inputs (relative to the repository root ~/DSM):
  data/drone/vuelo_*.csv        drone telemetry recorded by the Raspberry Pi (10 Hz)
  data/drone/dron_log_*.csv     drone on-board buffer exported after a run (200 Hz nominal)
  data/ball_plate/registro_*.csv camera / ball-and-plate log (~187 Hz)
Outputs: analysis/figures/*.pdf and data/*.json
Run: cd 00_paper_HardwareX/scripts_repositorio && python3 bp_metrics.py && python3 make_figures.py
"""
import csv, glob, json, math, os, time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from bp_metrics import load, stats

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FIG = os.path.join(ROOT, "00_paper_HardwareX", "figuras_regeneradas")
DAT = os.path.join(ROOT, "00_paper_HardwareX", "datos_regenerados")
plt.rcParams.update({"font.family": "serif", "font.size": 9, "axes.grid": True, "grid.alpha": 0.3,
                     "axes.spines.top": False, "axes.spines.right": False, "legend.fontsize": 8,
                     "savefig.bbox": "tight", "savefig.dpi": 300})
C = {"roll": "#1f77b4", "pitch": "#d62728", "col": "#444444", "ok": "#2ca02c", "cut": "#d62728", "x": "#1f77b4", "y": "#ff7f0e"}
W = 6.3  # text width of elsarticle preprint (in)

# ---------------------------------------------------------------- drone thrust model
def thrust(pwm):
    return 5.3955 * np.clip((pwm - 1140) / 860, 0, None) ** 1.6

def fig_thrust():
    p = np.linspace(1140, 2000, 300)
    m, g = 3.6, 9.81
    fig, ax = plt.subplots(figsize=(W * 0.62, 2.5))
    ax.plot(p, 8 * thrust(p), color=C["col"], label="Firmware thrust model, 8 motors")
    ax.axhline(m * g, color="k", ls="--", lw=0.9, label=f"Weight of the moving assembly ({m} kg)")
    ax.axvline(1898, color=C["pitch"], ls=":", lw=1.2)
    ax.text(1902, 3, "Isaac Lab\nhover 1898 µs", color=C["pitch"], fontsize=7)
    ax.axvspan(1725, 1775, color=C["ok"], alpha=0.25, label="Measured lift-off band (1725–1775 µs)")
    ax.set_xlabel("Collective PWM command (µs)"); ax.set_ylabel("Total thrust (N)")
    ax.set_xlim(1300, 2000); ax.set_ylim(0, 46); ax.legend(loc="upper left")
    fig.savefig(os.path.join(FIG, "fig_thrust_model.pdf")); plt.close(fig)

# ---------------------------------------------------------------- flight campaign
FLIGHTS = [  # file, firmware, note (outcome checked in salida.log)
    ("vuelo_20261120_091438.csv", "v2", "tilt cut-off (accelerometer vibration)"),
    ("vuelo_20261120_092412.csv", "v3", "yaw cut-off (-20°)"),
    ("vuelo_20261120_093101.csv", "v4", "operator stop, pitch drift"),
    ("vuelo_20261120_093636.csv", "v5", "operator stop, slow oscillation"),
    ("vuelo_20261120_094224.csv", "v6", "tilt cut-off at lift-off"),
    ("vuelo_20261120_150927.csv", "v6", "roll cut-off at 1725 µs"),
    ("vuelo_20261120_152249.csv", "v8", "hover 20 s, cut-off during landing"),
    ("vuelo_20261120_152743.csv", "v9", "hover and landing without cut-off"),
    ("vuelo_20261120_154605.csv", "v9", "hover 48 s, pitch cut-off at 1816 µs"),
]

def read_flight(f):
    r = list(csv.DictReader(open(os.path.join(ROOT, "08_datos_y_registros", "registros_vuelo_dron", f))))
    t = np.array([float(x["t_pc"]) for x in r]); t -= t[0]
    M = np.array([[float(x[f"M{i}_us"]) for i in range(1, 9)] for x in r])
    g = lambda k: np.array([float(x[k]) for x in r])
    return t, M.mean(1), g("roll_deg"), g("pitch_deg"), g("yaw_rel_deg"), M

def flight_table():
    rows = []
    for k, (f, v, note) in enumerate(FLIGHTS, 1):
        t, col, ro, pi, ya, M = read_flight(f)
        m = col > 1650
        tilt = np.degrees(np.arccos(np.cos(np.radians(ro)) * np.cos(np.radians(pi))))
        rows.append(dict(run=k, file=f, firmware=v, outcome=note, max_collective_us=round(float(col.max())),
                         time_above_1650_s=round(m.sum() / 10, 1),
                         roll_rms_deg=round(float(ro[m].std()), 2) if m.sum() > 5 else None,
                         pitch_rms_deg=round(float(pi[m].std()), 2) if m.sum() > 5 else None,
                         tilt_max_deg=round(float(tilt[m].max()), 1) if m.sum() > 5 else None,
                         yaw_max_deg=round(float(np.abs(ya[m]).max()), 1) if m.sum() > 5 else None,
                         motor_spread_us=round(float(np.median(M[m].max(1) - M[m].min(1)))) if m.sum() > 5 else None))
    json.dump(rows, open(os.path.join(DAT, "flight_campaign.json"), "w"), indent=1)
    return rows

def fig_campaign(rows):
    fig, ax = plt.subplots(1, 2, figsize=(W, 2.4), gridspec_kw={"width_ratios": [1, 1.25]})
    lab = [f"{r['run']}\n{r['firmware']}" for r in rows]
    x = np.arange(len(rows))
    rr = [r["roll_rms_deg"] or 0 for r in rows]; pp = [r["pitch_rms_deg"] or 0 for r in rows]
    ax[0].bar(x - 0.2, rr, 0.4, color=C["roll"], label="roll RMS")
    ax[0].bar(x + 0.2, pp, 0.4, color=C["pitch"], label="pitch RMS")
    ax[0].set_xticks(x, lab); ax[0].set_ylabel("RMS above 1650 µs (°)"); ax[0].set_xlabel("Run / firmware")
    for i, r in enumerate(rows):
        if r["roll_rms_deg"] is None: ax[0].text(i, 0.15, "no\nlift", ha="center", fontsize=6.5)
    ax[0].legend(loc="upper right")
    for f, v, c in (("vuelo_20261120_152249.csv", "v8", C["roll"]), ("vuelo_20261120_152743.csv", "v9", C["ok"])):
        t, col, ro, pi, ya, M = read_flight(f)
        tilt = np.degrees(np.arccos(np.cos(np.radians(ro)) * np.cos(np.radians(pi))))
        ax[1].plot(t, tilt, color=c, lw=0.9, label=f"tilt, {v}")
        a2 = ax[1].twinx() if v == "v8" else a2
        a2.plot(t, col, color=c, lw=0.8, ls="--")
    a2.set_ylabel("Mean motor cmd. (µs)"); a2.set_ylim(1000, 2000); a2.grid(False)
    ax[1].axhline(12, color="k", lw=0.8, ls=":"); ax[1].text(78, 12.4, "tilt cut-off 12°", fontsize=7)
    ax[1].set_xlabel("Time (s)"); ax[1].set_ylabel("Tilt (°)"); ax[1].set_ylim(0, 14); ax[1].legend(loc="center right")
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig_flight_campaign.pdf")); plt.close(fig)

# ---------------------------------------------------------------- bench (200 Hz buffer)
def fig_bench():
    out = []
    fig, ax = plt.subplots(1, 2, figsize=(W, 2.1), sharey=True)
    for a, f in zip(ax, sorted(glob.glob(os.path.join(ROOT, "08_datos_y_registros", "registros_vuelo_dron", "dron_log_20261118_*.csv")))):
        r = list(csv.DictReader(open(f)))
        t = np.array([int(x["t_us"]) for x in r]) * 1e-6; t -= t[0]
        ro = np.array([float(x["roll_deg"]) for x in r]); pi = np.array([float(x["pitch_deg"]) for x in r])
        M = np.array([[int(x[f"M{i}_us"]) for i in range(1, 9)] for x in r])
        a.plot(t, ro, color=C["roll"], lw=0.8, label="roll"); a.plot(t, pi, color=C["pitch"], lw=0.8, label="pitch")
        a.set_xlabel("Time (s)"); a.set_title(os.path.basename(f)[9:24].replace("_", " "), fontsize=8)
        out.append(dict(file=os.path.basename(f), fs_hz=round(len(t) / t[-1]), roll_rms=round(float(np.sqrt(np.mean(ro ** 2))), 2),
                        pitch_rms=round(float(np.sqrt(np.mean(pi ** 2))), 2), tilt_max=round(float(np.max(np.hypot(ro, pi))), 2),
                        motor_min=int(M.min()), motor_max=int(M.max())))
    ax[0].set_ylabel("Angle (°)"); ax[0].legend()
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig_drone_bench.pdf")); plt.close(fig)
    json.dump(out, open(os.path.join(DAT, "drone_bench.json"), "w"), indent=1)
    return out

# ---------------------------------------------------------------- ball and plate
VIDEOS = {  # recording (phone clock) -> scenario, classified by inspecting the videos
    "VID_20261118_154536.mp4": "flat", "VID_20261118_155304.mp4": "flat", "VID_20261118_170033.mp4": "flat",
    "VID_20261118_170530.mp4": "flat", "VID_20261118_161620.mp4": "ramp", "VID_20261118_161741.mp4": "ramp",
    "VID_20261118_162219.mp4": "ramp", "VID_20261118_165403.mp4": "ramp", "VID_20261118_165508.mp4": "ramp",
    "VID_20261118_165747.mp4": "ramp", "VID_20261118_171412.mp4": "ramp"}

def series(day, t0, t1):
    D = os.path.join(ROOT, "08_datos_y_registros", "copia_raspberry_pi_20261120", "pi", "registros")
    for f in sorted(glob.glob(os.path.join(D, f"registro_{day}_*.csv"))):
        t, x, y, e, px, py = load(f)
        if len(t) and t[0] <= t1 and t[-1] >= t0:
            m = (t >= t0) & (t <= t1) & (e == 3) & ~np.isnan(x)
            if m.sum() > 50:
                return t[m] - t0, x[m], y[m]
    return None

def fig_bp():
    met = json.load(open(os.path.join(DAT, "bp_metrics.json")))
    vids = met["videos_20261118"]
    D = os.path.join(ROOT, "08_datos_y_registros", "copia_raspberry_pi_20261120", "pi", "registros")
    # stationary references (17/11): drone motors off and on
    ref = {}
    for name, (f, a) in {"base still, drone off": ("registro_20261117_162914.csv", "19:21:15"),
                         "base still, drone on": ("registro_20261117_162914.csv", "19:29:18")}.items():
        t, x, y, e, px, py = load(os.path.join(D, f))
        t0 = time.mktime(time.strptime("2026-11-17 " + a, "%Y-%m-%d %H:%M:%S"))
        act = (t >= t0) & ((e == 1) | (e == 3)); ta = t[act]; br = np.where(np.diff(ta) > 1)[0]
        t1 = ta[br[0]] if len(br) else ta[-1]
        ref[name] = stats(t, x, y, e, t0, t1)
        s = series("20261117", t0, t1); ref[name]["d"] = np.hypot(s[1], s[2])
    groups = {"base still,\ndrone off": ref["base still, drone off"]["d"], "base still,\ndrone on": ref["base still, drone on"]["d"]}
    for sc in ("flat", "ramp"):
        ds = []
        for v in vids:
            if VIDEOS.get(v["video"]) == sc and v["dur_s"] > 20:
                tv = time.mktime(time.strptime(v["video"][4:19], "%Y%m%d_%H%M%S"))
                s = series("20261118", tv, tv + v["dur_s"])
                if s: ds.append(np.hypot(s[1], s[2]))
        groups[{"flat": "moving,\nflat floor", "ramp": "moving,\n10° ramp"}[sc]] = np.concatenate(ds)
    fig, ax = plt.subplots(1, 2, figsize=(W, 2.5), gridspec_kw={"width_ratios": [1, 1.4]})
    ax[0].boxplot(list(groups.values()), labels=list(groups.keys()), showfliers=False, whis=(5, 95),
                  medianprops={"color": "k"})
    ax[0].set_ylabel("Ball error |e| (mm)"); ax[0].tick_params(axis="x", labelsize=7)
    ax[0].axhline(110, color="k", ls=":", lw=0.8); ax[0].text(0.6, 112, "plate half-width 110 mm", fontsize=7)
    tv = time.mktime(time.strptime("20261118_165747", "%Y%m%d_%H%M%S"))
    t, x, y = series("20261118", tv, tv + 69.0)
    ax[1].plot(t, x, color=C["x"], lw=0.6, label="x"); ax[1].plot(t, y, color=C["y"], lw=0.6, label="y")
    ax[1].set_xlabel("Time (s)"); ax[1].set_ylabel("Ball position (mm)"); ax[1].legend(loc="lower right", ncol=2)
    ax[1].set_title("Ramp run (video VID_20261118_165747)", fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, "fig_bp_results.pdf")); plt.close(fig)
    summ = {k.replace("\n", " "): dict(n=int(len(v)), median=round(float(np.median(v)), 1), mean=round(float(np.mean(v)), 1),
                                       p90=round(float(np.percentile(v, 90)), 1), p95=round(float(np.percentile(v, 95)), 1),
                                       max=round(float(np.max(v)), 1)) for k, v in groups.items()}
    json.dump({"groups": summ, "references": {k: {kk: vv for kk, vv in v.items() if kk != "d"} for k, v in ref.items()}},
              open(os.path.join(DAT, "bp_summary.json"), "w"), indent=1)
    return summ

if __name__ == "__main__":
    os.makedirs(FIG, exist_ok=True); os.makedirs(DAT, exist_ok=True)
    fig_thrust()
    rows = flight_table(); fig_campaign(rows)
    for r in rows: print(r)
    print(fig_bench())
    print(json.dumps(fig_bp(), indent=1))
