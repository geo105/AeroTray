"""Fig. de actitud en banco (18/11, colectivo 1300 us), etiquetas en espanol."""
import os as _os
ROOT=_os.environ.get("AEROTRAY_ROOT",_os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"..","..")))
FIGDIR=_os.environ.get("FIG",_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"figuras")); _os.makedirs(FIGDIR,exist_ok=True)

import numpy as np, pandas as pd, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import figs_es_helper as H
plt.rcParams.update({"font.family":"serif","font.size":9,"axes.grid":True,"grid.alpha":0.3,"axes.spines.top":False,
  "axes.spines.right":False,"legend.fontsize":8,"savefig.bbox":"tight","savefig.dpi":300})
V=ROOT+"/08_datos_y_registros/registros_vuelo_dron"
fig,ax=plt.subplots(1,2,figsize=(6.3,2.1),sharey=True)
for a,f in zip(ax,("dron_log_20261118_144610.csv","dron_log_20261118_144932.csv")):
    d=pd.read_csv(os.path.join(V,f)); t=(d.t_us-d.t_us.iloc[0])/1e6
    a.plot(t,d.roll_deg,color="#1f77b4",lw=0.8,label="alabeo"); a.plot(t,d.pitch_deg,color="#d62728",lw=0.8,label="cabeceo")
    a.set_title(f"registro {f[9:17]}_{f[18:24]}",fontsize=8); a.set_xlabel("Tiempo (s)")
    print(f, "RMS roll %.2f pitch %.2f max tilt %.2f fs %.0f Hz"%(np.sqrt(np.mean(d.roll_deg**2)),np.sqrt(np.mean(d.pitch_deg**2)),
          np.degrees(np.arccos(np.cos(np.radians(d.roll_deg))*np.cos(np.radians(d.pitch_deg)))).max(),1/np.median(np.diff(t))))
ax[0].set_ylabel("Ángulo (°)"); ax[0].legend(loc="upper left")
H.es(fig); fig.tight_layout(); fig.savefig(FIGDIR+"/fig_drone_bench.pdf")
