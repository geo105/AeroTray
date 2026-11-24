import os as _os
ROOT=_os.environ.get("AEROTRAY_ROOT",_os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"..","..")))
FIGDIR=_os.environ.get("FIG",_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"figuras")); _os.makedirs(FIGDIR,exist_ok=True)
import numpy as np, pandas as pd, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams.update({"font.family":"serif","font.size":9,"axes.grid":True,"grid.alpha":0.3,"axes.spines.top":False,"axes.spines.right":False,"legend.fontsize":8,"savefig.bbox":"tight","savefig.dpi":300})
V=ROOT+"/08_datos_y_registros/registros_vuelo_dron"
F=[("vuelo_20261120_091438.csv","v2"),("vuelo_20261120_092412.csv","v3"),("vuelo_20261120_093101.csv","v4"),("vuelo_20261120_093636.csv","v5"),("vuelo_20261120_094224.csv","v6"),("vuelo_20261120_150927.csv","v6"),("vuelo_20261120_152249.csv","v8"),("vuelo_20261120_152743.csv","v9"),("vuelo_20261120_154605.csv","v9")]
def rd(f):
    d=pd.read_csv(os.path.join(V,f)); t=d.t_pc.values-d.t_pc.values[0]
    M=d[[f"M{i}_us" for i in range(1,9)]].values.astype(float)
    return t,M.mean(1),d.roll_deg.values,d.pitch_deg.values
rr=[];pp=[]
for f,v in F:
    t,col,ro,pi=rd(f); m=col>1650
    rr.append(np.sqrt(np.mean(ro[m]**2)) if m.sum()>5 else 0); pp.append(np.sqrt(np.mean(pi[m]**2)) if m.sum()>5 else 0)
fig,ax=plt.subplots(1,2,figsize=(6.3,2.4),gridspec_kw={"width_ratios":[1,1.25]})
x=np.arange(len(F))
ax[0].bar(x-0.2,rr,0.4,color="#1f77b4",label="alabeo RMS"); ax[0].bar(x+0.2,pp,0.4,color="#d62728",label="cabeceo RMS")
ax[0].set_xticks(x,[f"{i+1}\n{v}" for i,(f,v) in enumerate(F)]); ax[0].set_ylabel("Error RMS sobre 1650 µs (°)"); ax[0].set_xlabel("Vuelo / firmware")
for i in range(len(F)):
    if rr[i]==0: ax[0].text(i,0.2,"sin\ndesp.",ha="center",fontsize=6.5)
ax[0].legend(loc="upper right")
for (f,v,c) in (("vuelo_20261120_152249.csv","v8","#1f77b4"),("vuelo_20261120_152743.csv","v9","#2ca02c")):
    t,col,ro,pi=rd(f); tilt=np.degrees(np.arccos(np.cos(np.radians(ro))*np.cos(np.radians(pi))))
    ax[1].plot(t,tilt,color=c,lw=0.9,label=f"inclinación, {v}")
    if v=="v8": a2=ax[1].twinx()
    a2.plot(t,col,color=c,lw=0.8,ls="--")
a2.set_ylabel("Orden media de motor (µs)"); a2.set_ylim(1000,2000); a2.grid(False)
ax[1].axhline(12,color="k",lw=0.8,ls=":"); ax[1].text(70,12.4,"corte por inclinación 12°",fontsize=7)
ax[1].set_xlabel("Tiempo (s)"); ax[1].set_ylabel("Inclinación (°)"); ax[1].set_ylim(0,14); ax[1].legend(loc="center right")
import figs_es_helper as H; H.es(fig); fig.tight_layout(); fig.savefig(FIGDIR+"/fig_flight_campaign.pdf")
print([round(a,2) for a in rr],[round(a,2) for a in pp])
