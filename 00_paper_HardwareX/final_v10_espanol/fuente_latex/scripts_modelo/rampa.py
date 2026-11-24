"""Efecto del empuje del modulo aereo sobre la subida de la rampa (descarga de peso f = T/W_mod).
El empuje es vertical en {W} porque el octocoptero se nivela; en el balance de momentos equivale a reducir
el peso del modulo concentrado en la junta (Sec. tip-over). f=0: rotores en reposo; f=1: modulo flotando."""
import os as _os
ROOT=_os.environ.get("AEROTRAY_ROOT",_os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"..","..")))
FIGDIR=_os.environ.get("FIG",_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"figuras")); _os.makedirs(FIGDIR,exist_ok=True)

import numpy as np, json, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from params import P, g
plt.rcParams.update({"font.family":"serif","font.size":8.5,"axes.grid":True,"grid.alpha":0.3,"axes.spines.top":False,
  "axes.spines.right":False,"legend.fontsize":7,"savefig.bbox":"tight","savefig.dpi":300,"mathtext.fontset":"cm"})
FIG=FIGDIR
Mb,xb,hb=P['M_base'],P['xG_base'],P['hG_base']; Mm=P['m']+P['mc']; lJ,hJ=P['lJ'],P['hJ0']; L=P['Lwb']
r,w0,ts,Crr=P['r'],P['w0'],P['tau_s'],P['Crr']
def cg(f):
    me=Mm*(1-f); M=Mb+me
    return M,(Mb*xb+me*lJ)/M,(Mb*hb+me*hJ)/M
def tip(f): M,x,h=cg(f); return np.degrees(np.arctan(x/h))
def amax(f,b):
    # el empuje quita peso, no masa: la inercia del modulo completo sigue actuando en la rotula
    M,x,h=cg(f); b=np.radians(b); I=Mb*hb+Mm*hJ
    return g*(M*x*np.cos(b)-M*h*np.sin(b))/I
def vss(f,b,u=0.15): M,x,h=cg(f); b=np.radians(b); return r*w0*(u-r*M*g*(np.sin(b)+Crr*np.cos(b))/(2*ts))
def mu(f,b):
    M,x,h=cg(f); b=np.radians(b); Nr=M*g*((L-x)*np.cos(b)+h*np.sin(b))/L
    return M*g*(np.sin(b)+Crr*np.cos(b))/Nr
def taul(f,b): M,x,h=cg(f); b=np.radians(b); return r*M*g*(np.sin(b)+Crr*np.cos(b))/(2*ts)
kF=7.6
def ucol(f): xx=(f*Mm*g/(8*kF))**(1/P['n_th']); return P['u0']+P['uspan']*xx
Pm_h=62.5   # W por motor en estacionario (bateria.py)
def Irama(f,V=11.2): return 4*Pm_h*np.clip(f,0,None)**1.5/V
res={}
for f in (0.0,0.25,0.5,0.75,1.0):
    res[f"f={f}"]=dict(tip_deg=float(tip(f)),amax10=float(amax(f,10)),amax20=float(amax(f,20)),v20=float(vss(f,20)),v10=float(vss(f,10)),
      mu10=float(mu(f,10)),mu20=float(mu(f,20)),tau20_pct=float(100*taul(f,20)),u_col=float(ucol(f)),I_rama=float(Irama(f)),
      Q60s_Ah=float(Irama(f)*60/3600))
json.dump(res,open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"resultados","rampa.json"),"w"),indent=1)
for k,v in res.items(): print(k,{a:round(b,3) for a,b in v.items()})
f=np.linspace(0,1,101)
fig,ax=plt.subplots(1,2,figsize=(6.6,2.5))
ax[0].plot(100*f,tip(f),color="#1f77b4",lw=1.3,label="ángulo de vuelco estático")
ax[0].axhline(20,color="k",ls="--",lw=0.8); ax[0].text(2,20.8,"rampa de 20° (guía del curso)",fontsize=6.5)
ax[0].axhline(10,color="k",ls=":",lw=0.8); ax[0].text(2,10.8,"rampa de 10° (ensayada)",fontsize=6.5)
ax[0].set_xlabel("Peso del módulo sostenido por los rotores, $f$ (%)"); ax[0].set_ylabel("Pendiente de vuelco (°)")
ax[0].set_ylim(0,50); ax[0].set_title("(a)",loc="left",fontsize=9)
a2=ax[0].twiny(); a2.set_xlim(ax[0].get_xlim()); tk=[0,25,50,75,100]; a2.set_xticks(tk,[f"{ucol(t/100):.0f}" if t>0 else "1300" for t in tk])
a2.set_xlabel("PWM colectivo equivalente (µs)",fontsize=7.5); a2.grid(False)
for b,c in ((10,"#2ca02c"),(20,"#d62728")):
    ax[1].plot(100*f,amax(f,b),color=c,lw=1.3,label=f"margen de aceleración en {b}°")
ax[1].set_xlabel("Peso del módulo sostenido por los rotores, $f$ (%)"); ax[1].set_ylabel("Aceleración sin vuelco (m/s²)")
ax[1].set_ylim(0,9); ax[1].set_title("(b)",loc="left",fontsize=9)
b3=ax[1].twinx(); b3.spines["right"].set_visible(True); b3.grid(False)
b3.axhspan(0.5,0.8,color="0.85",zorder=0); b3.text(3,0.52,"μ disponible (caucho–madera)",fontsize=6.5,color="0.35")
for b,c in ((10,"#2ca02c"),(20,"#d62728")): b3.plot(100*f,mu(f,b),color=c,lw=1.0,ls="--")
b3.set_ylabel("μ requerido en ruedas motrices (– –)"); b3.set_ylim(0,1.0)
ax[1].axvspan(35,55,color="#ff7f0e",alpha=0.15,lw=0); ax[1].text(45,0.25,"modo\nrampa",fontsize=6.5,color="#d2691e",ha="center")
ax[1].set_zorder(b3.get_zorder()+1); ax[1].patch.set_visible(False)
ax[1].legend(loc="upper left",fontsize=6.5,framealpha=0.9)
for a in (ax[1],b3):
    a.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v,p: f"{v:g}".replace(".",",")))
fig.tight_layout(); fig.savefig(f"{FIG}/fig_rampa.pdf"); plt.close(fig)
