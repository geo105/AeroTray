"""Margen de empuje y autonomia del modulo aereo frente a la tension de la bateria.
Modelo: empuje total 8 k_F' (V_bus/V_bus,ref)^2 x^1.6, x=(u-1140)/860 (mapa del firmware v9, k_F' identificado
en el despegue con baterias a 12,4 V). Cada rama (LiPo 3S 3300 mAh) alimenta 4 motores; la potencia electrica
de vuelo estacionario se toma de la tabla A2212 920KV + 9450 de la auditoria (4,2 A a 370 gf, 7,8 A a 600 gf a 11,1 V),
escalada con T^1.5 a la masa real de 3,6 kg. Caida: R interna (EST) + lazo de cable 16,6 mOhm (auditoria) + contactos."""
import os as _os
ROOT=_os.environ.get("AEROTRAY_ROOT",_os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"..","..")))
FIGDIR=_os.environ.get("FIG",_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"figuras")); _os.makedirs(FIGDIR,exist_ok=True)

import numpy as np, json, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from params import P, g
plt.rcParams.update({"font.family":"serif","font.size":8.5,"axes.grid":True,"grid.alpha":0.3,"axes.spines.top":False,
  "axes.spines.right":False,"legend.fontsize":7,"savefig.bbox":"tight","savefig.dpi":300,"mathtext.fontset":"cm",
  "axes.formatter.use_locale":False})
FIG=FIGDIR
# --- curva OCV-SoC de una celda LiPo en reposo (tabla habitual de fabricantes, [EST])
SOC=np.array([0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100])/100
OCV=np.array([3.27,3.61,3.69,3.71,3.73,3.75,3.77,3.79,3.80,3.82,3.84,3.85,3.87,3.91,3.95,3.98,4.02,4.08,4.11,4.15,4.20])*3
soc_of=lambda v: np.interp(v,OCV,SOC); ocv_of=lambda s: np.interp(s,SOC,OCV)
W=(P['m']+P['mc'])*g                     # 35,3 N
Th=W/8                                   # 4,41 N por motor
# potencia electrica por motor en estacionario: P = 46,6 W (370 gf) * (T/370 gf)^1,5
P370=4.2*11.1; Pm=P370*(Th/(0.370*g))**1.5
Pb=4*Pm                                  # por rama (W)
Rext=0.0166+0.006                        # cable 14 AWG ida y vuelta + fusible, rele, XT60, interruptor
def Rpack(soc,rc=0.010): return 3*rc*(1+0.5*np.clip((0.40-soc)/0.30,0,1))
def vbus(ocv,soc,rc=0.010,Pbr=Pb):
    R=Rpack(soc,rc)+Rext; d=ocv**2-4*R*Pbr
    return np.where(d>0,(ocv+np.sqrt(np.maximum(d,0)))/2,np.nan)
x=lambda u:(u-P['u0'])/P['uspan']; u=lambda xx:P['u0']+P['uspan']*xx
uL=np.array([1725.,1775.])               # banda de despegue medida con 12,4 V
OCVref=12.4; socref=soc_of(OCVref)
def ureq(ocv,rc=0.010,uref=1750.):
    s=soc_of(ocv); vb=vbus(ocv,s,rc); vr=vbus(OCVref,socref,rc)
    return u(x(uref)*(vr/vb)**(2/P['n_th']))
CEIL={"flotar":1750.,"continuo":1850.,"maximo":1950.}
res={"W_N":W,"T_motor_N":Th,"P_motor_W":Pm,"P_rama_W":Pb,"soc_12v4":socref,"soc_11v1":float(soc_of(11.1))}
vr=float(vbus(OCVref,socref)); res["Vbus_12v4"]=vr; res["Irama_12v4"]=Pb/vr
v11=float(vbus(11.1,soc_of(11.1))); res["Vbus_11v1"]=v11; res["Irama_11v1"]=Pb/v11
# colectivo de despegue requerido a 11,1 V en reposo y a 11,1 V bajo carga (OCV equivalente)
res["ureq_11v1_reposo"]=[float(ureq(11.1,rc,ul)) for rc in (0.006,0.015) for ul in uL]
# si 11,1 V se midio con carga: OCV tal que vbus=11,1
from scipy.optimize import brentq
ocv_c=brentq(lambda o: vbus(o,soc_of(o))-11.1,11.1,12.6); res["ocv_si_11v1_con_carga"]=ocv_c; res["soc_si_11v1_con_carga"]=float(soc_of(ocv_c))
res["ureq_11v1_carga"]=[float(ureq(ocv_c,0.010,ul)) for ul in uL]
# margen T/W disponible a cada techo con 12,4 V
res["TW_12v4"]={k:[float((x(c)/x(ul))**P['n_th']) for ul in uL] for k,c in CEIL.items()}
# tension minima (OCV en reposo) para despegar con cada techo, banda (k_F' y R interna)
def ocv_min(ceil,rc,ul):
    f=lambda o: ureq(o,rc,ul)-ceil
    try: return brentq(f,10.0,12.4)
    except ValueError: return np.nan
res["ocv_min"]={k:[float(ocv_min(c,rc,ul)) for rc in (0.006,0.015) for ul in uL] for k,c in CEIL.items()}
# autonomia: tiempo desde 12,4 V (y desde 12,6 V) hasta que el despegue exige el techo continuo (1850 us), con I=Pb/Vbus
def t_hasta(ocv0,ceil,rc,ul,soc_stop=0.0,dt=1.0):
    s=soc_of(ocv0); t=0.
    while s>soc_stop and t<3600:
        o=ocv_of(s);
        if ureq(o,rc,ul)>ceil: break
        I=Pb/float(vbus(o,s,rc)); s-=I*dt/3600/3.3; t+=dt
    return t/60, s
res["t_1850_desde_12v4"]=[t_hasta(12.4,1850.,rc,ul)[0] for rc in (0.006,0.015) for ul in uL]
res["t_1850_desde_12v6"]=[t_hasta(12.6,1850.,rc,ul)[0] for rc in (0.006,0.015) for ul in uL]
res["t_80DoD_desde_12v6"]=t_hasta(12.6,9e9,0.010,1750.,soc_stop=0.20)[0]
res["Q_logs_tarde_Ah"]=1.01
json.dump(res,open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"resultados","bateria.json"),"w"),indent=1,default=float)
for k,v in res.items(): print(k, np.round(v,3) if not isinstance(v,dict) else {a:np.round(b,3) for a,b in v.items()})
# --------------------------- figura
o=np.linspace(10.6,12.6,200)
fig,ax=plt.subplots(1,2,figsize=(6.6,2.7))
lo=np.minimum.reduce([ureq(o,rc,ul) for rc in (0.006,0.015) for ul in uL]); hi=np.maximum.reduce([ureq(o,rc,ul) for rc in (0.006,0.015) for ul in uL])
ax[0].fill_between(o,lo,np.minimum(hi,2050),color="#1f77b4",alpha=0.3,lw=0,label="colectivo de despegue (banda)")
ax[0].plot(o,ureq(o,0.010,1750.),color="#1f77b4",lw=1.2)
for (k,c),ls in zip(CEIL.items(),(":","--","-.")):
    ax[0].axhline(c,color="k",lw=0.8,ls=ls);
lab={"flotar":"orden «flotar» 1750 µs","continuo":"límite sostenido 1850 µs","maximo":"máximo 1950 µs (≤ 20 s)"}
for k,c in CEIL.items(): ax[0].text(12.57,c+6,lab[k],fontsize=6.5,ha="left")
ax[0].axvline(12.4,color="#2ca02c",lw=1); ax[0].text(12.37,1655,"flotó (12,4 V)",color="#2ca02c",fontsize=6.5,ha="left",va="bottom")
ax[0].axvline(11.1,color="#d62728",lw=1); ax[0].text(11.07,1655,"no flotó (11,1 V)",color="#d62728",fontsize=6.5,ha="left")
ax[0].set_xlim(10.6,12.6); ax[0].set_ylim(1650,2050); ax[0].invert_xaxis()
ax[0].set_xlabel("Tensión de la LiPo en reposo (V)"); ax[0].set_ylabel("PWM colectivo (µs)")
sec=ax[0].secondary_xaxis("top",functions=(lambda v: 100*soc_of(v), lambda s: ocv_of(np.asarray(s)/100)))
sec.set_xlabel("Estado de carga (%)",fontsize=7.5); sec.set_xticks([10,30,50,70,90])
ax[0].set_title("(a)",loc="left",fontsize=9)
for k,c,col in (("continuo",1850.,"#ff7f0e"),("maximo",1950.,"#9467bd")):
    tw=[(x(c)/x(ureq(o,rc,ul)))**P['n_th'] for rc in (0.006,0.015) for ul in uL]
    ax[1].fill_between(o,np.minimum.reduce(tw),np.maximum.reduce(tw),color=col,alpha=0.3,lw=0)
    ax[1].plot(o,(x(c)/x(ureq(o,0.010,1750.)))**P['n_th'],color=col,lw=1.2,label=f"empuje/peso a {int(c)} µs")
ax[1].axhline(1.0,color="k",lw=0.8); ax[1].text(12.57,1.02,"empuje = peso",fontsize=6.5,ha="left")
ax[1].axvline(12.4,color="#2ca02c",lw=1); ax[1].axvline(11.1,color="#d62728",lw=1)
ax[1].set_xlim(10.6,12.6); ax[1].invert_xaxis(); ax[1].set_ylim(0.7,1.9)
ax[1].set_xlabel("Tensión de la LiPo en reposo (V)"); ax[1].set_ylabel("Empuje máximo / peso")
ax[1].legend(loc="upper right"); ax[1].set_title("(b)",loc="left",fontsize=9)
for a in ax:
    a.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v,p: f"{v:.1f}".replace(".",",")))
ax[1].yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v,p: f"{v:.1f}".replace(".",",")))
fig.tight_layout(); fig.savefig(f"{FIG}/fig_bateria.pdf"); plt.close(fig)
