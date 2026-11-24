import os as _os
ROOT=_os.environ.get("AEROTRAY_ROOT",_os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"..","..")))
FIGDIR=_os.environ.get("FIG",_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"figuras")); _os.makedirs(FIGDIR,exist_ok=True)
import numpy as np, json, os, pandas as pd, sys
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

import re as _re, matplotlib.ticker as _tk, matplotlib.text as _mt
def es(fig):
    """Coma decimal en textos y ejes lineales."""
    for t in fig.findobj(_mt.Text):
        tx=t.get_text()
        if tx: t.set_text(_re.sub(r"(\d)\.(\d)",r"\1,\2",tx))
    for a in fig.axes:
        for ax_ in (a.xaxis,a.yaxis):
            if ax_.get_scale()=="linear" and isinstance(ax_.get_major_formatter(),_tk.ScalarFormatter):
                ax_.set_major_formatter(_tk.FuncFormatter(lambda v,p: ("%g"%round(v,6)).replace(".",",").replace("-","\u2212")))
from params import P, g
from analysis import out as AN   # reuse functions/results
import analysis as an
import ball_dt as bd
import chain as ch
FIG=FIGDIR; os.makedirs(FIG,exist_ok=True)
plt.rcParams.update({"font.family":"serif","font.size":8.5,"axes.grid":True,"grid.alpha":0.3,"axes.spines.top":False,
  "axes.spines.right":False,"legend.fontsize":7.5,"savefig.bbox":"tight","savefig.dpi":300,"mathtext.fontset":"cm"})
W=6.3; C1,C2,C3,C4="#1f77b4","#d62728","#2ca02c","#7f7f7f"
res={}
# ---------------- Fig A: identification (moment balance + thrust)
V=ROOT+"/08_datos_y_registros/registros_vuelo_dron"
L=.35; D=L/np.sqrt(2); X=np.array([D,-D,-D,D,0,-L,0,L]); Y=np.array([-D,-D,D,D,-L,0,L,0])
runs=[("vuelo_20261120_093101.csv","3"),("vuelo_20261120_093636.csv","4"),("vuelo_20261120_094224.csv","5"),("vuelo_20261120_150927.csv","6"),("vuelo_20261120_152249.csv","7"),("vuelo_20261120_152743.csv","8"),("vuelo_20261120_154605.csv","9")]
rows=[]
for f,lab in runs:
    d=pd.read_csv(os.path.join(V,f)); d=d[d.modo=="CONTROL"]
    Mu=d[[f"M{i}_us" for i in range(1,9)]].values.astype(float); col=Mu.mean(1)
    k=(col>1650)&(np.abs(d.p_dps.values)<15)&(np.abs(d.q_dps.values)<15)
    Fi=np.clip((Mu[k]-1140)/860,0,None)**1.6; Ft=Fi.sum(1)
    xcp=(Fi*X).sum(1)/Ft; ycp=(Fi*Y).sum(1)/Ft
    th=np.radians(d.pitch_deg.values[k]); ph=np.radians(d.roll_deg.values[k])
    rows.append(dict(run=lab,n=int(k.sum()),xcp=xcp.mean(),ycp=ycp.mean(),sx=xcp.std()/np.sqrt(k.sum()/10),sy=ycp.std()/np.sqrt(k.sum()/10),sth=np.sin(th).mean(),sph=np.sin(ph).mean()))
c=P['m']/(P['m']+P['mc'])
A=[];b=[]
for r in rows: A+= [[c,0,c*r['sth']],[0,c,-c*r['sph']]]; b+=[r['xcp'],r['ycp']]
A=np.array(A); b=np.array(b); th_,*_=np.linalg.lstsq(A,b,rcond=None); e=b-A@th_; s2=e@e/(len(b)-3); cov=s2*np.linalg.inv(A.T@A)
res['moment_balance']=dict(dx=th_[0],dy=th_[1],hc=th_[2],sd=np.sqrt(np.diag(cov)).tolist(),rows=rows,resid_rms=float(np.sqrt(e@e/len(e))))
fig,ax=plt.subplots(1,2,figsize=(W,2.45),gridspec_kw={"width_ratios":[1.15,1]})
sth=np.array([r['sth'] for r in rows]); sph=np.array([r['sph'] for r in rows])
xc=np.array([r['xcp'] for r in rows]); yc=np.array([r['ycp'] for r in rows])
ax[0].scatter(np.degrees(sth),1000*xc,color=C1,s=18,label=r"$x_{cp}$ frente a cabeceo $\bar\theta$",zorder=3)
ax[0].scatter(np.degrees(sph),1000*yc,color=C2,marker="s",s=16,label=r"$y_{cp}$ frente a alabeo $\bar\phi$",zorder=3)
for r in rows: ax[0].annotate(r['run'],(np.degrees(r['sth']),1000*r['xcp']),fontsize=6,xytext=(3,2),textcoords="offset points")
gg=np.linspace(-2,4.2,10)
ax[0].plot(gg,1000*c*(th_[0]+th_[2]*np.radians(gg)),color=C1,lw=1)
ax[0].plot(gg,1000*c*(th_[1]-th_[2]*np.radians(gg)),color=C2,lw=1,ls="--")
ax[0].set_xlabel("Inclinación media en vuelo estacionario (°)"); ax[0].set_ylabel("Centro de empuje (mm)")
ax[0].legend(loc="upper left"); ax[0].set_title(r"(a) $\hat\delta_x$=%.1f mm, $\hat\delta_y$=%.1f mm, $\hat h_c$=%.2f m"%(1000*th_[0],1000*th_[1],th_[2]),fontsize=8)
u=np.linspace(1140,2000,300)
F=lambda u,k: k*np.clip((u-1140)/860,0,None)**1.6
ax[1].plot(u,8*F(u,P['kF_fw']),color=C4,label=r"mapa del firmware, $k_F$=5,40 N")
ax[1].fill_between(u,8*F(u,7.17),8*F(u,8.18),color=C3,alpha=0.3,label=r"identificado, $k_F$=7,17–8,18 N")
ax[1].axhline((P['m']+P['mc'])*g,color="k",ls=":",lw=0.9); ax[1].text(1310,(P['m']+P['mc'])*g+1.2,"peso, 3,6 kg",fontsize=6.5)
ax[1].axvspan(1725,1775,color=C3,alpha=0.15); ax[1].axvline(1899,color=C2,ls="--",lw=0.9); ax[1].text(1905,3,"estacionario\nmapa del\nfirmware\n1899 µs",fontsize=6.3,color=C2)
ax[1].axvline(1770,color=C1,ls="-.",lw=0.9); ax[1].text(1640,52,"auditoría\nde potencia\n1770 µs",fontsize=6.5,color=C1,ha="right")
ax[1].set_xlim(1300,2000); ax[1].set_ylim(0,62); ax[1].set_xlabel("Orden colectiva $u$ (µs)"); ax[1].set_ylabel("Empuje total (N)")
ax[1].legend(loc="lower right",fontsize=6.3); ax[1].set_title("(b) mapa de empuje (banda de despegue sombreada)",fontsize=8)
es(fig); fig.tight_layout(); fig.savefig(f"{FIG}/fig_ident.pdf"); plt.close(fig)
# ---------------- Fig B: drone authority + stability map
fig,ax=plt.subplots(1,2,figsize=(W,2.5))
th=np.linspace(0,14,200); m=P['m']
for h,ls in ((P['hc'],"-"),(P['hc_cad'],"--")):
    ax[0].plot(th,m*g*(P['dx']+h*np.sin(np.radians(th))),color="k",ls=ls,lw=1,label=(r"necesario, $h_c$=%.2f m"%h).replace(".",","))
for lab,wl,Il,col in (("v4–v7 (25°/s, $I_\\omega$≤0,3)",P['wlim_bench'],P['Ilim_bench'],C2),("v8–v9 en vuelo (60°/s, $I_\\omega$≤3)",P['wlim_fl'],P['Ilim_fl'],C3)):
    for kT,a in ((1.0,0.9),(1.42,0.5)):
        ta=kT*(P['kp_w']*wl+P['ki_w']*Il)
        ax[0].axhline(ta,color=col,lw=1.2 if kT==1 else 0.8,alpha=a,label=(f"disponible, {lab}" if kT==1 else None))
    tst=np.degrees(np.arcsin((P['kp_w']*wl+P['ki_w']*Il)/(m*g)-P['dx'])/P['hc']) if False else None
ax[0].axvline(12,color=C4,ls=":",lw=0.9); ax[0].text(12.2,0.1,"corte 12°",fontsize=6.5,rotation=90)
ax[0].set_xlabel(r"Inclinación $|\theta|$ (°)"); ax[0].set_ylabel("Par (N m)"); ax[0].set_ylim(0,1.9); ax[0].legend(fontsize=6.3,loc="upper left")
ax[0].set_title("(a) autoridad de par (líneas finas: $k_T$=1,42)",fontsize=8)
hcs=np.linspace(0.0,0.25,51); kTs=np.linspace(0.6,2.0,57)
for J_,ls,lab in ((P['J'][1],"-","$J$=0.12"),(0.09,"--","$J$=0.09"),(0.15,":","$J$=0.15")):
    Z=np.zeros((len(kTs),len(hcs)))
    for i,kT in enumerate(kTs):
        for j,h in enumerate(hcs):
            p,_=an.cl_poles(kT,h,J_=J_); Z[i,j]=np.max(p.real)
    cs=ax[1].contour(hcs,kTs,Z,levels=[0],colors="k",linestyles=ls,linewidths=1)
    if J_==P['J'][1]:
        ax[1].contourf(hcs,kTs,Z,levels=[-50,0],colors=[C3],alpha=0.15)
        res['stab_map_J012']=dict(hcs=hcs.tolist(),kTs=kTs.tolist(),Z=Z.tolist())
    ax[1].plot([],[],color="k",ls=ls,lw=1,label=lab)
ax[1].add_patch(plt.Rectangle((0.08,1.33),0.12,0.19,fill=False,ec=C2,lw=1.2))
ax[1].plot(0.14,1.42,"o",color=C2,ms=4); ax[1].text(0.082,1.56,"identificado",color=C2,fontsize=6.8)
ax[1].plot(0.10,1.0,"s",color=C1,ms=4); ax[1].text(0.003,0.98,"$h_c$ del CAD,\nmapa del firmware",color=C1,fontsize=6.5)
ax[1].text(0.02,1.8,"estable",fontsize=7,color=C3); ax[1].text(0.19,0.7,"inestable",fontsize=7,color=C2)
ax[1].set_xlabel(r"Altura del CM sobre la rótula $h_c$ (m)"); ax[1].set_ylabel(r"Razón de ganancias de empuje $k_T$"); ax[1].legend(fontsize=6.5,loc="lower right")
ax[1].set_title("(b) estabilidad lineal del lazo de actitud (v9)",fontsize=8); ax[1].grid(False)
es(fig); fig.tight_layout(); fig.savefig(f"{FIG}/fig_drone_stability.pdf"); plt.close(fig)
# ---------------- Fig C: ball loop stability + linkage
fig,ax=plt.subplots(1,2,figsize=(W,2.45),gridspec_kw={"width_ratios":[1.25,1]})
Ts=np.linspace(0.03,0.18,16)
for Kb,col in ((6.0,C4),(4.9,C1),(3.5,C3),(2.5,C2)):
    kc=[]
    for T in Ts:
        kds=np.linspace(0.2,14,139); ok=np.array([bd.stable(Kb,T,P['kp_b'],P['ki_b'],k/3)[0] for k in kds]); kc.append(kds[ok].max() if ok.any() else np.nan)
    ax[0].plot(1000*Ts,kc,color=col,lw=1.2,label=(r"$K_b$=%.1f (CAD)"%Kb if Kb==6.0 else r"$K_b$=%.1f"%Kb))
    res[f'kdcrit_{Kb}']=list(map(float,kc))
ax[0].axhspan(4.5,6.0,color="orange",alpha=0.18); ax[0].axhline(4.5,color="k",lw=0.8); ax[0].axhline(6,color="k",lw=0.8,ls="--")
ax[0].text(120,4.62,"$k_d$=4,5 usado: estable",fontsize=6.5); ax[0].text(120,6.12,"$k_d$=6: inestable",fontsize=6.5)
ax[0].set_xlabel("Retardo total del lazo $T$ (ms)"); ax[0].set_ylabel("$k_d$ crítico (unidades del firmware)"); ax[0].set_ylim(0,12); ax[0].legend(fontsize=6.3,loc="upper right",ncol=2)
ax[0].set_title("(a) frontera de estabilidad del lazo de la bola",fontsize=8)
gam,alp=AN['ball']['linkage_curve']
ax[1].plot(gam,alp,color=C1,lw=1.2,label="cuatro barras (CAD v5)"); ax[1].plot(gam,0.436*np.array(gam),color=C4,ls="--",lw=0.9,label="lineal, $G_\\ell$=0,436")
ax[1].axvspan(-220*45/400,220*45/400,color=C3,alpha=0.12,label="rango usado (±220 µs)")
ax[1].set_xlabel(r"Ángulo del servo $\gamma$ (°)"); ax[1].set_ylabel(r"Ángulo del plato $\alpha$ (°)"); ax[1].legend(fontsize=6.5,loc="upper left")
ax[1].set_title("(b) mecanismo del plato",fontsize=8)
es(fig); fig.tight_layout(); fig.savefig(f"{FIG}/fig_ball_stability.pdf"); plt.close(fig)
# ---------------- Fig D: transmissibility
w=np.logspace(-2,2,2000)
fig,ax=plt.subplots(1,3,figsize=(W,2.2))
for Kb,T,ls in ((4.2,0.10,"-"),(3.5,0.14,"--")):
    XD,_=ch.ball_closed(w,Kb=Kb,T=T)
    s=1j*w; lever=P['hP']+P['kappa']
    Xth=5/7*(g-lever*s**2)*1000*XD
    ax[0].loglog(w/2/np.pi,np.abs(Xth)*np.pi/180,color=C1,ls=ls,lw=1,label=r"$K_b$=%.1f, $T$=%d ms"%(Kb,1000*T))
for kT,hcv,ls in ((1.42,0.14,"-"),(1.42,0.10,"--")):
    TA,Td,Ld=ch.drone_closed(w,kT=kT,hc=hcv)
    ax[1].loglog(w/2/np.pi,np.degrees(np.abs(TA)),color=C2,ls=ls,lw=1,label=r"$h_c$=%.2f m"%hcv)
XD,_=ch.ball_closed(w); s=1j*w; lever=P['hP']+P['kappa']
TA,_,_=ch.drone_closed(w,kT=1.42)
Xdir=5/7*1000*XD; Xtot=5/7*1000*XD*((g-lever*s**2)*TA-1)
ax[2].loglog(w/2/np.pi,np.abs(Xdir),color=C4,lw=1,label="camino directo")
ax[2].loglog(w/2/np.pi,np.abs(Xtot),color="k",lw=1,label="directo + a través del dron")
for a,t,yl in ((ax[0],"(a) inclinación del dron → bola","$|X/\\Theta|$ (mm/°)"),(ax[1],"(b) acel. de la base → dron","$|\\Theta/A_J|$ (° por m s$^{-2}$)"),(ax[2],"(c) acel. de la base → bola","$|X/A_J|$ (mm por m s$^{-2}$)")):
    a.set_title(t,fontsize=8); a.set_xlabel("Frecuencia (Hz)"); a.set_ylabel(yl); a.legend(fontsize=6.3); a.set_xlim(2e-3,15)
es(fig); fig.tight_layout(); fig.savefig(f"{FIG}/fig_transmissibility.pdf"); plt.close(fig)
# ---------------- Fig E: base
M=AN['base']['M']; fig,ax=plt.subplots(1,2,figsize=(W,2.3))
be=np.linspace(0,25,200)
for u,col in ((0.15,C1),(0.30,C3),(0.50,C4)):
    v=[an.P['r']*an.P['w0']*(u-an.P['r']/2*M*g*(np.sin(np.radians(x))+an.P['Crr']*np.cos(np.radians(x)))/an.P['tau_s']) for x in be]
    ax[0].plot(be,np.clip(v,0,None),color=col,lw=1.1,label=f"ciclo {int(100*u)} %")
ax[0].errorbar([0],[0.07],yerr=[[0.01],[0.01]],fmt="o",color=C2,ms=4,capsize=3,label="medido, 15 % (LabVIEW)")
ax[0].set_xlabel(r"Pendiente $\beta$ (°)"); ax[0].set_ylabel("Velocidad estacionaria (m/s)"); ax[0].legend(fontsize=6.5); ax[0].set_title("(a) velocidad frente a pendiente",fontsize=8)
for hg,lab,ls in ((AN['base']['hG_rigid'],"dron fijo a la base","-"),(AN['base']['hG_level'],"dron nivelado (rótula libre)","--")):
    am=g*(np.cos(np.radians(be))*AN['base']['xG']/hg-np.sin(np.radians(be)))
    ax[1].plot(be,am,color="k",ls=ls,lw=1.1,label=lab)
ax[1].axhline(0,color=C2,lw=0.8); ax[1].axvline(10,color=C4,ls=":",lw=0.8); ax[1].axvline(20,color=C4,ls=":",lw=0.8)
ax[1].set_xlabel(r"Pendiente $\beta$ (°)"); ax[1].set_ylabel(r"$a_{\max}$ antes de volcar (m/s$^2$)"); ax[1].legend(fontsize=6.5)
ax[1].set_title("(b) margen de vuelco longitudinal",fontsize=8)
es(fig); fig.tight_layout(); fig.savefig(f"{FIG}/fig_base_limits.pdf"); plt.close(fig)
json.dump(res,open("figs_results.json","w"),indent=1,default=float)
print(json.dumps({k:v for k,v in res.items() if k not in ('stab_map_J012',)},indent=1,default=float)[:3000])
