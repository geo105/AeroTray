"""Calcula los datos de cada figura del paper (modelo y registros) y los guarda como CSV para los scripts
de MATLAB de figuras_matlab/. Python calcula; MATLAB solo dibuja.
Uso: AEROTRAY_ROOT=<carpeta del repositorio> python3 exportar_datos_matlab.py"""
import os as _os
ROOT=_os.environ.get("AEROTRAY_ROOT",_os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"..","..")))
OUT=_os.environ.get("DATOS",_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"..","figuras_matlab","datos"))
_os.makedirs(OUT,exist_ok=True)
import numpy as np, pandas as pd, os, json
from params import P, g
from analysis import out as AN
import analysis as an, ball_dt as bd, chain as ch
def W(name,df): df.to_csv(os.path.join(OUT,name),index=False,float_format="%.6g"); print("  ",name,df.shape)
V=ROOT+"/08_datos_y_registros/registros_vuelo_dron"
ma=P['m']+P['mc']
# ------------------------------------------------ 1. identificacion (balance de momentos + mapa de empuje)
L=.35; D=L/np.sqrt(2); X=np.array([D,-D,-D,D,0,-L,0,L]); Y=np.array([-D,-D,D,D,-L,0,L,0])
runs=[("vuelo_20261120_093101.csv",3),("vuelo_20261120_093636.csv",4),("vuelo_20261120_094224.csv",5),("vuelo_20261120_150927.csv",6),
      ("vuelo_20261120_152249.csv",7),("vuelo_20261120_152743.csv",8),("vuelo_20261120_154605.csv",9)]
rows=[]
for f,lab in runs:
    d=pd.read_csv(os.path.join(V,f)); d=d[d.modo=="CONTROL"]
    Mu=d[[f"M{i}_us" for i in range(1,9)]].values.astype(float); col=Mu.mean(1)
    k=(col>1650)&(np.abs(d.p_dps.values)<15)&(np.abs(d.q_dps.values)<15)
    Fi=np.clip((Mu[k]-1140)/860,0,None)**1.6; Ft=Fi.sum(1)
    rows.append(dict(vuelo=lab,theta_deg=np.degrees(np.sin(np.radians(d.pitch_deg.values[k])).mean()),
        phi_deg=np.degrees(np.sin(np.radians(d.roll_deg.values[k])).mean()),
        xcp_mm=1000*((Fi*X).sum(1)/Ft).mean(),ycp_mm=1000*((Fi*Y).sum(1)/Ft).mean()))
R=pd.DataFrame(rows); c=P['m']/ma
A=[];b=[]
for r in rows: A+=[[c,0,c*np.radians(r['theta_deg'])],[0,c,-c*np.radians(r['phi_deg'])]]; b+=[r['xcp_mm']/1000,r['ycp_mm']/1000]
A=np.array(A); b=np.array(b); th,*_=np.linalg.lstsq(A,b,rcond=None)
W("ident_puntos.csv",R)
gg=np.linspace(-2,4.2,20)
W("ident_ajuste.csv",pd.DataFrame(dict(ang_deg=gg,x_mm=1000*c*(th[0]+th[2]*np.radians(gg)),y_mm=1000*c*(th[1]-th[2]*np.radians(gg)))))
W("ident_param.csv",pd.DataFrame([dict(dx_mm=1000*th[0],dy_mm=1000*th[1],hc_m=th[2],peso_N=ma*g)]))
u=np.linspace(1140,2000,300); F=lambda u,k: 8*k*np.clip((u-1140)/860,0,None)**1.6
W("empuje.csv",pd.DataFrame(dict(u_us=u,T_firmware=F(u,P['kF_fw']),T_id_min=F(u,7.17),T_id_max=F(u,8.18))))
# ------------------------------------------------ 2. estabilidad del dron
th_=np.linspace(0,14,141); m=P['m']
W("autoridad_requerida.csv",pd.DataFrame(dict(theta_deg=th_,tau_hc_id=m*g*(P['dx']+P['hc']*np.sin(np.radians(th_))),
    tau_hc_cad=m*g*(P['dx']+P['hc_cad']*np.sin(np.radians(th_))))))
disp=[]
for lab,wl,Il in (("banco",P['wlim_bench'],P['Ilim_bench']),("vuelo",P['wlim_fl'],P['Ilim_fl'])):
    for kT in (1.0,1.42): disp.append(dict(limites=lab,kT=kT,tau=kT*(P['kp_w']*wl+P['ki_w']*Il)))
W("autoridad_disponible.csv",pd.DataFrame(disp))
hcs=np.linspace(0.0,0.25,51); kTs=np.linspace(0.6,2.0,57)
for J_,tag in ((0.12,"012"),(0.09,"009"),(0.15,"015")):
    Z=np.zeros((len(kTs),len(hcs)))
    for i,kT in enumerate(kTs):
        for j,h in enumerate(hcs):
            p,_=an.cl_poles(kT,h,J_=J_); Z[i,j]=np.max(p.real)
    M=np.full((len(kTs)+1,len(hcs)+1),np.nan); M[0,1:]=hcs; M[1:,0]=kTs; M[1:,1:]=Z
    np.savetxt(os.path.join(OUT,f"estabilidad_J{tag}.csv"),M,delimiter=",",fmt="%.6g"); print("   estabilidad_J%s.csv"%tag,M.shape)
# ------------------------------------------------ 3. lazo de la bola + cuatro barras
Ts=np.linspace(0.03,0.18,16); out={"T_ms":1000*Ts}
for Kb in (6.0,4.9,3.5,2.5):
    kc=[]
    for T in Ts:
        kds=np.linspace(0.2,14,139); ok=np.array([bd.stable(Kb,T,P['kp_b'],P['ki_b'],k/3)[0] for k in kds]); kc.append(kds[ok].max() if ok.any() else np.nan)
    out["Kb_%s"%str(Kb).replace(".","_")]=kc
W("kd_critico.csv",pd.DataFrame(out))
gam,alp=AN['ball']['linkage_curve']
W("cuatro_barras.csv",pd.DataFrame(dict(gamma_deg=gam,alpha_deg=alp)))
# ------------------------------------------------ 4. transmisibilidad
w=np.logspace(-2,2,1200); s=1j*w; lever=P['hP']+P['kappa']; o={"f_Hz":w/2/np.pi}
for Kb,T,tag in ((4.2,0.10,"a"),(3.5,0.14,"b")):
    XD,_=ch.ball_closed(w,Kb=Kb,T=T); o["X_theta_"+tag]=np.abs(5/7*(g-lever*s**2)*1000*XD)*np.pi/180
for hcv,tag in ((0.14,"a"),(0.10,"b")):
    TA,_,_=ch.drone_closed(w,kT=1.42,hc=hcv); o["theta_A_"+tag]=np.degrees(np.abs(TA))
XD,_=ch.ball_closed(w); TA,_,_=ch.drone_closed(w,kT=1.42)
o["X_A_directo"]=np.abs(5/7*1000*XD); o["X_A_total"]=np.abs(5/7*1000*XD*((g-lever*s**2)*TA-1))
W("transmisibilidad.csv",pd.DataFrame(o))
# ------------------------------------------------ 5. base: velocidad y vuelco
Mt=AN['base']['M']; be=np.linspace(0,25,126); o={"beta_deg":be}
for uu in (0.15,0.30,0.50):
    o["v_%d"%int(100*uu)]=np.clip([P['r']*P['w0']*(uu-P['r']/2*Mt*g*(np.sin(np.radians(x))+P['Crr']*np.cos(np.radians(x)))/P['tau_s']) for x in be],0,None)
for hg,tag in ((AN['base']['hG_rigid'],"fijo"),(AN['base']['hG_level'],"nivelado")):
    o["amax_"+tag]=g*(np.cos(np.radians(be))*AN['base']['xG']/hg-np.sin(np.radians(be)))
W("base_limites.csv",pd.DataFrame(o))
# ------------------------------------------------ 6. rampa: descarga de peso (a_max con la inercia completa del modulo)
Mb,xb,hb=P['M_base'],P['xG_base'],P['hG_base']; lJ,hJ,Lw,Crr=P['lJ'],P['hJ0'],P['Lwb'],P['Crr']
def rampa(f,beta):
    B=np.radians(beta); M=Mb+(1-f)*ma; Sx=Mb*xb+(1-f)*ma*lJ; Sh=Mb*hb+(1-f)*ma*hJ; I=Mb*hb+ma*hJ
    xG,hG=Sx/M,Sh/M
    amax=g*(Sx*np.cos(B)-Sh*np.sin(B))/I            # el empuje quita peso, no masa
    Nr=M*g*((Lw-xG)*np.cos(B)+hG*np.sin(B))/Lw
    return dict(vuelco_atras=np.degrees(np.arctan(xG/hG)),vuelco_adelante=np.degrees(np.arctan((Lw-xG)/hG)),
                amax=amax,mu=M*g*(np.sin(B)+Crr*np.cos(B))/Nr)
ff=np.linspace(0,1,101); o={"f":ff,"u_col_us":P['u0']+P['uspan']*(ff*ma*g/(8*7.6))**(1/P['n_th'])}
for k in ("vuelco_atras","vuelco_adelante"): o[k]=[rampa(f,0)[k] for f in ff]
for bta in (10,20):
    o["amax_%d"%bta]=[rampa(f,bta)["amax"] for f in ff]; o["mu_%d"%bta]=[rampa(f,bta)["mu"] for f in ff]
W("rampa.csv",pd.DataFrame(o))
W("rampa_param.csv",pd.DataFrame([dict(xG_base_m=xb,hG_base_m=hb,f_ralenti=8*7.6*((1300-1140)/860)**1.6/(ma*g))]))
# ------------------------------------------------ 7. bateria
import bateria as B
oc=np.linspace(10.6,12.6,201); rows=[]
for o_ in oc:
    us=[float(B.ureq(o_,rc,ul)) for rc in (0.006,0.015) for ul in (1725.,1775.)]
    uc=float(B.ureq(o_,0.010,1750.))
    rows.append(dict(Voc=o_,soc_pct=100*float(B.soc_of(o_)),u_c=uc,u_min=min(us),u_max=max(us),
        tw1850_c=(B.x(1850.)/B.x(uc))**1.6,tw1850_min=min((B.x(1850.)/B.x(v))**1.6 for v in us),tw1850_max=max((B.x(1850.)/B.x(v))**1.6 for v in us),
        tw1950_c=(B.x(1950.)/B.x(uc))**1.6,tw1950_min=min((B.x(1950.)/B.x(v))**1.6 for v in us),tw1950_max=max((B.x(1950.)/B.x(v))**1.6 for v in us)))
W("bateria.csv",pd.DataFrame(rows))
W("soc_ticks.csv",pd.DataFrame(dict(soc_pct=[10,30,50,70,90],Voc=[float(B.ocv_of(s/100)) for s in (10,30,50,70,90)])))
# ------------------------------------------------ 8. campana de vuelo
FL=[("vuelo_20261120_091438.csv","v2"),("vuelo_20261120_092412.csv","v3"),("vuelo_20261120_093101.csv","v4"),("vuelo_20261120_093636.csv","v5"),
    ("vuelo_20261120_094224.csv","v6"),("vuelo_20261120_150927.csv","v6"),("vuelo_20261120_152249.csv","v8"),("vuelo_20261120_152743.csv","v9"),("vuelo_20261120_154605.csv","v9")]
def rd(f):
    d=pd.read_csv(os.path.join(V,f)); t=d.t_pc.values-d.t_pc.values[0]
    Mm=d[[f"M{i}_us" for i in range(1,9)]].values.astype(float).mean(1); return t,Mm,d.roll_deg.values,d.pitch_deg.values
rows=[]
for i,(f,v) in enumerate(FL):
    t,col,ro,pi=rd(f); mk=col>1650
    rows.append(dict(vuelo=i+1,fw=v,alabeo_rms=np.sqrt(np.mean(ro[mk]**2)) if mk.sum()>5 else np.nan,cabeceo_rms=np.sqrt(np.mean(pi[mk]**2)) if mk.sum()>5 else np.nan))
W("vuelos_rms.csv",pd.DataFrame(rows))
for f,v in (("vuelo_20261120_152249.csv","v8"),("vuelo_20261120_152743.csv","v9")):
    t,col,ro,pi=rd(f)
    W(f"vuelo_serie_{v}.csv",pd.DataFrame(dict(t_s=t,inclinacion_deg=np.degrees(np.arccos(np.cos(np.radians(ro))*np.cos(np.radians(pi)))),colectivo_us=col)))
# ------------------------------------------------ 9. dron en banco
for f in ("dron_log_20261118_144610.csv","dron_log_20261118_144932.csv"):
    d=pd.read_csv(os.path.join(V,f)); t=(d.t_us-d.t_us.iloc[0])/1e6
    W("banco_"+f[9:24]+".csv",pd.DataFrame(dict(t_s=t,alabeo_deg=d.roll_deg,cabeceo_deg=d.pitch_deg)))
print("datos en",os.path.abspath(OUT))
