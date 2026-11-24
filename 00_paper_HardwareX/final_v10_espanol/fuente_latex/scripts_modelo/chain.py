import os as _os
ROOT=_os.environ.get("AEROTRAY_ROOT",_os.path.abspath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)),"..","..")))
import numpy as np, pandas as pd, json, time, os, sys
from params import P, g
from load import reg
h=1/P['fps']
def lsw(W=0.06):
    N=int(np.floor(W/h))+1; t=-np.arange(N)*h; tb=t.mean(); return (t-tb)/np.sum((t-tb)**2)
W8=lsw()
def ball_closed(w,Kb=4.2,T=0.10,tau_s=0.05):
    s=1j*w; kap=P['kappa']*1000  # mm
    G=Kb*(1-kap/(g*1000)*s**2)/(s**2*(tau_s*s+1))
    Hv=sum(wk*np.exp(-s*k*h) for k,wk in enumerate(W8))
    K=P['kp_b']+P['ki_b']/s+P['kd_b']*Hv
    L=G*K*np.exp(-s*T)
    S=1/(1+L)                    # sensitivity
    XD=(1/s**2)*S                # mm per (mm/s^2) of equivalent acceleration disturbance
    return XD,L
def drone_closed(w,kT=1.37,hc=P['hc'],J=P['J'][1],ki_th=P['ki_th_fl']):
    s=1j*w; m=P['m']
    Pl=1/(J*s**2-m*g*hc)
    A=kT*np.exp(-s*P['T_esc'])/(P['tau_m']*s+1)
    Cw=P['kp_w']+P['ki_w']/s
    Q=P['kp_th']+ki_th/s+P['kd_th']*s/(P['tf_d']*s+1)+s
    L=Pl*A*Cw*Q
    TA=-m*hc*Pl/(1+L)            # theta per (m/s^2) of joint acceleration
    Tdist=Pl/(1+L)               # theta per N m disturbance torque
    return TA,Tdist,L
w=np.logspace(-2,2.3,3000)
XD,Lb=ball_closed(w)
TA,Td,Ld=drone_closed(w)
s=1j*w
lever=(P['hP']+P['kappa'])
# X per unit theta (rad) -> mm : (5/7)*(g - lever s^2)*1000 * XD
X_theta=5/7*(g-lever*s**2)*1000*XD
X_aJ_direct=-5/7*1000*XD
X_aJ_total=X_aJ_direct+X_theta*TA
res=dict(w=w.tolist())
for k,v in dict(X_theta=X_theta,X_aJ_direct=X_aJ_direct,X_aJ_total=X_aJ_total,TA=TA,Td=Td,XD=XD).items():
    res[k]=np.abs(v).tolist()
# peak values
def pk(a): i=np.argmax(np.abs(a)); return float(w[i]),float(np.abs(a)[i])
summ=dict(X_theta_peak=pk(X_theta),X_aJ_total_peak=pk(X_aJ_total),TA_peak=pk(TA),X_aJ_dc=float(np.abs(X_aJ_total[0])))
# --- predicted ball RMS from measured drone attitude spectrum (bench, idle, 18/11)
V=_os.path.join(ROOT,"08_datos_y_registros","registros_vuelo_dron")
from scipy.signal import welch
tot=[]
for f in ("dron_log_20261118_144610.csv","dron_log_20261118_144932.csv"):
    d=pd.read_csv(os.path.join(V,f)); t=d.t_us.values*1e-6; fs=1/np.median(np.diff(t))
    for ax,Xf in (("pitch_deg",X_theta),("roll_deg",X_theta)):
        th=np.radians(d[ax].values); th=th-th.mean()
        fr,S=welch(th,fs=fs,nperseg=512)
        wf=2*np.pi*fr[1:]; Hm=np.interp(wf,w,np.abs(Xf))
        var=np.trapezoid(Hm**2*S[1:],fr[1:])
        tot.append((f,ax,float(np.degrees(th.std())),float(np.sqrt(var))))
summ['pred_ball_from_attitude']=tot
# --- measured ball spectrum drone on, base still (17/11 19:29) and drone off sway
d=reg("registro_20261117_162914.csv"); t=d.tiempo_unix.values
def tt(sx): return time.mktime(time.strptime("2026-11-17 "+sx,"%Y-%m-%d %H:%M:%S"))
for lab,a0,dur in (("drone_on","19:29:20",140),("drone_on2","19:31:46",108),("drone_on3","19:34:14",170)):
    m=(t>=tt(a0))&(t<=tt(a0)+dur)&(d.estado.values==3)&~np.isnan(d.x_mm.values)
    tg=np.arange(t[m][0],t[m][-1],h); 
    for ax in ("x_mm","y_mm"):
        xs=np.interp(tg,t[m],d[ax].values[m]); xs=xs-xs.mean()
        fr,S=welch(xs,fs=1/h,nperseg=4096)
        k=np.argmax(S[1:])+1
        # band powers
        lo=np.trapezoid(S[(fr<0.3)],fr[(fr<0.3)]); mid=np.trapezoid(S[(fr>=0.3)&(fr<3)],fr[(fr>=0.3)&(fr<3)]); hi=np.trapezoid(S[fr>=3],fr[fr>=3])
        summ[f'meas_{lab}_{ax}']=dict(rms=float(xs.std()),f_peak=float(fr[k]),rms_lt0p3Hz=float(np.sqrt(lo)),rms_0p3_3Hz=float(np.sqrt(mid)),rms_gt3Hz=float(np.sqrt(hi)))
json.dump(res,open('chain_freq.json','w')); json.dump(summ,open('chain_summary.json','w'),indent=1)
import pprint; pprint.pprint(summ,width=140)
