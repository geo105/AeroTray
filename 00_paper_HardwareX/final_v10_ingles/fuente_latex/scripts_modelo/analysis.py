import numpy as np, json
from params import P, g
from scipy import optimize
from math import factorial
class signal:
    @staticmethod
    def pade(T,n):
        c=[factorial(2*n-k)*factorial(n)/(factorial(2*n)*factorial(k)*factorial(n-k)) for k in range(n+1)]
        num=[c[k]*(-T)**k for k in range(n+1)][::-1]; den=[c[k]*T**k for k in range(n+1)][::-1]
        return num,den
out={}
# ============ A. MOBILE BASE ============
M=P['M_base']+P['m']+P['mc']
# CoM of the whole robot; aerial module lumped at joint (+hc vertical)
xG=(P['M_base']*P['xG_base']+(P['m']+P['mc'])*P['lJ'])/M
zJ=P['hJ0']
hG_rigid=(P['M_base']*P['hG_base']+P['mc']*zJ+P['m']*(zJ+P['hc']))/M
hG_lev=(P['M_base']*P['hG_base']+(P['m']+P['mc'])*zJ)/M   # drone kept level: its CoM stays vertical above J
A=dict(M=M,xG=xG,hG_rigid=hG_rigid,hG_level=hG_lev,
       beta_tip_rigid=np.degrees(np.arctan(xG/hG_rigid)),beta_tip_level=np.degrees(np.arctan(xG/hG_lev)),
       beta_lat=np.degrees(np.arctan(P['b']/2/hG_rigid)))
def v_at(u,beta):
    tau=P['r']/2*M*g*(np.sin(beta)+P['Crr']*np.cos(beta))
    w=P['w0']*(u-tau/P['tau_s'])
    return P['r']*w, tau
for deg in (0,10,20):
    v,tau=v_at(0.15,np.radians(deg)); A[f'v15_{deg}']=v; A[f'tau_wheel_{deg}']=tau
    A[f'stall_ratio_{deg}']=tau/P['tau_s']
# traction (rear driven axle, climbing forward)
for deg in (10,20):
    be=np.radians(deg)
    Nr=M*g*(np.cos(be)*(P['Lwb']-xG)/P['Lwb']+np.sin(be)*hG_rigid/P['Lwb'])
    A[f'mu_req_{deg}']=M*g*(np.sin(be)+P['Crr']*np.cos(be))/Nr
    A[f'amax_{deg}']=g*(np.cos(be)*xG/hG_rigid-np.sin(be))
A['amax_0']=g*xG/hG_rigid
A['a0_step15']=2*P['tau_s']*0.15/(M*P['r'])   # initial accel of a 15% duty step (no rotor inertia)
A['vmax']=P['r']*P['w0']
# rotation scenarios
for name,R in (('mid',P['lJ']),('wheel',np.hypot(P['b']/2,P['lJ']))):
    for lab,vw in (('low',0.03),('med',0.06),('high',0.10)):
        om=(2*vw/P['b']) if name=='mid' else vw/P['b']
        a=om**2*R
        A[f'rot_{name}_{lab}']=dict(omega=om,R=R,a_c=a,tilt_eq_deg=np.degrees(np.arctan(a/g)),centrifugal_ball=5/7*om**2)
out['base']=A
# ============ B. AERIAL MODULE ============
m,mc,J,hc=P['m'],P['mc'],P['J'],P['hc']
B={}
B['p_unst_hc']={str(h):np.sqrt(m*g*h/J[0]) for h in (0.10,0.14,0.20)}
B['grav_stiff']={str(h):m*g*h for h in (0.10,0.14)}
for kT in (1.0,1.37,1.52):
    B[f'Kctrl_kT{kT}']=kT*P['kp_w']*P['kp_th']
B['tau_offset']=m*g*P['dx']
# authority (steady torque with saturated rate request + saturated rate integral), firmware units
def tau_avail(wlim,Ilim,kT=1.0): return kT*(P['kp_w']*wlim+P['ki_w']*Ilim)
for lab,wl,Il in (('bench_v4_v7',P['wlim_bench'],P['Ilim_bench']),('flight_v8_v9',P['wlim_fl'],P['Ilim_fl'])):
    for kT in (1.0,1.37):
        ta=tau_avail(wl,Il,kT)
        th_star=np.degrees(np.arcsin(np.clip((ta/(m*g)-P['dx'])/hc,-1,1))) if ta/(m*g)>P['dx'] else 0.0
        B[f'auth_{lab}_kT{kT}']=dict(tau=ta,theta_star_deg=th_star)
# allocation authority vs collective (pure pitch), firmware force units -> physical with kT
def Ffw(u): return P['kF_fw']*np.clip((u-P['u0'])/P['uspan'],0,None)**P['n_th']
for ucol in (1300,1650,1750,1816,1900):
    B[f'tau_alloc_{ucol}']=4*P['Lr']*(Ffw(2000)-Ffw(ucol))
# thrust identification
for u in (1725,1750,1775):
    B[f'kF_at_{u}']=(m+mc)*g/(8*((u-P['u0'])/P['uspan'])**P['n_th'])
B['u_hover_fw']=P['u0']+P['uspan']*((m+mc)*g/8/P['kF_fw'])**(1/P['n_th'])
# momentum theory
A_disk=np.pi*(P['Dprop']/2)**2; T=(m+mc)*g/8
vh=np.sqrt(T/(2*P['rho_air']*A_disk)); Pid=T*vh
B.update(T_hover=T,v_induced=vh,P_ideal_motor=Pid,P_elec_total_FM05_eta08=8*Pid/0.5/0.8,I_total_FM05_eta08=8*Pid/0.5/0.8/11.1,
         tip_clearance=2*P['Lr']*np.sin(np.pi/8)-P['Dprop'])
# linear closed loop, pitch axis (firmware structure, rate/integral limits inactive)
def cl_poles(kT,h,J_=J[1],ki_th=P['ki_th_fl'],tau_m=P['tau_m'],T=P['T_esc'],pade=2):
    s=np.poly1d([1,0])
    Pd_num,Pd_den=signal.pade(T,pade)
    Dn=np.poly1d(Pd_num); Dd=np.poly1d(Pd_den)
    # L = kT*Pade/(tau_m s+1) * (kp_w + ki_w/s) * (kp_th + ki_th/s + kd s/(tf s+1) + s) / (J s^2 - m g h)
    tf=P['tf_d']
    Cw_num=np.poly1d([P['kp_w'],P['ki_w']]); Cw_den=np.poly1d([1,0])
    Q_num=np.poly1d([P['kp_th'],ki_th])*np.poly1d([tf,1])+np.poly1d([P['kd_th'],0,0])+np.poly1d([1,0,0])*np.poly1d([tf,1])
    Q_den=np.poly1d([1,0])*np.poly1d([tf,1])
    Ln=np.poly1d([float(kT)])*Dn*Cw_num*Q_num
    Ld=Dd*np.poly1d([tau_m,1])*Cw_den*Q_den*np.poly1d([J_,0,-m*g*h])
    return np.roots((Ld+Ln).coeffs), (Ln,Ld)
poles={}
for kT in (1.0,1.37):
    for h in (0.0,0.10,0.14,0.20):
        p,_=cl_poles(kT,h)
        p=p[np.argsort(np.abs(p))]
        poles[f'kT{kT}_hc{h}']=[(float(z.real),float(z.imag)) for z in p]
B['poles']=poles
# margins of the open loop at nominal (kT=1.37, h=0.14)
def margins(Ln,Ld,w=np.logspace(-2,3,20000)):
    s=1j*w; L=Ln(s)/Ld(s)
    mag=np.abs(L); ph=np.unwrap(np.angle(L))
    i=np.where(np.diff(np.sign(mag-1)))[0]
    res=[]
    for k in i: res.append((w[k],np.degrees(ph[k])))
    return res
for kT in (1.0,1.37):
    _,(Ln,Ld)=cl_poles(kT,0.14)
    B[f'crossings_kT{kT}']=margins(Ln,Ld)
# inner rate loop alone
for kT in (1.0,1.37):
    wc=kT*P['kp_w']/J[1]
    ph=-90-np.degrees(np.arctan(wc*P['tau_m']))-np.degrees(wc*P['T_esc'])+np.degrees(np.arctan(wc/(P['ki_w']/P['kp_w'])))-90
    B[f'rate_loop_kT{kT}']=dict(wc=wc,PM=180+ph)
out['drone']=B
# ============ C. BALL & PLATE ============
C={}
C['Kb']=P['Kb']; C['kalpha_deg_per_us']=np.degrees(P['kalpha']); C['plate_range_deg']=np.degrees(P['kalpha'])*P['u_lim']
C['z_nmp_servo']=np.sqrt(g/P['kappa']); C['z_nmp_drone']=np.sqrt(g/(P['hP']+P['kappa']))
# four-bar (Freudenstein-type numerical solution)
def plate_angle(gam):
    S=np.array(P['S']); G=np.array(P['Gp'])
    A_=S+P['r_arm']*np.array([np.cos(gam),np.sin(gam)])
    # B on circle radius R about G, |B-A|=l, choose solution near neutral (B0=(R,0)+G)
    def f(al): 
        Bp=G+P['R_horn']*np.array([np.cos(al),np.sin(al)]); return np.linalg.norm(Bp-A_)-P['l_rod']
    return optimize.brentq(f,-1.2,1.2)
gams=np.radians(np.linspace(-60,60,121)); als=np.array([plate_angle(x) for x in gams])
C['linkage_gain_center']=float(np.gradient(als,gams)[60]); C['linkage_range_deg']=(float(np.degrees(als.min())),float(np.degrees(als.max())))
C['linkage_curve']=(np.degrees(gams).tolist(),np.degrees(als).tolist())
# ball loop
def ball_L(w,kp,kd,ki,T,Kb=P['Kb'],tau_s=0.03,Tv=P['Tv'],kap=P['kappa']):
    s=1j*w
    Gp=Kb*(1-kap/g*s**2)/(s**2*(tau_s*s+1))*np.exp(-s*T)
    Cc=kp+ki/s+kd*s*np.exp(-s*Tv)
    return Gp*Cc
def pm_gm(kp,kd,ki,T,**kw):
    w=np.logspace(-2,2.5,40000); L=ball_L(w,kp,kd,ki,T,**kw)
    mag=np.abs(L); ph=np.unwrap(np.angle(L))
    i=np.where(np.diff(np.sign(mag-1)))[0]
    pm=[(w[k],180+np.degrees(ph[k])) for k in i]
    j=np.where(np.diff(np.sign(np.degrees(ph)+180)))[0]
    gm=[(w[k],-20*np.log10(mag[k])) for k in j]
    return pm,gm
for T in (0.06,0.08,0.10,0.12,0.16):
    C[f'margins_T{T}']=pm_gm(P['kp_b'],P['kd_b'],P['ki_b'],T)
    C[f'margins_kd6_T{T}']=pm_gm(P['kp_b'],6/3,P['ki_b'],T)
# critical kd (firmware units) vs T: largest kd keeping PM>0
def stable(kp,kd,ki,T,**kw):
    # Nyquist-free check: use closed-loop poles with Pade(4)
    n,dn=signal.pade(T,4); nv,dv=signal.pade(P['Tv'],3)
    tau_s=kw.get('tau_s',0.03); Kb=kw.get('Kb',P['Kb']); kap=P['kappa']
    s=np.poly1d([1,0]); one=np.poly1d([1])
    Gn=np.poly1d(float(Kb)*np.array([-kap/g,0,1]))*np.poly1d(n); Gd=np.poly1d([1,0,0])*np.poly1d([tau_s,1])*np.poly1d(dn)
    Cn=(np.poly1d([kp,ki])*np.poly1d(dv)+np.poly1d([float(kd),0,0])*np.poly1d(nv)); Cd=np.poly1d([1,0])*np.poly1d(dv)
    r=np.roots((Gd*Cd+Gn*Cn).coeffs)
    return np.max(r.real)<0, r
crit={}
for Kb in (P['Kb'],4.9,2.5):
  for T in (0.04,0.06,0.08,0.10,0.12,0.16):
    kds=np.linspace(0.3,12,236); ok=[stable(P['kp_b'],k/3,P['ki_b'],T,Kb=Kb)[0] for k in kds]
    ok=np.array(ok)
    crit[f'Kb{Kb:.1f}_T{T}']=float(kds[ok].max()) if ok.any() else None
C['kd_crit_fw_units']=crit
# slow mode of the nominal loop
for T in (0.08,0.10):
    ok,r=stable(P['kp_b'],P['kd_b'],P['ki_b'],T); r=r[np.argsort(np.abs(r))]
    C[f'poles_T{T}']=[(float(z.real),float(z.imag)) for z in r[:6]]
out['ball']=C
json.dump(out,open('analysis.json','w'),indent=1,default=float)
import pprint; pprint.pprint({k:v for k,v in out['base'].items()},width=150)
pprint.pprint({k:v for k,v in out['drone'].items() if k!='poles'},width=150)
for k,v in out['drone']['poles'].items(): print(k,[ (round(a,2),round(b,2)) for a,b in v[:5]])
pprint.pprint({k:v for k,v in out['ball'].items() if k!='linkage_curve'},width=150)
