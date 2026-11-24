import numpy as np, json
import scipy.signal as ss
from params import P, g
h=1/P['fps']
def lsw(W=0.06):
    N=int(np.floor(W/h))+1
    t=-np.arange(N)*h; tb=t.mean(); w=(t-tb)/np.sum((t-tb)**2)
    return w
def char_poly(Kb,T,kp,ki,kd,tau_s=0.05,kap=P['kappa'],W=0.06):
    num=[-Kb*kap/g,0,Kb]; den=[tau_s,1,0,0]
    nd,dd,_=ss.cont2discrete((num,den),h,method='zoh')
    Gn=np.poly1d(np.atleast_1d(np.squeeze(nd))); Gd=np.poly1d(dd)
    n=int(round(T/h))
    w=lsw(W); N=len(w)
    # K(z) = kp + ki*h*z/(z-1) + kd*sum w_k z^-k  -> common denominator (z-1) z^(N-1)
    zN1=np.poly1d([1]+[0]*(N-1))
    Hv=np.poly1d(w)          # w0 z^(N-1)+w1 z^(N-2)+... = z^(N-1) * sum w_k z^-k
    Kn=np.poly1d([kp])*np.poly1d([1,-1])*zN1+np.poly1d([ki*h,0])*zN1+np.poly1d([kd])*np.poly1d([1,-1])*Hv
    Kd=np.poly1d([1,-1])*zN1
    zn=np.poly1d([1]+[0]*n)
    return Gd*zn*Kd+Gn*Kn
def stable(*a,**k):
    r=np.roots(char_poly(*a,**k).coeffs); return np.max(np.abs(r))<1, r
kp,ki,kd=P['kp_b'],P['ki_b'],P['kd_b']
out={}
for Kb in (6.0,4.9,3.5,2.5):
  for T in (0.04,0.06,0.08,0.10,0.12,0.14,0.16):
    kds=np.linspace(0.2,12,119); ok=np.array([stable(Kb,T,kp,ki,k/3)[0] for k in kds])
    out[f"{Kb}_{T}"]=float(kds[ok].max()) if ok.any() else None
    print(Kb,T,out[f"{Kb}_{T}"])
# dominant slow mode nominal
for Kb in (6.0,4.9,3.5):
    ok,r=stable(Kb,0.08,kp,ki,kd); s=np.log(r.astype(complex))/h
    s=s[np.argsort(np.abs(s))]
    print("Kb",Kb,"stable",ok,"slowest:",[(round(z.real,2),round(z.imag,2)) for z in s[:4]])
json.dump(out,open('ball_dt.json','w'),indent=1)
