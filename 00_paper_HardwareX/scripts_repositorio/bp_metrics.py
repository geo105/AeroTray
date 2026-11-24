"""Ball-and-plate metrics from the Raspberry Pi camera logs (registro_*.csv).
Error = Euclidean distance of the ball to the plate centre (reference 0,0), only while the PID is active
and the ball is detected (estado == 3). Usage: python3 bp_metrics.py <dir with registro_*.csv>"""
import csv, glob, math, os, sys, json, time, numpy as np
RAIZ=os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","..")
D=sys.argv[1] if len(sys.argv)>1 else os.path.join(RAIZ,"08_datos_y_registros","copia_raspberry_pi_20261120","pi","registros")
def load(f):
    T,X,Y,E,PX,PY=[],[],[],[],[],[]
    with open(f) as fh:
        r=csv.reader(fh); next(r,None)
        for row in r:
            try: t=float(row[0]); e=int(float(row[6]))
            except: continue
            x=float(row[1]) if row[1] not in ("","nan") else np.nan
            y=float(row[2]) if row[2] not in ("","nan") else np.nan
            T.append(t);X.append(x);Y.append(y);E.append(e)
            try: PX.append(int(row[4]));PY.append(int(row[5]))
            except: PX.append(0);PY.append(0)
    return np.array(T),np.array(X),np.array(Y),np.array(E),np.array(PX),np.array(PY)
def stats(t,x,y,e,t0,t1):
    m=(t>=t0)&(t<=t1)
    act=m&((e==1)|(e==3)); vis=m&(e==3)&~np.isnan(x)
    if act.sum()<50 or vis.sum()<50: return None
    d=np.hypot(x[vis],y[vis])
    return dict(t0=time.strftime("%H:%M:%S",time.localtime(t0)),dur_s=round(float(t[act][-1]-t[act][0]),1),
        seen_pct=round(100*vis.sum()/act.sum(),1),mean_mm=round(float(d.mean()),1),median_mm=round(float(np.median(d)),1),
        p90_mm=round(float(np.percentile(d,90)),1),max_mm=round(float(d.max()),1),rms_x=round(float(np.sqrt(np.mean(x[vis]**2))),1),
        rms_y=round(float(np.sqrt(np.mean(y[vis]**2))),1),fs_hz=round(float(vis.sum()/max(1e-9,t[vis][-1]-t[vis][0])),0))
if __name__=="__main__":
    out={}
    # 1) every PID segment on 18/11 (gaps > 1 s split segments)
    segs=[]
    for f in sorted(glob.glob(os.path.join(D,"registro_20261118_*.csv"))):
        t,x,y,e,px,py=load(f)
        act=(e==1)|(e==3)
        if act.sum()==0: continue
        ta=t[act]; br=np.where(np.diff(ta)>1.0)[0]
        starts=np.r_[ta[0],ta[br+1]]; ends=np.r_[ta[br],ta[-1]]
        for a,b in zip(starts,ends):
            if b-a<20: continue
            s=stats(t,x,y,e,a,b)
            if s: s["file"]=os.path.basename(f); segs.append(s)
    out["segments_20261118"]=segs
    # 2) per video window (video start from file name, duration from the video)
    vids=json.load(open(sys.argv[2] if len(sys.argv)>2 else os.path.join(RAIZ,"00_paper_HardwareX","data","videos_index.json")))
    per=[]
    for v in vids:
        if not v["f"].startswith("VID_20261118"): continue
        tv=time.mktime(time.strptime(v["f"][4:19],"%Y%m%d_%H%M%S"))
        for f in sorted(glob.glob(os.path.join(D,"registro_20261118_*.csv"))):
            t,x,y,e,px,py=load(f)
            if len(t)==0 or t[0]>tv+v["dur"] or t[-1]<tv: continue
            s=stats(t,x,y,e,tv,tv+v["dur"])
            if s: s["video"]=v["f"]; per.append(s)
    out["videos_20261118"]=per
    os.makedirs(os.path.join(RAIZ,"00_paper_HardwareX","datos_regenerados"),exist_ok=True)
    json.dump(out,open(os.path.join(RAIZ,"00_paper_HardwareX","datos_regenerados","bp_metrics.json"),"w"),indent=1)
    for s in segs: print("SEG",s)
    for s in per: print("VID",s)
