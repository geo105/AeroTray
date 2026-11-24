"""PID del sistema Pi + PS3 Eye y planta reducida de rodadura.

Se conserva la ley del firmware, incluidas integral condicional, derivada
externa, saturación ANTES de holgura y truncamiento entero. La planta no está
identificada: no equivale a una prueba de la pelota real ni simula imágenes.
"""
from collections import deque
import json
import numpy as np
from control import CFG, ROOT, G

AJUSTES = json.loads((ROOT/CFG['ball_plate']['ajustes']).read_text())


class WorkingPID:
    """Vectorización de EjePID::paso y controlar(), sin acceso a los servos."""
    def __init__(self, count=1, gains=None, ajustes=None):
        self.a=dict(AJUSTES if ajustes is None else ajustes)
        a=self.a
        self.gains=np.broadcast_to(np.array(gains if gains is not None else
            [a['kp'],a['ki'],a['kd']],dtype=float),(count,3)).copy()
        self.integral=np.zeros((count,2)); self.velocity=np.zeros((count,2))
        self.previous=np.zeros((count,2)); self.error_previous=np.zeros((count,2))
        self.estimate=np.zeros((count,2)); self.ready=False
        self.pulse=np.broadcast_to([a['neutro_x'],a['neutro_y']],(count,2)).copy()
        self.saturated=np.zeros((count,2),dtype=bool)

    def step(self, position_units, dt, external_velocity=None):
        a=self.a; pos=np.array(position_units,dtype=float,copy=True)
        if not self.ready:
            self.previous=pos.copy(); self.estimate=pos.copy(); self.velocity[:]=0; self.ready=True
        if external_velocity is not None:
            self.velocity=np.array(external_velocity,copy=True)
        elif a['ab_alfa']>0:
            prediction=self.estimate+self.velocity*dt; innovation=pos-prediction
            self.estimate=prediction+a['ab_alfa']*innovation
            self.velocity+=a['ab_beta']/dt*innovation; pos=self.estimate.copy()
        else:
            self.velocity=a['filtro_v']*self.velocity+(1-a['filtro_v'])*(pos-self.previous)/dt
        self.previous=pos.copy()
        error=np.array([a['consigna_x'],a['consigna_y']])-pos
        if a['reinicio_integral']: self.integral[error*self.error_previous<0]=0
        self.error_previous=error.copy()
        allowed=(a['vel_integral']<=0)|(np.abs(self.velocity)<a['vel_integral'])
        self.integral+=np.where(allowed,error*dt,0.)
        kp,ki,kd=self.gains.T
        max_i=np.divide(a['autoridad_i']*a['limite'],ki,out=np.zeros_like(ki),where=ki>0)
        self.integral=np.clip(self.integral,-max_i[:,None],max_i[:,None])
        u=kp[:,None]*error+ki[:,None]*self.integral-kd[:,None]*self.velocity
        if a['friccion']>0:
            u+=np.where((np.abs(error)>1.5)&(np.abs(self.velocity)<8),np.sign(error)*a['friccion'],0)
        u*=np.array([a['factor_x'],a['factor_y']])
        self.saturated=np.abs(u)>=a['limite']; u=np.clip(u,-a['limite'],a['limite'])
        d=u*np.array([a['signo_x'],a['signo_y']]); xp,xn,yp,yn=a['holgura']
        gap=np.where(d>=0,[xp,yp],[xn,yn])
        d+=np.sign(d)*gap*np.minimum(1,np.abs(d)/a['holgura_rampa'])
        self.pulse=np.array([a['neutro_x'],a['neutro_y']])+np.trunc(d).astype(int)
        return self.pulse


class BallPlate:
    def __init__(self, count=1, gains=None, seed=7331, cfg=CFG, initial=None):
        self.cfg=cfg; self.b=cfg['ball_plate']; self.n=count; self.rng=np.random.default_rng(seed)
        self.pid=WorkingPID(count,gains); self.position=np.zeros((count,2))
        if initial is None: self.position[:]=[.030,-.020]
        else: self.position[:]=initial
        self.velocity=np.zeros((count,2)); self.angle=np.zeros((count,2)); self.old_angle=self.angle.copy()
        self.lost=np.zeros(count,dtype=bool); self.history=deque(); self.camera_history=deque(maxlen=20)
        self.last_camera=-self.b['camara_periodo_s']; self.saturation_count=np.zeros(count)
        self.frames=0; self.pulse=self.pid.pulse.copy()
        self.last_seen=np.zeros(count); self.feedback=True
        self.noise_cases=count; self.extra_pulse=np.zeros((count,2))

    def step(self, t, dt, base_rpy, base_omega=None, base_alpha=None, base_accel=None, missing=False):
        b=self.b; a=self.pid.a; rpy=np.broadcast_to(base_rpy,(self.n,3))
        self.history.append((t,self.position.copy()))
        while len(self.history)>1 and self.history[1][0]<=t-b['camara_retardo_s']:
            self.history.popleft()
        if t-self.last_camera>=b['camara_periodo_s']-1e-8:
            camera_dt=b['camara_periodo_s'] if self.frames==0 else t-self.last_camera
            self.last_camera=t; self.frames+=1
            seen=~self.lost & ~np.broadcast_to(missing,(self.n,))
            noise=self.rng.normal(0,b['camara_ruido_m'],(self.noise_cases,2))
            measured=self.history[0][1]+np.tile(noise,(self.n//self.noise_cases,1))
            units=measured/(b.get('mm_por_unidad',3.)/1000)
            self.camera_history.append((t,units.copy()))
            points=[p for p in self.camera_history if p[0]>=t-b['vel_ventana_s']]
            speed=None
            if len(points)>=4:
                times=np.array([p[0] for p in points]); times-=times.mean()
                values=np.stack([p[1] for p in points])
                speed=np.sum(times[:,None,None]*values,axis=0)/np.sum(times**2)
            old=self.pulse.copy(); old_integral=self.pid.integral.copy()
            self.pulse=self.pid.step(units,float(np.clip(camera_dt,.002,.1)),speed)
            self.pulse[~seen]=old[~seen]; self.pid.integral[~seen]=old_integral[~seen]
            self.last_seen[seen]=t
            expired=t-self.last_seen>.5
            self.pulse[expired]=[a['neutro_x'],a['neutro_y']]
            self.pid.integral[expired]=0; self.pid.velocity[expired]=0
            if not self.feedback: self.pulse[:]=[a['neutro_x'],a['neutro_y']]
            self.saturation_count+=self.pid.saturated.any(axis=1)
        # Holgura de la planta supuesta: el pulso que no supera el juego no inclina.
        command=self.pulse-np.array([a['neutro_x'],a['neutro_y']])+self.extra_pulse
        xp,xn,yp,yn=a['holgura']; gap=np.where(command>=0,[xp,yp],[xn,yn])
        effective=np.sign(command)*np.maximum(0,np.abs(command)-gap)
        target=effective*np.array([a['signo_x'],a['signo_y']])*b['ganancia_rad_us']
        self.old_angle=self.angle.copy()
        self.angle+=(1-np.exp(-dt/b['servo_tau_s']))*(target-self.angle)
        # Gravedad en marco del dron y luego marco de la placa.
        r,p=rpy[:,0],rpy[:,1]
        gravity=np.stack([G*np.sin(p),-G*np.sin(r)*np.cos(p),-G*np.cos(r)*np.cos(p)],axis=1)
        if base_accel is not None: gravity-=np.broadcast_to(base_accel,(self.n,3))
        if base_omega is not None and base_alpha is not None:
            pos=np.c_[self.position,np.full(self.n,b['altura_placa_m'])]
            w=np.broadcast_to(base_omega,(self.n,3)); alpha=np.broadcast_to(base_alpha,(self.n,3))
            gravity-=np.cross(alpha,pos)+np.cross(w,np.cross(w,pos))+2*np.cross(w,np.c_[self.velocity,np.zeros(self.n)])
        sr,cr=np.sin(-self.angle[:,1]),np.cos(-self.angle[:,1]); sp,cp=np.sin(self.angle[:,0]),np.cos(self.angle[:,0])
        gx=cp*gravity[:,0]-sp*gravity[:,2]
        gy=sp*sr*gravity[:,0]+cr*gravity[:,1]+cp*sr*gravity[:,2]
        acceleration=5/7*np.stack([gx,gy],axis=1)-b['amortiguamiento_bola_s']*self.velocity
        active=~self.lost
        self.velocity[active]+=dt*acceleration[active]
        self.position[active]+=dt*self.velocity[active]
        half=np.array(b['placa_m'])/2-b['diametro_bola_m']/2
        self.lost|=(np.abs(self.position)>half).any(axis=1)
        # La rodadura deja de ser válida fuera de la superficie. Conservar el
        # primer estado de salida y señalar fallo; no simular rebote/caída.
        self.velocity[self.lost]=0.
        return self.position

    def reaction_delta(self):
        """Cambio de momento por bola desplazada respecto al centro (masas ya incluidas)."""
        mass=self.b['masa_bola_kg']
        torque=np.stack([-mass*G*self.position[:,1],mass*G*self.position[:,0],np.zeros(self.n)],axis=1)
        torque[self.lost]=0.
        return torque
