"""PD de posición en cm para una primera prueba limitada con cámara.

El mapa se confirma observando la pelota: (eje afectado, signo) por GPIO41/42.
No usa ni presupone las ganancias de actitud del dron.
"""
import math


class ControlPD:
    def __init__(self, mapa, kp=.35, kd=.22, max_deg=2.0, centros=(90.0, 90.0), ki=0.0):
        if len(mapa)!=2 or sorted(m[0] for m in mapa)!=[0,1] or any(m[1] not in (-1,1) for m in mapa):
            raise ValueError('Confirma un eje distinto para cada servo y su sentido')
        if not (1.0 <= max_deg <= 18.0) or any(abs(c - 90.0) > 12.0 for c in centros) or not (0.0 <= ki <= 2.0):
            raise ValueError('Rango o centros fuera de límites seguros')
        self.mapa=mapa;self.kp=kp;self.kd=kd;self.ki=float(ki);self.max_deg=float(max_deg);self.centros=(float(centros[0]),float(centros[1]))
        self.previa=None;self.t=None;self.v=[0.,0.];self.integ=[0.,0.]

    def paso(self, xy, t):
        if len(xy)!=2 or not all(math.isfinite(x) for x in xy):raise ValueError('Posición no válida')
        if abs(xy[0])>8.55 or abs(xy[1])>6.2:raise ValueError('Pelota cerca del borde: recolócala junto al centro')
        if self.t is not None:
            dt=t-self.t
            if not .025<=dt<=.30:raise ValueError('Medición demasiado lenta para mantener el control')
            a=dt/(.14+dt)
            for k in range(2):
                velocidad=(xy[k]-self.previa[k])/dt
                if abs(velocidad)>100:raise ValueError('Salto de detección de la pelota')
                self.v[k]+=a*(velocidad-self.v[k])
                if self.ki>0 and abs(xy[k])<5.5:
                    self.integ[k]=max(-4.5,min(4.5,self.integ[k]+xy[k]*dt))
        self.previa=list(xy);self.t=t
        correccion=[-self.kp*xy[k]-self.kd*self.v[k]-self.ki*self.integ[k] for k in range(2)]
        lim=int(self.max_deg) if self.max_deg==int(self.max_deg) else self.max_deg
        c0=int(self.centros[0]) if self.centros[0]==int(self.centros[0]) else self.centros[0]
        c1=int(self.centros[1]) if self.centros[1]==int(self.centros[1]) else self.centros[1]
        cs=(c0,c1)
        return [cs[i]+max(-lim,min(lim,signo*correccion[eje])) for i,(eje,signo) in enumerate(self.mapa)]
