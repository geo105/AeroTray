"""Busca ganancias alrededor del PID probado y evalúa en casos separados.

La selección usa una planta reducida con parámetros supuestos. Los candidatos
son archivos nuevos de simulación; jamás reescribe ajustes.json del prototipo.
"""
import copy
import itertools
import json
import numpy as np
from control import ROOT, CFG, real_rows
from ball_plate import BallPlate, AJUSTES


def evaluate(gains, validation=False, profile=0, seconds=16.):
    gains=np.atleast_2d(gains); k=len(gains); cases=8; n=k*cases
    cfg=copy.deepcopy(CFG)
    cfg['ball_plate'].update(
        camara_retardo_s=[.025,.04,.065][profile] if not validation else [.035,.055,.085][profile],
        servo_tau_s=[.045,.06,.08][profile] if not validation else [.05,.075,.095][profile],
        camara_ruido_m=[.0003,.0005,.0008][profile] if not validation else [.0004,.0007,.001][profile])
    seed=(88001 if validation else 73001)+profile
    rng=np.random.default_rng(seed)
    starts=rng.uniform([-.040,-.035],[.040,.035],(cases,2))
    starts[0]=[.03,-.02]
    plant=BallPlate(n,np.repeat(gains,cases,axis=0),seed, cfg,
                    initial=np.tile(starts,(k,1)))
    plant.noise_cases=cases
    evidence=real_rows(); clock=(evidence[:,0]-evidence[0,0])/1000
    # Misma secuencia de ruido para todos los candidatos en cada caso.
    # Se repite explícitamente para impedir ventajas por semilla/candidato.
    phase=rng.uniform(0,15,cases); scale=rng.uniform(.6,1.25,cases)
    sum_error=np.zeros(n); tail_error=np.zeros(n); tail_count=0; count=0
    traces=[]; dt=.005
    for step in range(round(seconds/dt)):
        t=step*dt; rpy=np.zeros((cases,3))
        for axis in [0,1]:
            times=(t+phase)%clock[-1]
            rpy[:,axis]=np.deg2rad(np.interp(times,clock,evidence[:,axis+1]))*scale
        # Inversión y cambios de inclinación en la reserva, distintos a búsqueda.
        if validation:
            rpy[::2,:2]*=-1
            rpy[:,0]+=np.deg2rad(.5)*np.sin(t*1.1+phase)
            if 7<t<7.15: plant.velocity[:,0]+=.015*dt/.15
        plant.step(t,dt,np.tile(rpy,(k,1)))
        # Penalizar la salida de superficie a lo largo de todo el episodio.
        radial2=(plant.position**2).sum(1)
        sum_error+=radial2; count+=1
        if t>seconds-3: tail_error+=radial2; tail_count+=1
        if k==1 and step%10==0:
            traces.append([t,*plant.position[0],*plant.pulse[0]])
    rms=np.sqrt(sum_error/count).reshape(k,cases)*1000
    tail=np.sqrt(tail_error/tail_count).reshape(k,cases)*1000
    lost=plant.lost.reshape(k,cases)
    sat=(plant.saturation_count/max(1,plant.frames)).reshape(k,cases)
    # Pérdidas siempre dominan la puntuación. Métrica RMS radial temporal real.
    score=rms.mean(1)+.4*tail.mean(1)+1000*lost.mean(1)+15*sat.mean(1)
    return {'score':score,'rms_mm':rms.mean(1),'tail_mm':tail.mean(1),
        'lost':lost.mean(1),'saturation':sat.mean(1),'trace':traces}


def main():
    baseline=np.array([AJUSTES['kp'],AJUSTES['ki'],AJUSTES['kd']])
    candidates=np.array([baseline*np.array(f) for f in itertools.product(
        [.75,1.,1.25,1.5],[.5,1.,1.5],[.75,1.,1.25,1.5])])
    original=int(np.argmin(np.linalg.norm(candidates-baseline,axis=1)))
    results=[evaluate(candidates,profile=i) for i in range(3)]
    score=np.mean([r['score'] for r in results],axis=0)
    best=int(np.argmin(score)); selected=candidates[best]
    # No se vuelve a seleccionar según reserva: se evalúa una vez la ganadora.
    tested=np.stack([baseline,selected])
    reserve=[evaluate(tested,True,i) for i in range(3)]
    reserve_score=np.mean([r['score'] for r in reserve],axis=0)
    loss=np.mean([r['lost'] for r in reserve],axis=0)
    accepted=bool(reserve_score[1]<.95*reserve_score[0] and loss[1]<=loss[0])
    output={'referencia':'PID Pi + PS3 Eye confirmado por usuario','original':baseline.tolist(),
        'candidato':selected.tolist(),'candidatos_busqueda':len(candidates),
        'casos_busqueda':24,'casos_reserva':24,'segundos_por_caso':16,
        'semillas_busqueda':[73001,73002,73003],'semillas_reserva':[88001,88002,88003],
        'score_busqueda_original':float(score[original]),'score_busqueda_candidato':float(score[best]),
        'score_reserva_original':float(reserve_score[0]),'score_reserva_candidato':float(reserve_score[1]),
        'rms_reserva_original_mm':float(np.mean([r['rms_mm'][0] for r in reserve])),
        'rms_reserva_candidato_mm':float(np.mean([r['rms_mm'][1] for r in reserve])),
        'fraccion_perdida_original':float(loss[0]),'fraccion_perdida_candidato':float(loss[1]),
        'aceptado_solo_simulacion':accepted,
        'perfil_simulacion':(selected if accepted else baseline).tolist(),
        'limite':'Rodadura/latencias/servo supuestos, inclinaciones del TXT reproducidas. Reserva no usada para seleccionar. Validar después con Gazebo acoplado. No se modifican ganancias reales.'}
    (ROOT/'resultados/ajuste_ball_plate.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
    np.savetxt(ROOT/'resultados/busqueda_ball_plate.csv',np.c_[candidates,score],delimiter=',',
        header='kp,ki,kd,score_busqueda',comments='')
    print(json.dumps(output,ensure_ascii=False,indent=2))


if __name__=='__main__': main()
