"""Ensayos reproducibles del controlador de referencia, sin hardware."""
from pathlib import Path
import json
import csv
import numpy as np
import mujoco
from control_simulacion import ControlReferencia,rotation_vector

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'control_referencia'
CASES=[
    {'name':'nominal','duration_s':60},
    {'name':'inclinacion_inicial','attitude_deg':[5,-3,0],'initial_height_error_m':-.01},
    {'name':'guinada_inicial','attitude_deg':[0,0,5]},
    {'name':'impulso','impulse':True},
    {'name':'escalon_altura','step':True},
    {'name':'masa_mas_15','mass_scale':1.15},
    {'name':'masa_menos_15','mass_scale':.85},
    {'name':'inercia_mas_30','inertia_scale':1.3,'attitude_deg':[5,-3,0]},
    {'name':'centro_masa_desplazado','com_offset_m':[.01,-.005,0]},
    {'name':'motor_1_menos_15','weak_motor':True},
    {'name':'retardo_y_ruido','lag_s':.03,'noise':True,'attitude_deg':[5,-3,0]},
    {'name':'saturacion_y_recuperacion','attitude_deg':[10,-6,0],'initial_height_error_m':-.015},
]


def quaternion(vector):
    angle=np.linalg.norm(vector)
    return np.r_[np.cos(angle/2),vector*np.sin(angle/2)/angle] if angle else np.array([1.,0,0,0])


def run(case):
    model=mujoco.MjModel.from_xml_path(str(ROOT/'aerotray_conjunto_cad.xml'))
    controller=ControlReferencia(model)
    # El controlador conserva parámetros nominales aunque se modifique planta.
    factor=case.get('mass_scale',1.)
    model.body_mass[1:]*=factor
    model.body_inertia[1:]*=factor*case.get('inertia_scale',1.)
    model.body_ipos[controller.body]+=case.get('com_offset_m',[0,0,0])
    data=mujoco.MjData(model);mujoco.mj_setConst(model,data)
    data.qpos[0]=controller.target+case.get('initial_height_error_m',0)
    data.qpos[1:5]=quaternion(np.radians(case.get('attitude_deg',[0,0,0])))
    mujoco.mj_forward(model,data)
    forces=controller.inverse@np.array([controller.mass*controller.gravity,0,0,0])
    gain=np.ones(8)
    if case.get('weak_motor'):gain[0]=.85
    dt=float(model.opt.timestep);rng=np.random.default_rng(20261030)
    log=[];limited=0;force_min=99.;force_max=0.;low=99.;high=-99.;max_tilt=0.
    duration=case.get('duration_s',12)
    for step in range(round(duration/dt)):
        if case.get('step') and step==round(2/dt):controller.set_target(controller.target+.01)
        noise=None
        if case.get('noise'):
            noise={'height':rng.normal(0,.0002),'speed':rng.normal(0,.005),
                   'attitude':rng.normal(0,np.radians(.05),3),'omega':rng.normal(0,.01,3)}
        request,info=controller.update(data,dt,noise)
        limited+=int(info['limited'])
        alpha=1-np.exp(-dt/case['lag_s']) if case.get('lag_s') else 1.
        forces+=alpha*(request-forces)
        data.ctrl[:]=forces*gain
        data.xfrc_applied[:]=0
        if case.get('impulse') and 2<=data.time<2.15:
            data.xfrc_applied[controller.body,3:6]=[.6,-.4,.015]
        mujoco.mj_step(model,data)
        assert np.isfinite(data.qpos).all()
        q=data.qpos[1:5]
        tilt=float(np.degrees(np.arccos(np.clip(1-2*(q[1]**2+q[2]**2),-1,1))))
        rot=np.degrees(rotation_vector(q))
        force_min=min(force_min,float(request.min()));force_max=max(force_max,float(request.max()))
        low=min(low,float(data.qpos[0]));high=max(high,float(data.qpos[0]));max_tilt=max(max_tilt,tilt)
        if step%5==0:
            log.append([float(data.time),float(data.qpos[0]),controller.target,tilt,float(rot[2]),
                        float(request.min()),float(request.max()),int(info['limited'])])
    tail=np.array([row for row in log if row[0]>=duration-2])
    z_rms=float(np.sqrt(np.mean((tail[:,1]-tail[:,2])**2)))
    tilt_rms=float(np.sqrt(np.mean(tail[:,3]**2)))
    yaw_rms=float(np.sqrt(np.mean(tail[:,4]**2)))
    passed=(z_rms<.002 and tilt_rms<.5 and yaw_rms<1 and max_tilt<15 and
            low>model.jnt_range[0,0]-.0003 and high<model.jnt_range[0,1]+.0003 and
            force_min>=-1e-9 and force_max<=6+1e-9)
    with (OUT/f'{case["name"]}.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['tiempo_s','altura_m','referencia_m','inclinacion_deg',
            'guinada_deg','fuerza_min_N','fuerza_max_N','asignacion_saturada']);writer.writerows(log)
    return {'caso':case,'duracion_s':duration,'pasa':bool(passed),'error_altura_rms_ultimos_2s_mm':z_rms*1000,
            'inclinacion_rms_ultimos_2s_deg':tilt_rms,'guinada_rms_ultimos_2s_deg':yaw_rms,
            'inclinacion_maxima_deg':max_tilt,'altura_min_m':low,'altura_max_m':high,
            'fuerza_solicitada_min_N':force_min,'fuerza_solicitada_max_N':force_max,
            'fraccion_pasos_con_limites':limited/round(duration/dt)}


def main():
    OUT.mkdir(exist_ok=True)
    results=[]
    for case in CASES:
        result=run(case);results.append(result);print(json.dumps(result,ensure_ascii=False),flush=True)
    model=mujoco.MjModel.from_xml_path(str(ROOT/'aerotray_conjunto_cad.xml'))
    report={'estado':'completado' if all(x['pasa'] for x in results) else 'ensayos_con_fallos',
        'mujoco':mujoco.__version__,'casos':results,
        'controlador':ControlReferencia(model).configuration(),
        'limite':'Variaciones sintéticas de un modelo provisional; no certifica robustez del prototipo ni de firmware.'}
    (OUT/'resultados.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    assert all(x['pasa'] for x in results),'Revisar ensayos que fallaron'


if __name__=='__main__':main()
