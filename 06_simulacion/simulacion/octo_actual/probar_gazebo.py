"""Ejecuta física real de Gazebo sin interfaz, con pasos deterministas."""
import argparse
import json
import csv
import os
import subprocess
import xml.etree.ElementTree as E
import numpy as np
from control import ROOT, CFG, model_force


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--modo',choices=['banco','altura','altura_ideal'],default='banco')
    p.add_argument('--segundos',type=float,default=12)
    p.add_argument('--gain',type=float,default=1)
    p.add_argument('--seed',type=int,default=73021)
    p.add_argument('--sin-control',action='store_true')
    p.add_argument('--fallo-tof',action='store_true')
    p.add_argument('--salida',default=None)
    p.add_argument('--ball-gains',nargs=3,type=float,metavar=('KP','KI','KD'))
    p.add_argument('--ball-abierto',action='store_true')
    a=p.parse_args()
    stem=a.salida or a.modo+('_abierto' if a.sin_control else '')+('_tof_fallo' if a.fallo_tof else '')
    os.environ.update(DSM_SIM_MODO=a.modo,DSM_SIM_GAIN=str(a.gain),DSM_SIM_SEED=str(a.seed),
        DSM_SIM_SIN_CONTROL=str(int(a.sin_control)),DSM_SIM_TOF_FALLO=str(int(a.fallo_tof)),DSM_SIM_CSV=stem+'.csv')
    os.environ['DSM_SIM_BP_ABIERTO']=str(int(a.ball_abierto))
    if a.ball_gains: os.environ['DSM_SIM_BP_GAINS']=json.dumps(a.ball_gains)
    # PythonSystemLoader embebe su propio intérprete; usar el servidor externo
    # evita cargarlo de nuevo dentro del intérprete de TestFixture.
    env=os.environ.copy(); env['GZ_PARTITION']='dsm_prueba_'+stem+'_'+str(os.getpid())
    tree=E.parse(ROOT/f'{a.modo}.sdf'); tree.find('./world/physics/real_time_factor').text='0'
    world_path=ROOT/'resultados'/f'{stem}_prueba.sdf'; tree.write(world_path)
    with (ROOT/'resultados'/f'{stem}_gazebo.log').open('w') as log:
        proc=subprocess.run(['gz','sim','-s','-r','--iterations',str(round(a.segundos/.001)),
            str(world_path)],env=env,stdout=log,stderr=subprocess.STDOUT,
            timeout=max(45,a.segundos*5))
    success=proc.returncode==0
    data=np.genfromtxt(ROOT/'resultados'/f'{stem}.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
    tilt=np.hypot(data['roll_deg'],data['pitch_deg']); tail=data['t_s']>a.segundos-2
    with (ROOT/'resultados'/f'{stem}.csv').open() as f:
        cut=sorted({x['corte'] for x in csv.DictReader(f) if x['corte']})
    result={'motor_fisico':'Gazebo Harmonic 8 / DART','pasos_ejecutados':bool(success),
        'modo':a.modo,'control_activo':not a.sin_control,'ganancia_angulo_factor':a.gain,'seed':a.seed,
        'duracion_s':float(data['t_s'][-1]),'muestras':len(data),'inclinacion_max_deg':float(tilt.max()),
        'rms_tilt_ultimos_2s_deg':float(np.sqrt(np.mean(tilt[tail]**2))),
        'altura_final_m':float(data['z_m'][-1]),'altura_objetivo_m':CFG['altura']['objetivo_m'],
        'cortes':cut,'escala_RP_min':float(data['escala_RP'].min()),
        'ball_plate_pid':a.ball_gains or [1.875,.5,1.5625],
        'ball_plate_activo':not a.ball_abierto,
        'bola_fuera_placa':bool(data['bola_perdida'].max()),
        'rms_bola_mm':float(np.sqrt(np.mean(data['bola_x_m']**2+data['bola_y_m']**2))*1000),
        'rms_bola_ultimos_2s_mm':float(np.sqrt(np.mean(data['bola_x_m'][tail]**2+data['bola_y_m'][tail]**2))*1000),
        'empuje_max_1500us_N':float(8*model_force(1500)),
        'peso_movil_modelado_N':(CFG['masa_rotatoria_kg']+CFG['masa_carro_vertical_kg'])*9.81,
        'limite':'Modelo provisional; no es validacion del prototipo ni identificacion desde el TXT.'}
    (ROOT/'resultados'/f'{stem}.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    assert success and len(data)>a.segundos*40 and np.isfinite(tilt).all(), 'Gazebo no produjo una corrida válida'
    if a.modo=='banco' and not a.sin_control:
        assert not cut and tilt.max()<12, 'El banco no se mantuvo dentro del límite angular'
    if a.fallo_tof: assert 'TOF_INVALIDO' in cut


if __name__=='__main__': main()
