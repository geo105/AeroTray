"""Informe y figura reproducibles de evidencia, Gazebo y evaluación de Isaac."""
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from control import ROOT, CFG, real_rows


def load(name): return json.loads((ROOT/'resultados'/name).read_text())


def main():
    real=real_rows(); reserve=load('reserva_gazebo.json')
    baseline=load('isaac_originales.json'); policy=load('isaac_residual.json')
    improved=bool(policy['supervivencia']>=baseline['supervivencia'] and
        policy['rms_bola_mm']<baseline['rms_bola_mm'] and policy['rms_tilt_deg']<=baseline['rms_tilt_deg'])
    height=load('altura_ideal.json'); original_height=load('altura_limite_original.json')
    report={'evidencia_real':{'muestras':len(real),'duracion_s':float((real[-1,0]-real[0,0])/1000),
        'roll_media_deg':float(real[:,1].mean()),'pitch_media_deg':float(real[:,2].mean()),
        'inclinacion_max_deg':float(np.hypot(real[:,1],real[:,2]).max()),
        'nota':'Ángulos de la IMU, sin referencia externa ni medidas de altura/pelota.'},
        'ball_plate':{k:v for k,v in reserve.items() if k!='casos'},
        'isaac':{'original':baseline,'residual':policy,'politica_seleccionada':improved,
            'iteraciones':100,'entornos_entrenamiento':8,'transiciones_entrenamiento':25600,
            'criterio':'No reducir supervivencia; reducir RMS pelota sin empeorar RMS inclinación.'},
        'altura':{'original':original_height,'ideal':height,
            'nota':'Ideal comienza a 15 mm con fuerza de trim y referencia en rampa; no demuestra despegue.'},
        'cambios_hardware':False,'configuracion':CFG,
        'checkpoint_sha256':hashlib.sha256((ROOT/'modelos/residual_100_iteraciones.pt').read_bytes()).hexdigest()}
    (ROOT/'resultados/resumen.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'white'})
    fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    teal='#137d8d'; orange='#d47b21'
    t=(real[:,0]-real[0,0])/1000
    axs[0,0].plot(t,real[:,1],label='Roll IMU',color=teal)
    axs[0,0].plot(t,real[:,2],label='Pitch IMU',color=orange)
    axs[0,0].set(title='Registro real: control angular en la junta',xlabel='Tiempo (s)',ylabel='Ángulo registrado (°)')
    axs[0,0].legend(frameon=False)
    for name,label,color in [('reserva_81243_base','PID original',teal),('reserva_81243_candidato','PID candidato',orange)]:
        data=np.genfromtxt(ROOT/'resultados'/f'{name}.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
        axs[0,1].plot(data['t_s'],1000*np.hypot(data['bola_x_m'],data['bola_y_m']),label=label,color=color)
    axs[0,1].set(title='Gazebo + rodadura: pelota, caso de reserva',xlabel='Tiempo (s)',ylabel='Distancia al centro (mm)')
    axs[0,1].legend(frameon=False)
    d=np.genfromtxt(ROOT/'resultados/altura_ideal.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
    axs[1,0].plot(d['t_s'],1000*d['z_m'],color=teal,label='Altura verdadera simulada')
    axs[1,0].axhline(CFG['altura']['objetivo_m']*1000,color=orange,ls='--',label='Objetivo final')
    axs[1,0].set(title='Altura con fuerzas ideales y trim inicial',xlabel='Tiempo (s)',ylabel='Desplazamiento en guía (mm)')
    axs[1,0].legend(frameon=False)
    axs[1,1].bar([0,1],[baseline['rms_bola_mm'],policy['rms_bola_mm']],color=[teal,orange],width=.55)
    axs[1,1].set(xticks=[0,1],xticklabels=['Controles originales','Con residuo PPO'],ylabel='RMS pelota (mm)',
        title='Isaac Lab: primera política sin mejora')
    for i,val in enumerate([baseline['rms_bola_mm'],policy['rms_bola_mm']]):
        axs[1,1].text(i,val+.3,f'{val:.1f}',ha='center')
    axs[1,1].set_ylim(0,max(baseline['rms_bola_mm'],policy['rms_bola_mm'])*1.2)
    for ax in axs.flat: ax.grid(axis='y',alpha=.2); ax.set_axisbelow(True)
    fig.suptitle('DSM · Control del octocóptero y ball and plate\nModelo provisional; mejoras de simulación, sin cambios en el prototipo',fontsize=15)
    fig.savefig(ROOT/'resultados/comparacion.png',dpi=160); plt.close(fig)
    print('Informe:',ROOT/'resultados/resumen.json')
    print('Mejora ball/plate Gazebo (%):',reserve['reduccion_error_porcentaje'])
    print('Seleccionar PPO:',improved)


if __name__=='__main__': main()
