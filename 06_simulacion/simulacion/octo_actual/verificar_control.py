"""Equivalencia con C++ original y geometría de los sensores desplazados."""
import ctypes
import json
import subprocess
import numpy as np
from control import ROOT, CFG, FirmwareControl, Sensors, rotation, model_force
from ball_plate import WorkingPID, AJUSTES


def main():
    source=(ROOT/'datos/ball_plate_firmware_original.cpp').read_text()
    start=source.index('struct EjePID {'); end=source.index('static EjePID pid_x',start)
    pid=source[start:end]
    start=source.index('static float compensarHolgura('); end=source.index('// Nucleo 0: captura',start)
    gap=source[start:end]
    a=AJUSTES
    globals_='\n'.join(f'float {k}={float(a[k])}f;' if '.' in str(float(a[k])) else '' for k in
        ['kp','ki','kd','filtro_v','vel_integral','autoridad_i','ab_alfa','ab_beta','friccion_us'] if k in a)
    # friccion_us y limite_us son nombres de firmware; ajustes usa nombres cortos.
    globals_+=f'\nfloat friccion_us={float(a["friccion"])}f, limite_us={float(a["limite"])}f;\n'
    globals_+=f'bool reinicio_integral={str(bool(a["reinicio_integral"])).lower()};\n'
    globals_+='float ZONA_MUERTA_PX=1.5f, VELOCIDAD_QUIETA=8.f;\n'
    code=('#include <cmath>\n#include <algorithm>\n'
        'template<class T> T constrain(T x,T a,T b){return std::max(a,std::min(x,b));}\n'+globals_+pid+
        f'float holgura_rampa_us={float(a["holgura_rampa"])}f;\n'+gap+
        'EjePID e;\nextern "C" void reset(){e.reiniciar();}\n'
        'extern "C" float step(float p,float dt,float v,int ext){return e.paso(p,0,dt,ext,v);}\n'
        'extern "C" int pulse(float u,float gp,float gn){return (int)compensarHolgura(constrain(u,-limite_us,limite_us),gp,gn);}\n')
    cpp=ROOT/'resultados/pid_equivalencia.cpp'; cpp.write_text(code)
    library=ROOT/'resultados/pid_equivalencia.so'
    subprocess.run(['g++','-O2','-shared','-fPIC',str(cpp),'-o',str(library)],check=True)
    native=ctypes.CDLL(str(library)); native.step.argtypes=[ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_int]
    native.step.restype=ctypes.c_float; native.pulse.argtypes=[ctypes.c_float]*3; native.pulse.restype=ctypes.c_int
    max_error=0.; pulse_error=0; rng=np.random.default_rng(20041)
    for external in [False,True]:
        native.reset(); port=WorkingPID()
        for t in np.arange(0,8,.01):
            pos=15*np.sin(t*1.2)+rng.normal(0,.05); v=18*np.cos(t*1.2)
            expected=native.step(pos,.01,v,int(external))
            pulse=port.step(np.array([[pos,pos]]),.01,np.array([[v,v]]) if external else None)
            kp,ki,kd=port.gains[0]
            raw=kp*(-pos)+ki*port.integral[0,0]-kd*port.velocity[0,0]
            max_error=max(max_error,abs(expected-raw))
            expected_p=a['neutro_x']+native.pulse(expected,a['holgura'][0],a['holgura'][1])
            pulse_error=max(pulse_error,abs(int(pulse[0,0])-expected_p))
    assert max_error<.002 and pulse_error<=1, (max_error,pulse_error)
    # ToF debe recuperar altura del pivote aunque esté sobre M1 y haya inclinación.
    tof_error=0
    for r,p in [[0,0],[.08,-.07],[-.10,.06]]:
        s=Sensors(noise=False); s.angles=np.array([r,p,0.]); R=rotation(s.angles)
        s.update_tof(R,CFG['pivot_altura_m']+.025,0.)
        tof_error=max(tof_error,abs(s.height-.025))
    assert tof_error<1e-10
    # Una IMU fuera del pivote registra el término tangencial, aunque el pivote esté fijo.
    s=Sensors(noise=False); s.update_imu(np.eye(3),np.zeros(3),np.array([1.,0,0]),np.zeros(3),.005)
    assert abs(s.accel[1])>.005
    c=FirmwareControl(); s=Sensors(noise=False); s.angles[0]=np.deg2rad(12.1)
    assert not c.update(s,.005).any() and c.cut=='INCLINACION'
    c=FirmwareControl('altura'); s=Sensors(noise=False)
    assert not c.update(s,.005).any() and c.cut=='TOF_INVALIDO'
    assert 8*model_force(1500)<(CFG['masa_rotatoria_kg']+CFG['masa_carro_vertical_kg'])*9.81
    out={'equivalencia_PID_cpp_muestras':1600,'error_max_us_antes_holgura':max_error,
        'error_max_pulso_us':int(pulse_error),'error_geometria_tof_m':tof_error,
        'imu_fuera_pivote':True,'corte_tilt':True,'corte_tof_invalido':True,
        'nota':'Equivalencia numérica de ley de control, no validación de la planta física.'}
    (ROOT/'resultados/verificacion_control.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out,indent=2))


if __name__=='__main__': main()
