"""Compara el puerto C++ con el controlador Python original; sin motores."""
from pathlib import Path
import ctypes
import json
import subprocess
import sys
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'simulacion'))
from control_simulacion import ControlReferencia

src=ROOT/'tmp/puente_control_referencia.cpp'
src.write_text('''#include "../firmware/dron_octo_esp32/control_referencia_sim.h"
static simref::ControlReferencia c;
extern "C" void reset(){c.reiniciar();}
extern "C" int paso(const float* x,float dt,float ref,float* f,float* desired,float* achieved){
 simref::Estado s{x[0],x[1],{x[2],x[3],x[4]},{x[5],x[6],x[7]},x[8]};
 bool ok=c.paso(s,dt,ref,f);for(int k=0;k<4;k++){desired[k]=c.solicitado[k];achieved[k]=c.alcanzado[k];}return ok;
}
''')
lib=ROOT/'tmp/control_referencia.so'
subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',str(src),'-o',str(lib)],check=True)
cpp=ctypes.CDLL(str(lib));ptr=ctypes.POINTER(ctypes.c_float)
cpp.paso.argtypes=[ptr,ctypes.c_float,ctypes.c_float,ptr,ptr,ptr]
model=mujoco.MjModel.from_xml_path(str(ROOT/'simulacion/aerotray_conjunto_cad.xml'))
ctrl=ControlReferencia(model);data=mujoco.MjData(model);rng=np.random.default_rng(73021)
def p(x):return x.ctypes.data_as(ptr)
max_deseado=0.;max_coste_rel=0.;nominal=0;limitados=0
for n in range(500):
    # Secuencia de estados conocidos. MuJoCo calcula exactamente la cinemática
    # usada por la implementación Python para no confundir Euler y rotvec.
    if n%50==0:ctrl.reset();cpp.reset()
    data.qpos[0]=ctrl.target+rng.uniform(-.015,.015)
    rot=rng.uniform(-.15,.15,3);ang=np.linalg.norm(rot)
    data.qpos[1:5]=np.r_[np.cos(ang/2),rot*np.sin(ang/2)/ang]
    data.qvel[:]=rng.uniform(-.1,.1,model.nv)
    mujoco.mj_forward(model,data)
    vel=np.zeros(6);mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,ctrl.body,vel,1)
    from control_simulacion import rotation_vector
    x=np.array([data.qpos[ctrl.z_address],data.qvel[ctrl.v_address],*rotation_vector(data.xquat[ctrl.body]),*vel[:3],data.xmat[ctrl.body].reshape(3,3)[2,2]],dtype=np.float32)
    # Ambos reciben las mismas mediciones redondeadas a float del ESP32.
    # Forzar el cálculo Python con esos datos, conservando su allocate original.
    dt=.004;c=max(.5,float(x[8]));error=ctrl.target-float(x[0])
    vertical=ctrl.mass*ctrl.gravity+ctrl.kp_z*error-ctrl.kd_z*float(x[1])+ctrl.integral_force
    torque=-ctrl.kp*x[2:5]-ctrl.kd*x[5:8]+ctrl.integral_torque
    desired=np.r_[vertical/c,torque]
    forces,limited=ctrl.allocate(desired);achieved=ctrl.matrix@forces
    ctrl.integral_force=float(np.clip(ctrl.integral_force+dt*(ctrl.ki_z*error+5*(achieved[0]*c-vertical)),-12,12))
    ctrl.integral_torque=np.clip(ctrl.integral_torque+dt*(-ctrl.ki*x[2:5]+5*(achieved[1:]-torque)),[-1,-1,-.08],[1,1,.08])
    f=np.zeros(8,np.float32);d=np.zeros(4,np.float32);a=np.zeros(4,np.float32)
    assert cpp.paso(p(x),dt,ctrl.target,p(f),p(d),p(a))==1
    assert np.isfinite(f).all() and f.min()>=0 and f.max()<=6
    max_deseado=max(max_deseado,float(np.abs(d-desired).max()))
    cost_py=np.linalg.norm(ctrl.weights*(achieved-desired))**2
    cost_cpp=np.linalg.norm(ctrl.weights*(a-desired))**2
    max_coste_rel=max(max_coste_rel,float((cost_cpp-cost_py)/max(1,cost_py)))
    if limited:limitados+=1
    else:
        nominal+=1
        assert np.max(np.abs(a-achieved))<2e-3
assert max_deseado<2e-3,(max_deseado,max_coste_rel)
assert max_coste_rel<1e-3,(max_deseado,max_coste_rel)
report={'estados_comparados':500,'sin_saturacion':nominal,'con_saturacion':limitados,'max_error_esfuerzo_deseado_SI':max_deseado,'max_exceso_coste_relativo_asignacion':max_coste_rel,'pasa':True,'nota':'Paridad numerica del modelo restringido. No demuestra vuelo libre ni conversion N a PWM.'}
out=ROOT/'registros/comparacion_control_simulacion.json';out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
