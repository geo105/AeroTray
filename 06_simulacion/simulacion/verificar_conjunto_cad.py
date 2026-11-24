"""Importación y equilibrio ideal del conjunto completo: no valida hardware."""
from pathlib import Path
import argparse
import json
import os
import numpy as np
import mujoco
from trim_cad import trim

ROOT=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--newton',action='store_true')
    parser.add_argument('--render',action='store_true');args=parser.parse_args()
    path=ROOT/'aerotray_conjunto_cad.xml'
    known=json.loads((ROOT/'supuestos_conjunto_cad.json').read_text())
    model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model)
    assert model.nu==8 and model.nv==4 and model.nq==5
    assert abs(float(model.body_mass.sum())-3.6)<1e-9
    assert np.allclose(model.jnt_range[0],[0,known['recorrido_simulacion_m']])
    target=float(model.jnt_range[0].mean());data.qpos[0]=target;data.ctrl[:]=trim(model)
    mujoco.mj_forward(model,data)
    for _ in range(1000):mujoco.mj_step(model,data)
    assert np.isfinite(data.qpos).all() and abs(data.qpos[0]-target)<.001
    result={'estado':'conjunto_CAD_importado_y_equilibrio_ideal_verificado',
        'masa_movil_kg':float(model.body_mass.sum()),'actuadores':model.nu,'grados_libertad':model.nv,
        'mallas_visuales':model.nmesh,'recorrido_simulacion_m':float(model.jnt_range[0,1]),
        'duracion_MuJoCo_s':float(data.time),'altura_final_MuJoCo_m':float(data.qpos[0]),
        'error_equilibrio_m':float(data.qpos[0]-target),'controles_N':data.ctrl.tolist(),
        'limite':'Prueba de importación, cuerpos y fuerzas ideales; sin identificación ni entrenamiento ni colisiones CAD.'}
    if args.render:
        camera=mujoco.MjvCamera();camera.lookat[:]=[0,0,.45];camera.distance=1.75
        camera.azimuth=135;camera.elevation=-20
        renderer=mujoco.Renderer(model,height=900,width=1200)
        renderer.update_scene(data,camera=camera)
        import PIL.Image
        PIL.Image.fromarray(renderer.render()).save(ROOT/'conjunto_cad_simulacion.png')
        renderer.close()
    if args.newton:
        os.environ.setdefault('XDG_CACHE_HOME',str(ROOT/'cache'))
        import newton
        import warp as wp
        wp.init();assert wp.is_cuda_available()
        builder=newton.ModelBuilder(gravity=(0.,0.,-9.80665));builder.add_mjcf(str(path),ctrl_direct=True)
        builder.joint_q[0]=target
        nmodel=builder.finalize(device='cuda:0')
        solver=newton.solvers.SolverMuJoCo(nmodel,disable_contacts=True)
        a,b=nmodel.state(),nmodel.state();newton.eval_fk(nmodel,nmodel.joint_q,nmodel.joint_qd,a)
        control=nmodel.control();control.mujoco.ctrl.assign(data.ctrl.astype(np.float32))
        for _ in range(100):
            a.clear_forces();solver.step(a,b,control,None,.002);a,b=b,a
        wp.synchronize();q=a.joint_q.numpy()
        assert np.isfinite(q).all() and abs(float(q[0])-target)<.001
        result['Newton_GPU']={'dispositivo':str(nmodel.device),'duracion_s':.2,
            'altura_final_m':float(q[0]),'masa_kg':float(nmodel.body_mass.numpy().sum())}
    (ROOT/'verificacion_conjunto_cad.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
