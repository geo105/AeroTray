"""Comprobar importación CAD y que no modifica la dinámica provisional."""
from pathlib import Path
import json
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parent


def run(filename):
    model = mujoco.MjModel.from_xml_path(str(ROOT/filename))
    data = mujoco.MjData(model)
    assert model.nu == 8
    mass = float(model.body_mass.sum())
    assert abs(mass-3.6)<1e-9
    data.qpos[0] = .1
    data.ctrl[:] = mass*9.80665/8
    mujoco.mj_forward(model,data)
    for _ in range(1000):
        mujoco.mj_step(model,data)
    assert np.isfinite(data.qpos).all()
    assert abs(data.qpos[0]-.1)<.001
    return model,data


def main():
    original, a = run('aerotray_provisional.xml')
    converted, b = run('aerotray_cad_visual.xml')
    assert np.allclose(a.qpos,b.qpos,atol=1e-9)
    assert converted.nmesh == 6
    result = {'estado':'importacion_CAD_visual_y_simulacion_basica_correctas',
              'motor_fisico':'MuJoCo', 'version':mujoco.__version__,
              'duracion_s':float(b.time), 'masa_total_kg':float(converted.body_mass.sum()),
              'actuadores':converted.nu, 'mallas':converted.nmesh,
              'altura_deslizamiento_final_m':float(b.qpos[0]),
              'diferencia_qpos_con_modelo_provisional':float(np.max(np.abs(a.qpos-b.qpos))),
              'limite':'Prueba de importación de geometría; no valida empuje, sensores, control ni ball-and-plate.'}
    (ROOT/'verificacion_cad_visual.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
