"""Visor local del conjunto CAD con control de referencia de simulación."""
from pathlib import Path
import time
import threading
import argparse
import mujoco
import mujoco.viewer
from trim_cad import trim
from control_simulacion import ControlReferencia

ROOT = Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--sin-control',action='store_true',help='Repetir el ensayo anterior de empuje constante')
    args=parser.parse_args()
    complete = ROOT/'aerotray_conjunto_cad.xml'
    full = complete.exists()
    model = mujoco.MjModel.from_xml_path(str(complete if full else ROOT/'aerotray_cad_visual.xml'))
    data = mujoco.MjData(model)
    control=ControlReferencia(model) if not args.sin_control else None
    def reset():
        mujoco.mj_resetData(model,data)
        data.qpos[0]=float(model.jnt_range[0].mean())
        data.ctrl[:]=trim(model)
        if control:
            control.reset();control.set_target(float(model.jnt_range[0].mean()))
        mujoco.mj_forward(model,data)
    reset()
    reset_requested=threading.Event();perturb_requested=threading.Event()
    def key_callback(key):
        if key in (ord('R'),ord('r')):reset_requested.set()
        if key in (ord('P'),ord('p')):perturb_requested.set()
    perturb_until=-1.
    print('Conjunto CAD completo; guía/rótula reconstruidas. Dinámica provisional.' if full else 'CAD del UAV con bandeja nueva. Guía/rótula y dinámica provisionales.')
    print('Control de altura e inclinación activo.' if control else 'Empuje constante, sin estabilización.')
    print('P: perturbación pequeña de prueba. R: reiniciar simulación. Cerrar ventana: salir.')
    print('Bola y bandeja rígidas. Parámetros provisionales; esto aún no es entrenamiento en Isaac Lab.')
    with mujoco.viewer.launch_passive(model,data,key_callback=key_callback,show_left_ui=False,show_right_ui=False) as viewer:
        viewer.cam.lookat[:] = [0,0,.45 if full else 1.0]
        viewer.cam.distance = 1.75 if full else 1.65
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        while viewer.is_running():
            start = time.monotonic()
            if reset_requested.is_set():
                reset();reset_requested.clear();perturb_until=-1.
            if perturb_requested.is_set():
                perturb_until=float(data.time)+.15;perturb_requested.clear()
            for _ in range(8):
                if control:data.ctrl[:]=control.update(data,float(model.opt.timestep))[0]
                data.xfrc_applied[:]=0
                if data.time<perturb_until:
                    data.xfrc_applied[control.body if control else model.body('conjunto_rotatorio').id,3:6]=[.6,-.4,.015]
                mujoco.mj_step(model,data)
            if control:
                viewer.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    'MuJoCo | Control de referencia\nP: perturbacion   R: reinicio',
                    f'Altura: {1000*data.qpos[0]:.1f} / {1000*control.target:.1f} mm')])
            viewer.sync()
            time.sleep(max(0,8*model.opt.timestep-(time.monotonic()-start)))


if __name__ == '__main__':
    main()
