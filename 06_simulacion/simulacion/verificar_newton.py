"""Importar el modelo provisional y probar su equilibrio en Newton/MJWarp en GPU."""
from pathlib import Path
from importlib.metadata import version
import json
import os
import argparse

ROOT = Path(__file__).resolve().parent
os.environ.setdefault("XDG_CACHE_HOME", str(ROOT / "cache"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--modelo', type=Path, default=ROOT/'aerotray_provisional.xml')
    parser.add_argument('--salida', type=Path, default=ROOT/'verificacion_newton.json')
    args = parser.parse_args()
    import newton
    import numpy as np
    import warp as wp

    wp.init()
    if not wp.is_cuda_available():
        raise RuntimeError("La prueba requiere la GPU NVIDIA; no se usará CPU como sustituto.")
    builder = newton.ModelBuilder(gravity=(0.,0.,-9.80665))
    builder.add_mjcf(str(args.modelo.resolve()), ctrl_direct=True)
    builder.joint_q[0] = 0.10
    model = builder.finalize(device="cuda:0")
    total_mass = float(model.body_mass.numpy().sum())
    assert abs(total_mass - 3.6) < 1e-4
    solver = newton.solvers.SolverMuJoCo(model, disable_contacts=True)
    state_a, state_b = model.state(), model.state()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)
    control = model.control()
    assert control.mujoco.ctrl.shape[0] == 8
    control.mujoco.ctrl.assign(np.full(8, total_mass * 9.80665 / 8, dtype=np.float32))
    for _ in range(100):
        state_a.clear_forces()
        solver.step(state_a, state_b, control, None, 0.002)
        state_a, state_b = state_b, state_a
    wp.synchronize()
    coordinates = state_a.joint_q.numpy()
    assert np.isfinite(coordinates).all()
    assert abs(float(coordinates[0]) - 0.10) < 0.005
    results = {
        "estado": "modelo_importado_y_equilibrio_ideal_provisional_verificado",
        "modelo": str(args.modelo.resolve()),
        "backend": "Newton SolverMuJoCo MJWarp", "dispositivo": str(model.device),
        "versiones": {name: version(name) for name in ("newton", "warp-lang", "mujoco", "mujoco-warp")},
        "masa_total_kg": total_mass, "actuadores": control.mujoco.ctrl.shape[0],
        "duracion_simulada_s": 0.2, "altura_final_m": float(coordinates[0]),
        "limite": "No es tarea de Isaac Lab ni entrenamiento ni validacion del prototipo real"
    }
    args.salida.write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
