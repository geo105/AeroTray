"""Comprobación mecánica en MuJoCo del archivo que también se importa en Newton."""
import argparse
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    import numpy as np

    model = mujoco.MjModel.from_xml_path(str(ROOT / "aerotray_provisional.xml"))
    assert model.nu == 8 and model.nv == 4
    mass = float(model.body_mass.sum())
    assert abs(mass - 3.6) < 1e-8
    results = {"modelo": "PROVISIONAL", "motor": "MuJoCo", "masa_total_kg": mass, "actuadores": model.nu, "grados_libertad": model.nv, "casos": {}}
    for case, force in (("sin_empuje", 0.0), ("equilibrio_ideal", mass * 9.80665 / 8), ("tope_superior", 6.0)):
        data = mujoco.MjData(model)
        data.qpos[0] = 0.10
        data.ctrl[:] = force
        heights = []
        for _ in range(1000):
            mujoco.mj_step(model, data)
            heights.append(float(data.qpos[0]))
        assert np.isfinite(data.qpos).all()
        assert min(heights) > -0.005 and max(heights) < 0.205
        if case == "sin_empuje":
            assert abs(data.qpos[0]) < 0.005
        elif case == "equilibrio_ideal":
            assert abs(data.qpos[0] - 0.10) < 0.001
        else:
            assert abs(data.qpos[0] - 0.20) < 0.005
        results["casos"][case] = {"tiempo_s": float(data.time), "empuje_por_motor_N": force, "altura_final_m": float(data.qpos[0]), "minimo_m": min(heights), "maximo_m": max(heights)}
    if args.render:
        from PIL import Image, ImageDraw
        data = mujoco.MjData(model)
        data.qpos[0] = 0.10
        mujoco.mj_forward(model, data)
        with mujoco.Renderer(model, height=750, width=1000) as renderer:
            renderer.update_scene(data, camera="vista")
            picture = Image.fromarray(renderer.render())
        draw = ImageDraw.Draw(picture)
        draw.rectangle((0, 0, 1000, 65), fill=(24, 29, 36))
        draw.text((18, 14), "AEROTRAY - MODELO PROVISIONAL | NO EXPORTADO DE FUSION", fill="white")
        draw.text((18, 36), "8 puntos de empuje | guias 0.20 m | junta esferica | masa total 3.6 kg", fill="white")
        picture.save(ROOT / "modelo_inicial.png")
    (ROOT / "verificacion_modelo.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
