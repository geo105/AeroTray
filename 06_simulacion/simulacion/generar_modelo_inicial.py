"""Modelo mecánico simplificado; no contiene parámetros identificados del prototipo."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent


def generate():
    known = json.loads((ROOT / "parametros_actuales.json").read_text())
    mass = known["masas"]["conjunto_movible_total_kg"]
    travel = known["mecanismo"]["recorrido_m"]
    # Reparto provisional: conservar la masa total sin atribuir inercias a Fusion.
    carriage_mass = 0.3
    upper_mass = mass - carriage_mass
    radius = 0.35  # Referencia del PDF, pendiente de medición.
    doc = ET.Element("mujoco", model="AeroTray_provisional_NO_IDENTIFICADO")
    ET.SubElement(doc, "compiler", angle="radian", inertiafromgeom="false")
    ET.SubElement(doc, "option", timestep="0.002", gravity="0 0 -9.80665", integrator="implicitfast")
    visual = ET.SubElement(doc, "visual")
    ET.SubElement(visual, "global", offwidth="1000", offheight="750")
    default = ET.SubElement(doc, "default")
    ET.SubElement(default, "geom", contype="0", conaffinity="0", density="0")
    world = ET.SubElement(doc, "worldbody")
    ET.SubElement(world, "light", pos="1 -1 3", dir="-0.3 0.3 -1")
    ET.SubElement(world, "geom", name="suelo_visual", type="plane", size="2 2 0.1", rgba="0.15 0.18 0.22 1")
    ET.SubElement(world, "camera", name="vista", pos="1.25 -1.5 1.55", xyaxes="0.768 0.640 0 -0.230 0.276 0.934")
    ET.SubElement(world, "geom", name="base_fija_visual", type="box", pos="0 0 0.08", size="0.20 0.20 0.03", rgba="0.25 0.27 0.30 1")
    for y in (-0.065, 0.065):
        ET.SubElement(world, "geom", type="capsule", fromto=f"0 {y} 0.1 0 {y} 1.03", size="0.012", rgba="0.65 0.68 0.7 1")
    carriage = ET.SubElement(world, "body", name="carro_deslizante", pos="0 0 0.8")
    ET.SubElement(carriage, "joint", name="elevacion", type="slide", axis="0 0 1", limited="true", range=f"0 {travel}", damping="0.5", solreflimit="0.005 1")
    ET.SubElement(carriage, "inertial", pos="0 0 -0.04", mass=str(carriage_mass), diaginertia="0.002 0.002 0.002")
    ET.SubElement(carriage, "geom", type="box", pos="0 0 -0.04", size="0.045 0.09 0.04", rgba="0.3 0.6 0.85 1")
    upper = ET.SubElement(carriage, "body", name="conjunto_rotatorio")
    ET.SubElement(upper, "joint", name="rotula", type="ball", damping="0.02")
    ET.SubElement(upper, "inertial", pos="0 0 0.10", mass=str(upper_mass), diaginertia="0.12 0.12 0.20")
    ET.SubElement(upper, "geom", name="junta_visual", type="sphere", size="0.026", rgba="0.95 0.6 0.1 1")
    ET.SubElement(upper, "geom", name="centro_visual", type="box", pos="0 0 0.04", size="0.12 0.12 0.012", rgba="0.65 0.15 0.13 1")
    ET.SubElement(upper, "geom", name="bandeja_rigida_provisional", type="box", pos="0 0 0.20", size="0.14 0.14 0.008", rgba="0.4 0.7 0.5 1")
    actuators = ET.SubElement(doc, "actuator")
    motor_order = (1, 5, 2, 6, 3, 7, 4, 8)
    positions = []
    for i, number in enumerate(motor_order):
        theta = i * math.pi / 4
        x, y = radius * math.cos(theta), -radius * math.sin(theta)
        pos = f"{x:.9f} {y:.9f} 0.04"
        ET.SubElement(upper, "geom", name=f"brazo_M{number}", type="capsule", fromto=f"0 0 0.04 {pos}", size="0.009", rgba="0.65 0.15 0.13 1")
        ET.SubElement(upper, "geom", name=f"helice_visual_M{number}", type="cylinder", pos=pos, size="0.12 0.002", rgba="0.3 0.65 0.9 0.4")
        ET.SubElement(upper, "site", name=f"empuje_M{number}", pos=pos, size="0.007", rgba="1 0.8 0.1 1")
        yaw_sign = 1 if i % 2 == 0 else -1
        # Control en NEWTONS, no PWM. 6 N es un límite ficticio para la demostración.
        ET.SubElement(actuators, "motor", name=f"fuerza_M{number}", site=f"empuje_M{number}", gear=f"0 0 1 0 0 {yaw_sign * 0.015}", ctrllimited="true", ctrlrange="0 6")
        positions.append({"motor": number, "x_m": x, "y_m": y, "z_m": 0.04})
    ET.indent(doc)
    path = ROOT / "aerotray_provisional.xml"
    ET.ElementTree(doc).write(path, encoding="utf-8", xml_declaration=True)
    assumptions = {
        "estado": "PROVISIONAL_NO_EXPORTADO_DE_FUSION_NO_USAR_EN_HARDWARE",
        "confirmados_usuario": {"masa_total_kg": mass, "recorrido_vertical_m": travel, "numero_motores": 8},
        "referencia_pdf_no_verificada": {"radio_motor_m": radius},
        "supuestos_para_demostracion": {
            "masa_carro_kg": carriage_mass, "masa_rotatoria_kg": upper_mass,
            "centro_masa_rotatorio_m": [0, 0, 0.10], "inercia_diagonal_kg_m2": [0.12, 0.12, 0.20],
            "altura_inicial_junta_m": 0.8, "amortiguamiento_vertical_Ns_m": 0.5,
            "amortiguamiento_rotatorio_Nms_rad": 0.02, "fuerza_maxima_motor_N": 6,
            "relacion_par_empuje_m": 0.015, "discos_helice_radio_m": 0.12
        },
        "motor_sites_provisionales": positions,
        "no_modelado": ["bola libre", "servos y bielas", "vision", "baterias y caidas de tension", "dinamica ESC", "friccion real", "limites angulares reales", "movimiento del carrito"],
        "nota": "El dron solo de 2.3 kg no define la masa de todo lo que gira; reparto ficticio conservando 3.6 kg totales."
    }
    (ROOT / "supuestos_modelo.json").write_text(json.dumps(assumptions, indent=2, ensure_ascii=False) + "\n")
    print(path)


if __name__ == "__main__":
    generate()
