#!/usr/bin/env python3
"""Exporta a URDF las mallas métricas ya colocadas del conjunto CAD completo."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
CAD = ROOT / 'cad/junta_nuevo_de_8_motores_con_camara/conjunto_completo'
OUT = Path(__file__).resolve().parent
GROUPS = ('carrito_fijo', 'carro_vertical', 'conjunto_rotatorio')


def main():
    manifest = json.loads((CAD / 'ensamblaje_completo.json').read_text())
    parts = manifest['piezas']
    robot = ET.Element('robot', name='junta_nuevo_de_8_motores_con_camara')
    ET.SubElement(robot, 'link', name='cad_mundo')
    for group in GROUPS:
        link = ET.SubElement(robot, 'link', name='cad_' + group)
        joint = ET.SubElement(robot, 'joint', name='cad_fijo_' + group, type='fixed')
        ET.SubElement(joint, 'parent', link='cad_mundo')
        ET.SubElement(joint, 'child', link='cad_' + group)
        ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
        for part in parts:
            if part['cuerpo'] != group:
                continue
            mesh = CAD / part['malla']
            if not mesh.is_file():
                raise FileNotFoundError(mesh)
            visual = ET.SubElement(link, 'visual', name=f"pieza_{part['indice']:04d}")
            # Los STL ya están en metros y en coordenadas del conjunto.
            # No se aplica de nuevo la transformación ni la conversión mm -> m.
            ET.SubElement(visual, 'origin', xyz='0 0 0', rpy='0 0 0')
            geom = ET.SubElement(visual, 'geometry')
            ET.SubElement(geom, 'mesh', filename=mesh.resolve().as_uri(), scale='1 1 1')
            material = ET.SubElement(visual, 'material', name=f"color_{part['indice']:04d}")
            ET.SubElement(material, 'color', rgba=' '.join(map(str, part['color'])))
    assert len(robot.findall('.//visual')) == len(parts) == 383
    ET.indent(robot, space='  ')
    target = OUT / 'conjunto_completo.urdf'
    ET.ElementTree(robot).write(target, encoding='utf-8', xml_declaration=True)
    summary = {
        'origen': str(CAD / 'ensamblaje_completo.json'),
        'urdf': str(target), 'piezas': len(parts), 'escala': 'metros, 1:1',
        'grupos': {g: sum(p['cuerpo'] == g for p in parts) for g in GROUPS},
        'limites_m': [[min(p['limites_m'][0][i] for p in parts) for i in range(3)],
                      [max(p['limites_m'][1][i] for p in parts) for i in range(3)]],
        'modo': 'pose CAD fija; no física, no telemetría, sin control de hardware',
        'materiales': 'colores de identificación de la exportación local, no materiales nativos de Fusion',
    }
    (OUT / 'conjunto_completo_verificacion.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
