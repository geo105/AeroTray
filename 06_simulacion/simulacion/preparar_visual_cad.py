"""Usar el CAD recompuesto como visual del mecanismo provisional existente.

No identifica la dinámica ni añade control del ball-and-plate. La bola y los
servos siguen rígidos para esta primera prueba de importación.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import vtk
from vtk.util.numpy_support import vtk_to_numpy

ROOT = Path(__file__).resolve().parent
CAD = ROOT.parent / 'cad/junta_nuevo_de_8_motores_con_camara'
OUT = ROOT / 'mallas_cad'


def category(part):
    name = part['ruta_origen'][1]
    if part['grupo'] == 'estructura_uav':
        return 'estructura', [.23,.27,.32,1]
    if name.startswith('CAM '):
        return 'soporte_camara', [.84,.42,.12,1]
    if 'servo' in name.lower() or 'brazo 25T' in name:
        return 'servos', [.12,.35,.65,1]
    if 'bola 25' in name:
        return 'bola_referencia_rigida', [.83,.12,.13,1]
    if 'mesa mitad' in name or 'pantalla' in name:
        return 'plato', [.25,.58,.39,1]
    return 'mecanismo_bandeja', [.46,.49,.52,1]


def main():
    OUT.mkdir(exist_ok=True)
    data = json.loads((CAD/'ensamblaje.json').read_text())
    groups = {}
    scene = trimesh.Scene()
    reduction = []
    for part in data['piezas']:
        group, rgba = category(part)
        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(CAD/part['malla']))
        reader.Update()
        poly = reader.GetOutput()
        original = poly.GetNumberOfCells()
        if original > 20000:
            decimator = vtk.vtkQuadricDecimation()
            decimator.SetInputData(poly)
            decimator.SetTargetReduction(1 - 20000/original)
            decimator.Update()
            poly = decimator.GetOutput()
        vertices = vtk_to_numpy(poly.GetPoints().GetData()).copy()
        indices = vtk_to_numpy(poly.GetPolys().GetConnectivityArray()).copy().reshape(-1,3)
        mesh = trimesh.Trimesh(vertices=vertices, faces=indices, process=False)
        assert np.isfinite(mesh.vertices).all()
        expected = np.array(part['limites_m'])
        assert np.max(np.abs(mesh.bounds-expected))<.002, 'Simplificación alteró límites'
        mesh.visual.vertex_colors = np.tile(np.round(np.array(rgba)*255).astype(np.uint8), (len(vertices),1))
        scene.add_geometry(mesh, node_name=f'{part["indice"]:03d} {part["nombre"]}')
        groups.setdefault(group, {'rgba':rgba,'meshes':[]})['meshes'].append(mesh)
        reduction.append({'pieza':part['nombre'], 'original':original, 'ligera':len(mesh.faces)})
    scene.export(str(CAD/'uav_camara_visual.glb'))
    xml = ET.parse(ROOT/'aerotray_provisional.xml')
    doc = xml.getroot()
    doc.set('model','AeroTray_CAD_visual_DINAMICA_PROVISIONAL')
    upper = doc.find('.//body[@name="conjunto_rotatorio"]')
    for geom in list(upper.findall('geom')):
        if geom.get('name') != 'junta_visual':
            upper.remove(geom)
    assets = ET.SubElement(doc,'asset')
    for group, values in groups.items():
        combined = trimesh.util.concatenate(values['meshes'])
        target = OUT/f'{group}.stl'
        combined.export(target)
        ET.SubElement(assets,'mesh',name=group,file=str(target),inertia='shell')
        ET.SubElement(upper,'geom',name='CAD_'+group,type='mesh',mesh=group,
                      pos='0 0 0.04',rgba=' '.join(map(str,values['rgba'])),group='2')
    # Valores físicos conservados de la demostración; el CAD sólo cambia visuales.
    ET.indent(xml)
    xml.write(ROOT/'aerotray_cad_visual.xml',encoding='utf-8',xml_declaration=True)
    result = {'estado':'CAD_visual_sobre_mecanismo_provisional',
              'malla_glb':str(CAD/'uav_camara_visual.glb'), 'modelo':str(ROOT/'aerotray_cad_visual.xml'),
              'triangulos_originales':sum(x['original'] for x in reduction),
              'triangulos_visuales_ligeros':sum(x['ligera'] for x in reduction),
              'piezas':reduction,
              'supuestos_no_identificados':['posición del pivote respecto al UAV',
                  'reparto de masa e inercias', 'posiciones y numeración de actuadores',
                  'límite de empuje y relación de par', 'base y guías simplificadas'],
              'limitaciones':['bola y bandeja rígidas en esta prueba', 'sin cámara simulada',
                  'sin hélices visuales exportadas en el CAD recibido', 'sin colisiones de las mallas',
                  'no entrenamiento ni validación de control del prototipo']}
    (ROOT/'cad_visual.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='piezas'},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
