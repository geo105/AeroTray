"""Copia neutral del UAV con bandeja nueva; originales Fusion intactos.

Las piezas compartidas verifican el marco de montaje antes de sustituir.
Las mallas son visuales: no generan masas ni articulaciones por inferencia.
"""
from pathlib import Path
import json
import numpy as np
import trimesh
from leer_ensamblaje import read_document, walk_document
from OCP.BRep import BRep_Tool
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepTools import BRepTools
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Pnt, gp_Trsf, gp_Vec
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.IFSelect import IFSelect_RetDone

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'junta_nuevo_de_8_motores_con_camara'
OFFSET = np.array([2.0000574, 1081.4999356, 1094.9244173])


def load(name):
    print('Importando ' + name, flush=True)
    doc, tool, roots = read_document(ROOT / 'recibidos' / name)
    rows, shapes, root_bounds = walk_document(tool, roots)
    boxes = np.array([r['limites_mundo_mm'] for r in rows if r.get('limites_mundo_mm')])
    union = np.r_[boxes[:,:3].min(0), boxes[:,3:].max(0)]
    assert len(root_bounds) == 1 and np.allclose(union, root_bounds[0], atol=1e-5), 'Transformaciones inconsistentes'
    print(f'Verificados {len(shapes)} componentes geométricos', flush=True)
    return doc, rows, shapes


def name(label, value):
    TDataStd_Name.Set_s(label, TCollection_ExtendedString(value))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    meshes = OUT / 'mallas'
    meshes.mkdir(exist_ok=True)
    tray_doc, tray_rows, tray_shapes = load('ball_and_plate_camara.stp')
    uav_doc, uav_rows, uav_shapes = load('x8_UAV_R350_imprimible_bandeja_v5.stp')
    shared = ['PISO2 Q1:1', 'PISO2 Q2:1', 'PISO2 Q3:1', 'PISO2 Q4:1', 'BANDEJA cruceta:1']
    comparisons = []
    for part in shared:
        a = next(r for r in tray_rows if r['nombre'] == part)
        b = next(r for r in uav_rows if r['nombre'] == part)
        error = float(np.max(np.abs(np.array(a['limites_mundo_mm'])-b['limites_mundo_mm'])))
        assert error < 1e-5, 'Montaje distinto en ' + part
        assert np.allclose(a['matriz_mundo_mm'], b['matriz_mundo_mm'], atol=1e-7)
        comparisons.append({'pieza':part, 'error_limites_mm':error})
    selections = []
    kept = []
    removed = []
    for index, shape in uav_shapes.items():
        row = uav_rows[index]
        branch = row['ruta'][1]
        if branch.startswith(('OCTO - ', 'IMPRIMIBLE ', 'SEPARADOR ')):
            selections.append(('estructura_uav', row, shape))
            kept.append(branch)
        else:
            removed.append(branch)
    for index, shape in tray_shapes.items():
        selections.append(('bandeja_camara', tray_rows[index], shape))
    assert sum(r['ruta'][1].startswith('OCTO - ') for _,r,_ in selections) == 8
    doc = TDocStd_Document(TCollection_ExtendedString('CadXCAF'))
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    root = tool.NewShape()
    name(root, 'junta nuevo de 8 motores con camara - UAV preliminar')
    groups = {}
    for group in ('estructura_uav', 'bandeja_camara'):
        definition = tool.NewShape()
        name(definition, group)
        instance = tool.AddComponent(root, definition, TopLoc_Location())
        name(instance, group)
        groups[group] = definition
    shift = gp_Trsf()
    shift.SetTranslation(gp_Vec(*(-OFFSET)))
    scale = gp_Trsf()
    scale.SetScale(gp_Pnt(0,0,0), 0.001)
    report = []
    for i, (group, row, shape) in enumerate(selections):
        print(f'Pieza {i+1}/{len(selections)}: {row["nombre"]}', flush=True)
        centered = shape.Moved(TopLoc_Location(shift))
        definition = tool.AddShape(centered, False)
        name(definition, row['nombre'])
        instance = tool.AddComponent(groups[group], definition, TopLoc_Location())
        name(instance, row['nombre'])
        # BREP permite reutilizar geometría sin volver a importar STEP grande.
        if not BRepTools.Write_s(centered, str(meshes / f'{i:03d}.brep')):
            raise RuntimeError('No se pudo guardar BREP')
        metric = BRepBuilderAPI_Transform(centered, scale, True).Shape()
        BRepMesh_IncrementalMesh(metric, 0.0005, False, 0.5, True)
        missing = 0
        faces = TopExp_Explorer(metric, TopAbs_FACE)
        while faces.More():
            tri = BRep_Tool.Triangulation_s(TopoDS.Face(faces.Current()), TopLoc_Location())
            missing += int(tri is None or tri.NbTriangles() == 0)
            faces.Next()
        path = meshes / f'{i:03d}.stl'
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        if not writer.Write(metric, str(path)):
            raise RuntimeError('No se pudo escribir ' + str(path))
        mesh = trimesh.load_mesh(path, process=True)
        assert len(mesh.faces)>0 and np.isfinite(mesh.vertices).all()
        source_box = np.array(row['limites_mundo_mm']).reshape(2,3)
        expected = (source_box - OFFSET) * .001
        assert np.allclose(mesh.bounds, expected, atol=.001), 'Escala/posición incorrecta'
        report.append({'indice':i, 'grupo':group, 'nombre':row['nombre'], 'ruta_origen':row['ruta'],
                       'malla':str(path.relative_to(OUT)), 'limites_m':mesh.bounds.tolist(),
                       'triangulos':len(mesh.faces), 'caras_sin_triangulacion':missing,
                       'cerrada':bool(mesh.is_watertight)})
    tool.UpdateAssemblies()
    result = {'estado':'geometria_UAV_sustituida_en_copia_neutral_no_ensamblaje_completo',
              'unidad_step':'mm', 'unidad_mallas':'m', 'origen_cad_resta_mm':OFFSET.tolist(),
              'verificacion_montaje':comparisons, 'piezas':report,
              'ramas_uav_conservadas':sorted(set(kept)), 'ramas_uav_retiradas':sorted(set(removed)),
              'pendiente':['STEP carrito y rótula', 'uniones físicas', 'masas e inercias',
                           'curva de empuje', 'revisión de interferencias del montaje',
                           'modelo electrónico real de cámara en soporte nuevo'],
              'nota':'STEP neutral con nombres, no archivo nativo de Fusion ni simulación validada. REF pantalla se conserva como referencia geométrica de superficie, no como sensor funcional.'}
    (OUT / 'ensamblaje.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print('Escribiendo STEP con nombres...', flush=True)
    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    assert writer.Transfer(doc)
    target = OUT / 'junta nuevo de 8 motores con camara - UAV preliminar.step'
    assert writer.Write(str(target)) == IFSelect_RetDone
    print('STEP listo: ' + str(target), flush=True)


if __name__ == '__main__':
    main()
