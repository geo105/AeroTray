"""Releer el STEP y comprobar transformaciones y recortes de las bielas."""
from pathlib import Path
import json
import numpy as np
import trimesh
from leer_ensamblaje import read_document, walk_document
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Trsf, gp_Pnt

ROOT = Path(__file__).resolve().parent
OUT = ROOT/'junta_nuevo_de_8_motores_con_camara'


def main():
    expected = json.loads((OUT/'ensamblaje.json').read_text())
    doc, tool, roots = read_document(OUT/'junta nuevo de 8 motores con camara - UAV preliminar.step')
    print('STEP transferido',flush=True)
    rows, shapes, limits = walk_document(tool,roots)
    assert len(shapes)==58
    targets = {p['nombre']:p for p in expected['piezas']}
    assert set(targets)=={rows[i]['nombre'] for i in shapes}
    scale = gp_Trsf()
    scale.SetScale(gp_Pnt(0,0,0),.001)
    report = []
    for i,shape in shapes.items():
        row = rows[i]
        target = targets[row['nombre']]
        error = float(np.max(np.abs(np.array(row['limites_mundo_mm']).reshape(2,3)/1000-target['limites_m'])))
        record = {'pieza':row['nombre'], 'error_caja_analitica_mm':error*1000}
        # La caja analítica de superficies recortadas puede ser conservadora
        # después de la traducción STEP. Comprobar su triangulación real.
        if error>.001:
            metric = BRepBuilderAPI_Transform(shape,scale,True).Shape()
            BRepMesh_IncrementalMesh(metric,.0005,False,.5,True)
            path = ROOT/'procesados'/f'verificacion_recorte_{target["indice"]:03d}.stl'
            writer=StlAPI_Writer()
            writer.ASCIIMode=False
            assert writer.Write(metric,str(path))
            mesh=trimesh.load_mesh(path,process=True)
            mesh_error=float(np.max(np.abs(mesh.bounds-target['limites_m'])))
            record['error_limites_triangulados_mm']=mesh_error*1000
            assert mesh_error<.001, row['nombre']+' cambió de geometría'
        report.append(record)
    result={'estado':'STEP_nuevo_releido_y_geometria_visual_comprobada','componentes':58,
            'limites_mm':limits, 'comprobaciones':report, 'archivo_nativo_Fusion_modificado':False,
            'carrito_integrado':False}
    (OUT/'verificacion_step.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print('58 componentes y geometría comprobados',flush=True)


if __name__=='__main__':
    main()
