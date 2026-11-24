"""Recomponer el carrito y UAV con geometría local, sin modificar originales.

Se conserva la interfaz del ensamblaje recibido, incluidos sus solapes
heredados. La clasificación de cuerpos es una interpretación del STEP,
no la recuperación de las uniones nativas de Fusion.
"""
from pathlib import Path
import json
import numpy as np
import trimesh
import vtk
from vtk.util.numpy_support import vtk_to_numpy
from OCP.BRep import BRep_Builder
from OCP.BRepTools import BRepTools
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopoDS import TopoDS_Shape
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf,gp_Vec,gp_Pnt
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.IFSelect import IFSelect_RetDone
from OCP.StlAPI import StlAPI_Writer
from leer_ensamblaje import bounds

ROOT=Path(__file__).resolve().parent
UAV=ROOT/'junta_nuevo_de_8_motores_con_camara'
OUT=UAV/'conjunto_completo'


def read_brep(path):
    shape=TopoDS_Shape()
    if not BRepTools.Read_s(shape,str(path),BRep_Builder()):
        raise RuntimeError('No se pudo leer '+str(path))
    return shape


def translation(xyz):
    t=gp_Trsf();t.SetTranslation(gp_Vec(*map(float,xyz)))
    return TopLoc_Location(t)


def set_name(label,value):
    TDataStd_Name.Set_s(label,TCollection_ExtendedString(value))


def transform_matrix(location):
    t=location.Transformation()
    m=np.eye(4)
    m[:3,:]=[[t.Value(i,j) for j in range(1,5)] for i in range(1,4)]
    m[:3,3]*=.001
    return m


def mesh_template(shape,path):
    scale=gp_Trsf();scale.SetScale(gp_Pnt(0,0,0),.001)
    metric=BRepBuilderAPI_Transform(shape.Located(TopLoc_Location()),scale,True).Shape()
    BRepMesh_IncrementalMesh(metric,.0005,False,.5,True)
    writer=StlAPI_Writer();writer.ASCIIMode=False
    assert writer.Write(metric,str(path))
    reader=vtk.vtkSTLReader();reader.SetFileName(str(path));reader.Update()
    poly=reader.GetOutput()
    if poly.GetNumberOfCells()>16000:
        dec=vtk.vtkQuadricDecimation();dec.SetInputData(poly)
        dec.SetTargetReduction(1-16000/poly.GetNumberOfCells());dec.Update()
        poly=dec.GetOutput()
    vertices=vtk_to_numpy(poly.GetPoints().GetData()).copy()
    faces=vtk_to_numpy(poly.GetPolys().GetConnectivityArray()).copy().reshape(-1,3)
    assert len(faces)>0 and np.isfinite(vertices).all()
    return trimesh.Trimesh(vertices=vertices,faces=faces,process=False)


def tint(group,name):
    if name.startswith('CAM '):return [.84,.42,.12,1]
    if 'bola 25' in name:return [.83,.12,.13,1]
    if 'mesa mitad' in name or 'pantalla' in name:return [.25,.58,.39,1]
    if group=='carro_vertical':return [.22,.48,.68,1]
    if 'servo' in name.lower() or 'brazo 25T' in name:return [.12,.35,.65,1]
    if 'HELICE' in name:return [.46,.56,.65,1]
    if group=='carrito_fijo':return [.48,.52,.56,1]
    return [.23,.27,.32,1]


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'mallas').mkdir(exist_ok=True)
    (OUT/'plantillas').mkdir(exist_ok=True)
    source=json.loads((ROOT/'procesados/carrito_arbol.json').read_text())
    rows=source['componentes']
    uav=json.loads((UAV/'ensamblaje.json').read_text())
    mechanisms=json.loads((ROOT/'procesados/mecanismo_geometria.json').read_text())
    joint=next(x for x in mechanisms if x['indice']==74)
    spheres=[x for x in joint['superficies'] if x['tipo']=='esfera']
    pivot=np.array(spheres[0]['centro'])
    assert all(np.linalg.norm(np.array(x['centro'])-pivot)<1e-6 for x in spheres)
    origin=np.array([pivot[0],pivot[1],source['limites_raices_mm'][0][2]])
    shift=translation(-origin)
    # Centrar el patrón nominal 20 x 55 mm de la base nueva en el patrón
    # medido de la interfaz antigua (20 x 54.996606 mm).
    interface=next(x for x in mechanisms if x['indice']==82)
    mounting=np.array([s['punto'] for s in interface['superficies']
                       if s['tipo']=='cilindro' and abs(s['eje'][2])>.99 and 2.4<s['radio']<2.7])
    assert mounting.shape==(4,3)
    placement=np.array([*mounting[:,:2].mean(0),rows[81]['limites_mundo_mm'][2]])
    attach=translation(placement)
    cap_top=min(rows[i]['limites_mundo_mm'][5] for i in (68,70))
    bearing_bottom=min(rows[i]['limites_mundo_mm'][2] for i in (72,73))
    nominal_stroke=bearing_bottom-cap_top
    assert 0<nominal_stroke<100
    numerical_margin=2.0
    stroke=(nominal_stroke-numerical_margin)*.001
    axes=json.loads((ROOT/'procesados/ejes_motores_nuevos.json').read_text())
    # Relación de numeración tomada de las hélices del CAD original,
    # pendiente de comprobar contra el cableado físico.
    motor_ids={0:6,1:7,2:8,3:5,4:1,5:4,6:2,7:3}
    old_prop_z=rows[469]['matriz_mundo_mm'][2][3]
    old_motor_top=rows[463]['limites_mundo_mm'][5]
    prop_offset=old_prop_z-old_motor_top
    motor_positions={}
    for axis in axes:
        index=axis['indice']
        top=read_brep(UAV/f'mallas/{index:03d}.brep')
        z=bounds(top)[5]+prop_offset
        motor_positions[motor_ids[index]]=np.r_[axis['eje_xy_mm'],z]+placement
    selections=[]
    removed=[]
    for i,row in enumerate(rows):
        if row['ensamblaje']:continue
        branch=row['ruta'][1]
        if branch.startswith('COMPLEX_UAV') and i not in (78,82):
            removed.append(row['ruta']);continue
        if branch.startswith(('Brushless motor','30A ESC','OCTO - 4 conjuntos')):
            removed.append(row['ruta']);continue
        shape=read_brep(ROOT/f'procesados/carrito_brep/{i:04d}.brep')
        group='carrito_fijo'
        if i in (66,68,69,70,71,75,76):group='carro_vertical'
        if (branch.startswith(('Lipo','ESP32','830p','OCTO - helices')) or i in (78,82)
            or branch=='Fermion_MPU6050 board v1:2'):
            group='conjunto_rotatorio'
        if i==74:
            explorer=TopExp_Explorer(shape,TopAbs_SOLID);n=0
            while explorer.More():
                solid=explorer.Current()
                rotary=n in (3,4,5)
                group='conjunto_rotatorio' if rotary else 'carro_vertical'
                selections.append({'shape':solid,'location':shift*solid.Location(),
                    'group':group,'name':f'Rotula esfera - solido {n}',
                    'key':f'rotula_{n}','source':row['ruta']+[f'solido {n}']})
                explorer.Next();n+=1
            assert n==7
            continue
        location=shape.Location()
        label=row['nombre']
        if branch.startswith('OCTO - helices'):
            number=int(row['ruta'][2].split('_')[0][1:])
            t=location.Transformation()
            t.SetTranslationPart(gp_Vec(*map(float,motor_positions[number])))
            location=TopLoc_Location(t)
            label=f'M{number}_HELICE_9450_REFERENCIA'
        selections.append({'shape':shape,'location':shift*location,'group':group,
                           'name':label,'key':'original_'+row['definicion_id'].replace(':','_'),
                           'source':row['ruta']})
    for part in uav['piezas']:
        i=part['indice']
        shape=read_brep(UAV/f'mallas/{i:03d}.brep')
        selections.append({'shape':shape,'location':shift*attach*shape.Location(),
            'group':'conjunto_rotatorio','name':part['nombre'],'key':f'nuevo_{i}',
            'source':part['ruta_origen']})
    doc=TDocStd_Document(TCollection_ExtendedString('CadXCAF'))
    tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    root=tool.NewShape();set_name(root,'junta nuevo de 8 motores con camara')
    groups={}
    for group in ('carrito_fijo','carro_vertical','conjunto_rotatorio'):
        label=tool.NewShape();set_name(label,group)
        instance=tool.AddComponent(root,label,TopLoc_Location());set_name(instance,group)
        groups[group]=label
    definitions={};templates={};report=[];scene=trimesh.Scene()
    for i,item in enumerate(selections):
        print(f'{i+1}/{len(selections)} {item["group"]} {item["name"]}',flush=True)
        key=item['key'];local=item['shape'].Located(TopLoc_Location())
        if key not in definitions:
            definition=tool.AddShape(local,False);set_name(definition,item['name'])
            definitions[key]=definition
            templates[key]=mesh_template(local,OUT/f'plantillas/{len(templates):03d}.stl')
        instance=tool.AddComponent(groups[item['group']],definitions[key],item['location'])
        set_name(instance,item['name'])
        mesh=templates[key].copy();mesh.apply_transform(transform_matrix(item['location']))
        assert np.isfinite(mesh.vertices).all()
        rgba=tint(item['group'],item['name'])
        mesh.visual.vertex_colors=np.tile(np.round(np.array(rgba)*255).astype(np.uint8),(len(mesh.vertices),1))
        path=OUT/f'mallas/{i:04d}.stl';mesh.export(path)
        scene.add_geometry(mesh,node_name=f'{i:04d} {item["name"]}')
        report.append({'indice':i,'cuerpo':item['group'],'nombre':item['name'],'ruta_origen':item['source'],
            'malla':str(path.relative_to(OUT)),'triangulos':len(mesh.faces),'color':rgba,
            'limites_m':mesh.bounds.tolist()})
    tool.UpdateAssemblies()
    scene.export(str(OUT/'junta nuevo de 8 motores con camara.glb'))
    result={'estado':'conjunto_neutral_recompuesto_con_interfaz_heredada_en_revision',
        'piezas':report,'origen_global_resta_mm':origin.tolist(),
        'colocacion_uav_nuevo_mm':placement.tolist(),
        'pivot_global_mm':pivot.tolist(),'pivot_modelo_m':((pivot-origin)*.001).tolist(),
        'recorrido_cad_nominal_contacto_mm':nominal_stroke,
        'margen_numerico_demostracion_mm':numerical_margin,'recorrido_simulacion_m':stroke,
        'referencia_elevacion':'q=0 es la posición guardada en CAD, no un tope inferior identificado. Solo se usa subida desde esa pose.',
        'decision_usuario_Base_ipt':'El usuario confirmó el 30 de octubre de 2026 que Base.ipt se conserva en el montaje nuevo.',
        'centros_empuje_modelo_m':{str(k):((v-origin)*.001).tolist() for k,v in motor_positions.items()},
        'ramas_retiradas':removed,'uniones_nativas_Fusion_recuperadas':False,
        'asignacion_rotula':{'trasladan':[0,1,2,6],'giran':[3,4,5]},
        'limitaciones':['Interfaz original conservada; hay solapes heredados con la rotula. Ver union_interferencias.json.',
            'Numeracion de motor tomada del CAD, no validada contra GPIO.',
            'La bateria CAD 2200 mAh es una referencia antigua; hardware declarado: dos 3300 mAh.',
            'Bola y bandeja rigidas en esta etapa; sin vision ni entrenamiento.',
            'Recorrido vertical inferido de topes y geometria; STEP no incluye restricciones nativas.']}
    (OUT/'ensamblaje_completo.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print('Escribiendo STEP completo...',flush=True)
    writer=STEPCAFControl_Writer();writer.SetNameMode(True);assert writer.Transfer(doc)
    target=OUT/'junta nuevo de 8 motores con camara.step'
    assert writer.Write(str(target))==IFSelect_RetDone
    print('Listo: '+str(target),flush=True)


if __name__=='__main__':main()
