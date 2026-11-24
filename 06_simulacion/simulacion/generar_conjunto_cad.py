"""Guía y rótula reconstruidas sobre el conjunto CAD completo.

Geometría y ejes vienen del CAD. Masas por cuerpo y motores son provisionales.
"""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parent
CAD=ROOT.parent/'cad/junta_nuevo_de_8_motores_con_camara/conjunto_completo'
OUT=ROOT/'mallas_conjunto_cad'


def values(vector):return ' '.join(f'{x:.10g}' for x in vector)


def main():
    meta=json.loads((CAD/'ensamblaje_completo.json').read_text())
    OUT.mkdir(exist_ok=True)
    pivot=np.array(meta['pivot_modelo_m'])
    clusters={}
    for part in meta['piezas']:
        key=(part['cuerpo'],tuple(part['color']))
        mesh=trimesh.load_mesh(CAD/part['malla'],process=False)
        if part['cuerpo']!='carrito_fijo':mesh.apply_translation(-pivot)
        clusters.setdefault(key,[]).append(mesh)
    doc=ET.Element('mujoco',model='AeroTray_conjunto_CAD_DINAMICA_PROVISIONAL')
    ET.SubElement(doc,'compiler',angle='radian',inertiafromgeom='false')
    ET.SubElement(doc,'option',timestep='.002',gravity='0 0 -9.80665',integrator='implicitfast')
    defaults=ET.SubElement(doc,'default')
    ET.SubElement(defaults,'geom',contype='0',conaffinity='0',density='0')
    visual=ET.SubElement(doc,'visual')
    ET.SubElement(visual,'global',offwidth='1200',offheight='900')
    ET.SubElement(visual,'headlight',ambient='.4 .4 .4',diffuse='.6 .6 .6',specular='.1 .1 .1')
    asset=ET.SubElement(doc,'asset')
    ET.SubElement(asset,'texture',name='fondo',type='skybox',builtin='gradient',
                  rgb1='.90 .94 .98',rgb2='.65 .74 .84',width='256',height='1536')
    world=ET.SubElement(doc,'worldbody')
    ET.SubElement(world,'light',pos='1 -1 2',dir='-.3 .3 -1',diffuse='.6 .6 .6')
    ET.SubElement(world,'geom',name='suelo_visual',type='plane',size='2 2 .1',rgba='.76 .79 .82 1')
    ET.SubElement(world,'camera',name='vista',pos='1.3 -1.5 1.15',xyaxes='.756 .655 0 -.20 .23 .952')
    carriage=ET.SubElement(world,'body',name='carro_vertical',pos=values(pivot))
    ET.SubElement(carriage,'joint',name='elevacion',type='slide',axis='0 0 1',limited='true',
                  range=values([0,meta['recorrido_simulacion_m']]),damping='.5',solreflimit='.005 1')
    ET.SubElement(carriage,'inertial',mass='.3',pos='0 0 -.1',diaginertia='.002 .002 .002')
    upper=ET.SubElement(carriage,'body',name='conjunto_rotatorio')
    ET.SubElement(upper,'joint',name='rotula',type='ball',damping='.02')
    ET.SubElement(upper,'inertial',mass='3.3',pos='0 0 .1',diaginertia='.12 .12 .20')
    parents={'carrito_fijo':world,'carro_vertical':carriage,'conjunto_rotatorio':upper}
    mesh_count=0
    for (group,rgba),meshes in clusters.items():
        # El lector STL de MuJoCo limita cada asset a 200000 caras.
        chunks=[[]];faces=0
        for mesh in meshes:
            if faces+len(mesh.faces)>150000 and chunks[-1]:
                chunks.append([]);faces=0
            chunks[-1].append(mesh);faces+=len(mesh.faces)
        for chunk in chunks:
            mesh=trimesh.util.concatenate(chunk);assert len(mesh.faces)<200000
            target=OUT/f'{mesh_count:02d}_{group}.stl';mesh.export(target)
            name=f'cad_{mesh_count:02d}_{group}'
            ET.SubElement(asset,'mesh',name=name,file=str(target),inertia='shell')
            ET.SubElement(parents[group],'geom',name=name,type='mesh',mesh=name,rgba=values(rgba),group='2')
            mesh_count+=1
    actuators=ET.SubElement(doc,'actuator')
    matrix=[]
    for number in range(1,9):
        pos=np.array(meta['centros_empuje_modelo_m'][str(number)])-pivot
        ET.SubElement(upper,'site',name=f'empuje_M{number}',pos=values(pos),size='.004',rgba='1 .8 .1 1')
        yaw=.015 if number<=4 else -.015
        ET.SubElement(actuators,'motor',name=f'fuerza_M{number}',site=f'empuje_M{number}',
                      gear=f'0 0 1 0 0 {yaw}',ctrllimited='true',ctrlrange='0 6')
        matrix.append([1,pos[1],-pos[0],yaw])
    allocation=np.array(matrix).T
    trim=np.linalg.lstsq(allocation,np.array([3.6*9.80665,0,0,0]),rcond=None)[0]
    assert np.all((trim>0)&(trim<6))
    ET.indent(doc);ET.ElementTree(doc).write(ROOT/'aerotray_conjunto_cad.xml',encoding='utf-8',xml_declaration=True)
    parameters={'estado':'geometria_CAD_y_uniones_de_simulacion_reconstruidas_dinamica_provisional',
        'masa_total_kg':3.6,'masas_por_cuerpo_provisionales_kg':{'carro_vertical':.3,'conjunto_rotatorio':3.3},
        'centro_rotula_modelo_m':pivot.tolist(),'recorrido_cad_contacto_mm':meta['recorrido_cad_nominal_contacto_mm'],
        'recorrido_simulacion_m':meta['recorrido_simulacion_m'],'centros_empuje_modelo_m':meta['centros_empuje_modelo_m'],
        'referencia_elevacion':meta['referencia_elevacion'],
        'trim_empuje_ideal_N':trim.tolist(),'limite_empuje_ficticio_N':6,'relacion_par_empuje_ficticia_m':.015,
        'geometrias_visuales':mesh_count,'componentes_CAD':len(meta['piezas']),
        'precedencia':'Usuario pide priorizar CAD para dimensiones y recorrido; se sustituye el dato anterior de 0.20 m.',
        'limites':['Interfaz heredada con solapes; no se simulan colisiones de CAD.',
            'El reparto de masa, inercias, COM y amortiguamiento no se han identificado.',
            'El STEP no contiene articulaciones Fusion; asignación de sólidos interpretada de geometría.',
            'Bola y servos rígidos; sin visión ni entrenamiento.',
            'No hay límites angulares identificados; rótula esférica ideal.',
            'Cart fijo; la etapa móvil la trabaja el compañero.']}
    (ROOT/'supuestos_conjunto_cad.json').write_text(json.dumps(parameters,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in parameters.items() if k not in ('centros_empuje_modelo_m','limites')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
