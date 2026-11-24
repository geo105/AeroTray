"""Genera Gazebo SDF y MJCF/Isaac a partir de la misma configuración."""
from pathlib import Path
import xml.etree.ElementTree as E
import argparse
from control import CFG, ROOT


def add(parent, tag, text=None, **attrs):
    node = E.SubElement(parent, tag, {k:str(v) for k,v in attrs.items()})
    if text is not None: node.text = str(text)
    return node


def write(root, path):
    E.indent(root); E.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def visual(link, group, sdf=True):
    # importar_mallas.py convirtió los STL CAD absolutos a FLU local del pivote.
    v = add(link, 'visual' if sdf else 'geom', name=group)
    if sdf:
        geom = add(v, 'geometry'); mesh = add(geom, 'mesh')
        add(mesh, 'uri', (ROOT/'mallas'/f'{group}.stl').as_uri())
        mat = add(v, 'material'); add(mat, 'ambient', '.35 .48 .62 1'); add(mat, 'diffuse', '.35 .48 .62 1')
    else:
        v.set('type','mesh'); v.set('mesh',group); v.set('rgba','.35 .48 .62 1')


def inertial(link, mass, com, inertia):
    inert = add(link, 'inertial'); add(inert, 'mass', mass)
    add(inert, 'pose', ' '.join(map(str,com))+' 0 0 0')
    tensor = add(inert, 'inertia')
    for tag,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
        add(tensor, tag, inertia[i][j])


def gazebo(mode):
    initial=CFG['altura']['altura_inicial_ideal_m'] if mode=='altura_ideal' else 0.
    root = E.Element('sdf', version='1.9'); world = add(root,'world',name='octo_actual')
    add(world,'gravity','0 0 -9.81')
    p = add(world,'physics',name='paso',type='ignored'); add(p,'max_step_size',.001); add(p,'real_time_factor',1)
    for filename,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')]:
        add(world,'plugin',filename=f'gz-sim-{filename}-system',name=f'gz::sim::systems::{name}')
    light=add(world,'light',name='sol',type='directional'); add(light,'pose','0 0 5 0 0 0'); add(light,'direction','-.4 .2 -1')
    add(light,'diffuse','.9 .9 .9 1'); add(light,'cast_shadows','true')
    model=add(world,'model',name='octo'); add(model,'self_collide','false')
    base=add(model,'link',name='base'); add(base,'pose',f'0 0 {CFG["pivot_altura_m"]} 0 0 0')
    inertial(base, 1., [0,0,0], [[.1,0,0],[0,.1,0],[0,0,.1]])
    visual(base,'carrito_fijo')
    fixed=add(model,'joint',name='anclaje',type='fixed'); add(fixed,'parent','world'); add(fixed,'child','base')
    carriage=add(model,'link',name='carro'); add(carriage,'pose',f'0 0 {CFG["pivot_altura_m"]+initial} 0 0 0')
    inertial(carriage,CFG['masa_carro_vertical_kg'],[0,0,-.1],[[.002,0,0],[0,.002,0],[0,0,.002]])
    visual(carriage,'carro_vertical')
    guide=add(model,'joint',name='elevacion',type='fixed' if mode=='banco' else 'prismatic')
    add(guide,'parent','base'); add(guide,'child','carro')
    if mode!='banco':
        axis=add(guide,'axis'); add(axis,'xyz','0 0 1')
        lim=add(axis,'limit'); add(lim,'lower',-initial); add(lim,'upper',CFG['recorrido_m']-initial)
        dyn=add(axis,'dynamics'); add(dyn,'damping',.5)
    body=add(model,'link',name='dron'); add(body,'pose',f'0 0 {CFG["pivot_altura_m"]+initial} 0 0 0')
    inertial(body,CFG['masa_rotatoria_kg'],CFG['cg_m'],CFG['inercia_cg']); visual(body,'conjunto_rotatorio')
    ball=add(model,'joint',name='rotula',type='ball'); add(ball,'parent','carro'); add(ball,'child','dron')
    plugin=add(model,'plugin',filename='gz-sim-python-system-loader-system',name='gz::sim::systems::PythonSystemLoader')
    add(plugin,'module_name','sistema_gazebo'); add(plugin,'mode',mode)
    # Suelo visual: la intersección ToF con z=0 se calcula sin renderizador GPU.
    ground=add(world,'model',name='suelo'); add(ground,'static','true'); link=add(ground,'link',name='suelo')
    v=add(link,'visual',name='suelo'); geo=add(v,'geometry'); plane=add(geo,'plane'); add(plane,'normal','0 0 1'); add(plane,'size','3 3')
    plate=add(world,'model',name='placa_visual'); add(plate,'static','true')
    add(plate,'pose',f'0 0 {CFG["pivot_altura_m"]+CFG["ball_plate"]["altura_placa_m"]} 0 0 0')
    plink=add(plate,'link',name='placa'); visual(plink,'placa_movil')
    ballmodel=add(world,'model',name='bola_visual'); add(ballmodel,'static','true')
    add(ballmodel,'pose','0 0 1 0 0 0'); bl=add(ballmodel,'link',name='bola')
    bv=add(bl,'visual',name='bola'); bg=add(bv,'geometry'); sphere=add(bg,'sphere')
    add(sphere,'radius',CFG['ball_plate']['diametro_bola_m']/2)
    bm=add(bv,'material'); add(bm,'ambient','.1 .3 1 1'); add(bm,'diffuse','.1 .3 1 1')
    write(root,ROOT/f'{mode}.sdf')


def mjcf():
    root=E.Element('mujoco',model='DSM_control_actual_banco')
    add(root,'compiler',angle='radian',inertiafromgeom='false')
    add(root,'option',timestep='.001',gravity='0 0 -9.81',integrator='implicitfast')
    default=add(root,'default'); add(default,'geom',contype='0',conaffinity='0',density='0')
    asset=add(root,'asset')
    for name in ['carrito_fijo','carro_vertical','conjunto_rotatorio','placa_movil']:
        add(asset,'mesh',name=name,file=str(ROOT/'mallas'/f'{name}.stl'),inertia='shell')
    world=add(root,'worldbody')
    base=add(world,'body',name='base',pos=f'0 0 {CFG["pivot_altura_m"]}')
    visual(base,'carrito_fijo',False); visual(base,'carro_vertical',False)
    body=add(base,'body',name='dron'); add(body,'joint',name='rotula',type='ball',damping='.035')
    I=CFG['inercia_cg']; full=[I[0][0],I[1][1],I[2][2],I[0][1],I[0][2],I[1][2]]
    add(body,'inertial',mass=CFG['masa_rotatoria_kg'],pos=' '.join(map(str,CFG['cg_m'])),fullinertia=' '.join(map(str,full)))
    visual(body,'conjunto_rotatorio',False)
    plate=add(body,'body',name='placa_visual',pos=f'0 0 {CFG["ball_plate"]["altura_placa_m"]}')
    visual(plate,'placa_movil',False)
    write(root,ROOT/'banco.xml')


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--isaac',action='store_true'); args=parser.parse_args()
    for mode in ['banco','altura','altura_ideal']: gazebo(mode)
    mjcf()
    if args.isaac:
        from mujoco_usd_converter import Converter
        result=Converter(layer_structure=False,scene=False).convert(str(ROOT/'banco.xml'),str(ROOT/'usd'))
        (ROOT/'usd_path.txt').write_text(result.path+'\n'); print(result.path)
    print('Modelos preparados en', ROOT)


if __name__=='__main__': main()
