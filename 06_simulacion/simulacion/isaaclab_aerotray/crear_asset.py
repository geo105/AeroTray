"""Preparar el MJCF que importará Isaac Lab, con CAD y física explícita."""
from pathlib import Path
import xml.etree.ElementTree as ET
import json

ROOT=Path(__file__).resolve().parents[1]


def main():
    target=ROOT/'isaaclab_aerotray/assets'
    target.mkdir(parents=True,exist_ok=True)
    tree=ET.parse(ROOT/'aerotray_conjunto_cad.xml')
    doc=tree.getroot();doc.set('model','AeroTray_CAD_IsaacLab_PROVISIONAL')
    # Los ocho empujes se aplican explícitamente en la tarea como wrench.
    for tag in ('actuator','visual'):
        node=doc.find(tag)
        if node is not None:doc.remove(node)
    world=doc.find('worldbody')
    for node in list(world):
        if node.tag in ('camera','light') or node.get('name')=='suelo_visual':world.remove(node)
    asset=doc.find('asset')
    for node in list(asset):
        if node.tag=='texture':asset.remove(node)
    ET.indent(tree);tree.write(target/'aerotray.xml',encoding='utf-8',xml_declaration=True)
    # API oficial de conversión MJCF→USD sin requerir schemas de PhysX.
    # El backend de esta tarea es Newton, no Isaac Sim/PhysX.
    from mujoco_usd_converter import Converter
    asset=Converter(layer_structure=False,scene=False).convert(str(target/'aerotray.xml'),str(target/'usd_newton'))
    (target/'asset.json').write_text(json.dumps({'usd_path':asset.path},indent=2)+'\n')
    print(asset.path)


if __name__=='__main__':main()
