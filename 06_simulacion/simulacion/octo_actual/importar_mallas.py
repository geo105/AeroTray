"""Convierte los STL originales CAD a FLU local del pivote.

Los centros de datos_cad.json ya son locales, pero los STL del ZIP están en
coordenadas CAD absolutas. Transformar vértices y normales explícitamente.
"""
from collections import defaultdict
from zipfile import ZipFile
import json
import struct
import numpy as np
from control import ROOT, CFG


def main():
    cad=json.loads((ROOT/'datos/cad.json').read_text()); R=np.array(cad['rotacionCADaFLU'])
    pivot=np.array(cad['pivotCAD']); groups=defaultdict(list)
    dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attr','<u2')])
    with ZipFile(CFG['fuentes']['zip']) as archive:
        for piece in cad['piezas']:
            if piece['indice']==370: continue  # bola CAD rígida de referencia
            group='placa_movil' if piece['indice'] in [349,350] else piece['cuerpo']
            raw=archive.read('Octo_DSM/matlab/cad/'+piece['archivo'])
            triangles=np.frombuffer(raw,dtype=dtype,offset=84).copy()
            vertices=(triangles['vertices']-pivot)@R.T
            if group=='placa_movil': vertices[:,:,2]-=CFG['ball_plate']['altura_placa_m']
            triangles['vertices']=vertices; triangles['normal']=triangles['normal']@R.T
            # Verifica las unidades y el cambio de marco contra los límites del JSON.
            lo,hi=np.array(piece['limites'])
            unshifted=vertices.copy()
            if group=='placa_movil': unshifted[:,:,2]+=CFG['ball_plate']['altura_placa_m']
            assert np.allclose(unshifted.reshape(-1,3).min(0),lo,atol=2e-6),piece['nombre']
            assert np.allclose(unshifted.reshape(-1,3).max(0),hi,atol=2e-6),piece['nombre']
            groups[group].append(triangles.tobytes())
    for name,chunks in groups.items():
        raw=b''.join(chunks)
        (ROOT/'mallas'/f'{name}.stl').write_bytes(b'DSM FLU local del pivote'.ljust(80,b'\0')+
            struct.pack('<I',len(raw)//50)+raw)
    print('Mallas FLU verificadas contra límites CAD:',len(cad['piezas'])-1,'piezas')


if __name__=='__main__': main()
