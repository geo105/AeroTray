#!/usr/bin/env python3
"""Evalúa imágenes originales del panel abierto sin ocupar USB ni mandar servos.

El panel guarda unas dos imágenes por segundo: esta prueba mide detección en
esas muestras, no los FPS ni la latencia del lazo de control.
Activar «Guardar fotos para diagnóstico» en Cámara o iniciar el panel con
--guardar-imagenes antes de ejecutar esta prueba.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time
import cv2
from calibracion_ball_plate import RUTA, cargar, crear_args
from vision_ball_plate import crear_detector, procesar

RAIZ=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frames',type=int,default=60)
    p.add_argument('--espera-max',type=float,default=90)
    p.add_argument('--etiqueta',choices=('pelota','vacia','sin_etiquetar'),default='sin_etiquetar')
    p.add_argument('--imagen',type=Path,default=RAIZ/'estado_camara/panel/original.jpg')
    p.add_argument('--calibracion',type=Path,default=RUTA)
    p.add_argument('--salida',type=Path,default=RAIZ/'estado_camara'/datetime.now().strftime('revision_%Y%m%d_%H%M%S'))
    a=p.parse_args()
    if not 1<=a.frames<=1000 or not 1<=a.espera_max<=600:p.error('Revisa frames (1–1000) y espera-max (1–600 s)')
    config=cargar(a.calibracion)
    perfiles={nombre:crear_args({**config,'auto_contraste':auto}) for nombre,auto in [('fijo',False),('adaptativo',True)]}
    for args in perfiles.values():args.memoria_aruco=False
    det=crear_detector();a.salida.mkdir(parents=True,exist_ok=True)
    anterior=None;muestras=[];limite=time.monotonic()+a.espera_max
    while len(muestras)<a.frames and time.monotonic()<limite:
        try:
            marca=a.imagen.stat().st_mtime_ns
            if marca==anterior or time.time()-marca/1e9>2:
                time.sleep(.05);continue
            im=cv2.imread(str(a.imagen))
            if im is None or a.imagen.stat().st_mtime_ns!=marca:
                time.sleep(.05);continue
        except OSError:
            time.sleep(.1);continue
        anterior=marca;n=len(muestras)+1
        cv2.imwrite(str(a.salida/f'frame_{n:03d}.png'),im)
        muestra={'n':n,'archivo_mtime_ns':marca}
        for nombre,args in perfiles.items():
            _,_,r=procesar(im,det,args);muestra[nombre]=r
        muestras.append(muestra)
        if n%10==0:print(f'{n}/{a.frames} imágenes nuevas',flush=True)
    resumen={'etiqueta':a.etiqueta,'imagenes':len(muestras),'solicitadas':a.frames,'configuracion':config,
             'nota':'Muestras del archivo del panel (~2 Hz), no FPS de cámara ni validación de control.'}
    for nombre in perfiles:
        n=sum(m[nombre]['bola'] is not None for m in muestras)
        resumen[nombre]={'cuatro_aruco_observados':sum(m[nombre]['geometria_observada'] for m in muestras),'detecciones':n}
        if a.etiqueta=='pelota':resumen[nombre]['omisiones']=len(muestras)-n
        elif a.etiqueta=='vacia':resumen[nombre]['falsos_positivos']=n
    (a.salida/'resultados.json').write_text(json.dumps({'resumen':resumen,'muestras':muestras},indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(resumen,indent=2,ensure_ascii=False),flush=True)
    if len(muestras)<a.frames:raise SystemExit('Serie incompleta: deja abierto el panel actualizado y revisa original.jpg')


if __name__=='__main__':main()
