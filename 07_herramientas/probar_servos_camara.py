#!/usr/bin/env python3
"""Prueba limitada 90/90 y +-2 grados, conservando fotos y estado de la S3.

GPIO41 entre IDs 0-3; GPIO42 entre IDs 2-3. Las posiciones son ordenes
de servo, no lecturas de encoder. Cada prueba termina centrando y apagando PWM.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time

import cv2
from vision_ball_plate import FuenteUSB,argumentos,crear_detector,procesar,panel


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--fase',choices=('centro','41','42','ambos'),default='centro')
    p.add_argument('--delta',type=float,default=2)
    p.add_argument('--duracion-ms',type=int,default=500)
    p.add_argument('--ejecutar',action='store_true')
    p.add_argument('--sin-ventana',action='store_true')
    a=p.parse_args()
    if not -2<=a.delta<=2 or not 100<=a.duracion_ms<=800:p.error('delta -2..2 y duracion-ms 100..800')
    src=FuenteUSB(a.port)
    out=Path(__file__).resolve().parent.parent/'estado_camara'/'prueba_servos'/datetime.now().strftime(f'{a.fase}_%Y%m%d_%H%M%S')
    out.mkdir(parents=True,exist_ok=True)
    args=argumentos(['--modo','metal','--area-min','100','--area-max','1200','--ancho-cm','18.7','--alto-cm','14'])
    det=crear_detector();medidas=[];n=0;ventana='DSM - Prueba servos 41 / 42';t0=time.monotonic()
    def cmd(texto):src.port.write((texto+'\n').encode())
    def orden(texto,confirmacion):
        cmd(texto);fin=time.monotonic()+1
        while time.monotonic()<fin:
            linea=src.port.readline().decode(errors='replace').strip()
            if linea.startswith('# ERROR_SERVO'):raise RuntimeError(linea)
            if linea.startswith(confirmacion):return
        raise RuntimeError(f'No se confirmo {texto}')
    def observar(etapa,segundos):
        nonlocal n
        fin=time.monotonic()+segundos
        errores=0
        while time.monotonic()<fin:
            cmd('hb');comienzo=time.monotonic()
            try:im=src.read()
            except (OSError,RuntimeError,ValueError) as e:
                errores+=1;print('Imagen descartada:',e,flush=True)
                if errores>=3:raise
                continue
            errores=0
            foto,mask,r=procesar(im,det,args);n+=1
            r.update({'etapa':etapa,'t_s':time.monotonic()-t0,'transferencia_ms':(time.monotonic()-comienzo)*1000})
            medidas.append(r)
            cv2.imwrite(str(out/f'original_{n:03d}.png'),im)
            vista=panel(foto,r,r['transferencia_ms']/1000,0)
            cv2.putText(vista,etapa,(15,55),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
            cv2.imwrite(str(out/'actual.jpg'),vista)
            if not a.sin_ventana:
                cv2.imshow(ventana,vista)
                if cv2.waitKey(1)&0xff in (ord('q'),27) or cv2.getWindowProperty(ventana,cv2.WND_PROP_VISIBLE)<1:raise KeyboardInterrupt
    try:
        estado=src.leer_estado(2)
        print('Estado:',json.dumps(estado),flush=True)
        if estado.get('version') not in ('camara-diag-0.4','camara-diag-0.5'):raise RuntimeError('Requiere diagnostico 0.4 o 0.5; no se envio ninguna orden de movimiento.')
        if not a.ejecutar:
            print('Solo estado. --ejecutar habilita la prueba fisica limitada.',flush=True);return
        if not a.sin_ventana:
            cv2.namedWindow(ventana,cv2.WINDOW_NORMAL);cv2.resizeWindow(ventana,960,800)
        cmd('hb');orden('servo_arm 90 90','# SERVO_ARM')
        observar('Centro: GPIO41=90 / GPIO42=90',1.2)
        estado=src.leer_estado(2)
        if not estado['servos']['pwm_activo'] or estado['servos']['error_hardware']:
            raise RuntimeError('Los canales PWM no quedaron activos o reportan error')
        pines=[] if a.fase=='centro' else [41,42] if a.fase=='ambos' else [int(a.fase)]
        for pin in pines:
            cmd('hb');orden(f'servo_test {pin} {a.delta:g} {a.duracion_ms}','# SERVO_TEST')
            observar(f'GPIO{pin}: {90+a.delta:g} grados, retorno automatico a 90',a.duracion_ms/1000+.35)
            observar('Ambos de vuelta a 90',.6)
        cmd('hb');cmd('servo_centro');observar('Final: ambos a 90 grados',.4)
        final=src.leer_estado(2)
        print('Final antes de desactivar:',json.dumps(final),flush=True)
    finally:
        try:
            if a.ejecutar:
                cmd('servo_centro');time.sleep(.15);cmd('servo_off');time.sleep(.08)
        finally:src.close()
        if not a.sin_ventana:cv2.destroyAllWindows()
        (out/'medidas.json').write_text(json.dumps(medidas,indent=2,ensure_ascii=False)+'\n')
        print('Registro:',out,flush=True)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('Prueba interrumpida; se solicito centro y PWM apagado.')
    except (OSError,RuntimeError) as e:raise SystemExit(f'ERROR: {e}')
