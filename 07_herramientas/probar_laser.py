#!/usr/bin/env python3
"""Registro del diagnostico VL53L0X independiente, sin PWM ni MPU6050."""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import statistics
import time
from enlace_diagnostico_dsm import DiagnosticoDSM


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--segundos',type=float,default=10)
    p.add_argument('--distancia-mm',type=float,help='Distancia real desde la ventana del sensor a un blanco plano')
    p.add_argument('--iniciar',action='store_true',help='Inicializa el VL53L0X; sin esta opcion solo lee estado')
    a=p.parse_args()
    if not 1<=a.segundos<=120 or a.distancia_mm is not None and not 1<=a.distancia_mm<=2000:
        p.error('segundos 1..120 y distancia-mm 1..2000')
    base=Path(__file__).resolve().parent.parent/'registros'/'laser';base.mkdir(parents=True,exist_ok=True)
    nombre=datetime.now().strftime('laser_%Y%m%d_%H%M%S')
    c=DiagnosticoDSM(a.port,'dsm-laser-0.1')
    muestras=[];vistos=set()
    try:
        if a.iniciar:c.enviar('iniciar_laser')
        c.enviar('probar')
        fin=time.monotonic()+a.segundos
        while time.monotonic()<fin:
            r=c.leer()
            if r and r['seq'] not in vistos and r['seq']>0:
                vistos.add(r['seq']);muestras.append(r)
    finally:
        try:c.enviar('pausa')
        finally:c.close()
    validas=[r['mm'] for r in muestras if r['valida'] and r['range_status']==0]
    resultado={'version':'dsm-laser-0.1','solicito_inicializacion':a.iniciar,'muestras_nuevas':len(muestras),'validas':len(validas),
               'mediana_mm':statistics.median(validas) if validas else None,
               'min_mm':min(validas) if validas else None,'max_mm':max(validas) if validas else None,
               'desviacion_mm':statistics.pstdev(validas) if validas else None,
               'distancia_real_mm':a.distancia_mm,'motores':'sin_PWM'}
    if validas and a.distancia_mm is not None:
        resultado['error_mediana_mm']=statistics.median(validas)-a.distancia_mm
    (base/f'{nombre}.json').write_text(json.dumps({'resumen':resultado,'muestras':muestras},indent=2)+'\n')
    if muestras:
        with (base/f'{nombre}.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(muestras[0]));w.writeheader();w.writerows(muestras)
    print(json.dumps(resultado,indent=2,ensure_ascii=False))
    print('Registro:',base/f'{nombre}.json')
    if not muestras:print('No hubo medidas nuevas. Revisar inicializacion, modelo, I2C y alimentacion; no es una medicion de altura valida.')


if __name__=='__main__':main()
