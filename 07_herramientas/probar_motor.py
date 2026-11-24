#!/usr/bin/env python3
"""Prueba UN motor a 1200 us por defecto, con latido y parada al terminar.

Ejecutar sin helices y con la alimentacion USB/buck correctamente separada.
El sentido se observa visualmente; no hay medicion de RPM ni de giro.
Sin --ejecutar solo lee el estado y muestra el plan, sin habilitar PWM.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import time
from enlace_diagnostico_dsm import DiagnosticoDSM

PINES=[25,26,27,14,21,19,5,17]


def main():
    p=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port',required=True)
    p.add_argument('--motor',type=int,choices=range(1,9),default=1)
    p.add_argument('--pwm',type=int,default=1200)
    p.add_argument('--segundos',type=float,default=2)
    p.add_argument('--ejecutar',action='store_true')
    p.add_argument('--sentido-observado',choices=('cw','ccw','no-gira','pendiente'),default='pendiente')
    p.add_argument('--sentido-m1',choices=('cw','ccw'),help='Referencia M1 observada desde arriba; no se adivina')
    a=p.parse_args()
    if not 1000<=a.pwm<=1950 or not 0.1<=a.segundos<=5:p.error('PWM 1000..1950 us, duracion 0.1..5 s')
    plan={'motor':a.motor,'gpio':PINES[a.motor-1],'pwm_us':a.pwm,'duracion_s':a.segundos,
          'respecto_a_M1':'mismo sentido' if a.motor<=4 else 'sentido opuesto',
          'sentido_observado':a.sentido_observado,'sentido_medido_por_sensor':False,'fecha':datetime.now().astimezone().isoformat()}
    if a.sentido_m1:
        esperado=a.sentido_m1 if a.motor<=4 else {'cw':'ccw','ccw':'cw'}[a.sentido_m1]
        plan['sentido_esperado_desde_arriba']=esperado
        plan['coincide_observacion']=a.sentido_observado==esperado if a.sentido_observado!='pendiente' else None
    print('Plan:',json.dumps(plan,ensure_ascii=False),flush=True)
    c=DiagnosticoDSM(a.port,'dsm-motores-0.3');ultimo_hb=0
    def esperar(predicado,segundos):
        nonlocal ultimo_hb
        fin=time.monotonic()+segundos
        while time.monotonic()<fin:
            if time.monotonic()-ultimo_hb>=0.2:c.enviar('hb');ultimo_hb=time.monotonic()
            r=c.leer()
            if r and predicado(r):return r
        raise RuntimeError('No se confirmo el estado esperado. Se envia parada y se desarma.')
    try:
        print('Estado:',json.dumps(c.estado),flush=True)
        if not a.ejecutar:
            print('Solo lectura. Para la prueba fisica, sin helices, usar --ejecutar.')
            return
        c.enviar('hb');c.enviar('armar')
        esperar(lambda r:r['estado']==2,5)
        c.enviar(f'motor {a.motor} {a.pwm} {round(a.segundos*1000)}')
        esperar(lambda r:r['estado']==3 and r['motor']==a.motor and r['pwm_us']==a.pwm,1.5)
        print(f'M{a.motor} en prueba a {a.pwm} us. Observar giro SIN helices.',flush=True)
        final=esperar(lambda r:r['estado']==2,a.segundos+2)
        plan['resultado']='pulso terminado por temporizador; giro pendiente de observacion humana'
        plan['estado_final_prueba']=final
    finally:
        try:
            if a.ejecutar:c.enviar('!');c.enviar('desarmar')
        finally:c.close()
    carpeta=Path(__file__).resolve().parent.parent/'registros'/'motores';carpeta.mkdir(parents=True,exist_ok=True)
    ruta=carpeta/datetime.now().strftime(f'motor_{a.motor}_%Y%m%d_%H%M%S.json')
    ruta.write_text(json.dumps(plan,indent=2,ensure_ascii=False)+'\n');print('Registro:',ruta)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('Prueba interrumpida; se solicita parada.')
    except (OSError,RuntimeError) as e:raise SystemExit(f'ERROR: {e}')
