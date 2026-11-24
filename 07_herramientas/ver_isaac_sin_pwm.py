#!/usr/bin/env python3
"""Muestra el cálculo PPO con sensores reales; no abre puertos ni envía comandos.

Las fuerzas están en orden CAD, sin correspondencia física verificada. Este
observador no valida estabilidad, la adaptación al banco ni el control de vuelo.
"""
import argparse
import csv
import ctypes
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT/'firmware/dron_octo_esp32/politica_ppo_sim.h'
META = ROOT/'firmware/dron_octo_esp32/politica_ppo_sim.json'


def ultima_medida():
    archivos = list((ROOT/'registros').glob('identificacion_motores_*.jsonl'))
    if not archivos:
        return None
    archivo = max(archivos, key=lambda p:p.stat().st_mtime)
    with archivo.open('rb') as f:
        f.seek(max(0, archivo.stat().st_size-20000))
        lineas = f.read().decode(errors='replace').splitlines()
    for linea in reversed(lineas):
        try:
            r = json.loads(linea)
        except ValueError:
            continue
        if r.get('tipo') == 'diagnostico':
            d = r['dato']
            claves = ('z_cm','pitch','roll','yaw','gx','gy','gz','ms')
            if not (0 <= time.time()-r['t_pc_s'] <= .25 and d.get('imu_ok') and d.get('z_valida')):
                return None
            if not all(type(d.get(k)) in (int,float) and math.isfinite(d[k]) for k in claves):
                return None
            return d
    return None


def rotacion(d, yaw_inicial):
    r,p,y = -math.radians(d['roll']),-math.radians(d['pitch']),math.radians(d['yaw']-yaw_inicial)
    cr,sr,cp,sp,cy,sy = math.cos(r/2),math.sin(r/2),math.cos(p/2),math.sin(p/2),math.cos(y/2),math.sin(y/2)
    q = [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]
    if q[0] < 0:
        q = [-x for x in q]
    n = math.sqrt(sum(x*x for x in q[1:]))
    factor = 2*math.atan2(n,q[0])/n if n > 1e-7 else 2.
    return [x*factor for x in q[1:]]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--segundos',type=float,default=15)
    p.add_argument('--referencia-cm',type=float,default=3.831472969)
    p.add_argument('--rampa-cm-s',type=float,default=.5)
    a = p.parse_args()
    if not all(math.isfinite(x) for x in (a.segundos,a.referencia_cm,a.rampa_cm_s)):
        p.error('Parámetros finitos requeridos')
    if not (1 <= a.segundos <= 60 and 1 <= a.referencia_cm <= 6.7 and .05 <= a.rampa_cm_s <= 5):
        p.error('Duración 1..60 s; referencia del modelo 1..6.7 cm; rampa 0.05..5 cm/s')
    meta = json.loads(META.read_text())
    if hashlib.sha256(HEADER.read_bytes()).hexdigest() != meta['header_sha256']:
        raise RuntimeError('La cabecera PPO no coincide con la exportación verificada')

    import rclpy
    from std_msgs.msg import Float32MultiArray
    from visualization_msgs.msg import Marker,MarkerArray
    # Este proceso publica solamente resultados y marcadores; no crea ninguna
    # publicación de comandos ni establece conexión serie/UDP con el ESP32.
    rclpy.init()
    node = rclpy.create_node('dsm_isaac_solo_calculo')
    pub = node.create_publisher(Float32MultiArray,'/dron/isaac_fuerzas_cad_N',10)
    visual = node.create_publisher(MarkerArray,'/dron/marcadores',10)
    nombre = datetime.now().strftime('isaac_sin_pwm_%Y%m%d_%H%M%S_%f')
    salida = ROOT/'registros'/nombre
    validas = total = 0
    try:
        with tempfile.TemporaryDirectory(prefix='dsm_ppo_sin_pwm_') as tmp:
            tmp = Path(tmp)
            (tmp/'infer.cpp').write_text('#include '+json.dumps(str(HEADER))+'\n'
                'extern "C" void paso(float z,float v,const float* r,const float* w,float ref,float* prev,float* out) {'
                'dsm_ppo::paso_red(z,v,r,w,ref,prev,out);}\n')
            subprocess.run(['g++','-std=c++17','-O2','-shared','-fPIC',str(tmp/'infer.cpp'),'-o',str(tmp/'infer.so')],check=True)
            lib = ctypes.CDLL(str(tmp/'infer.so'))
            ptr = ctypes.POINTER(ctypes.c_float)
            lib.paso.argtypes = [ctypes.c_float,ctypes.c_float,ptr,ptr,ctypes.c_float,ptr,ptr]
            lib.paso.restype = None
            prev = (ctypes.c_float*8)()
            anterior = None
            referencia = yaw0 = None
            velocidad = 0.
            inicio = siguiente = time.monotonic()
            with salida.with_suffix('.csv').open('w',newline='') as f:
                csvout = csv.writer(f)
                csvout.writerow(['t_s','datos_validos','z_cm','referencia_modelo_cm','v_estimado_m_s']+[f'CAD{i}_N' for i in range(1,9)])
                print('PPO original, 50 Hz, SIN PWM. Fuerzas en orden CAD; mapa y dinámica del banco NO verificados.',flush=True)
                print('CSV:',salida.with_suffix('.csv'),flush=True)
                while time.monotonic()-inicio < a.segundos:
                    ahora = time.monotonic()
                    d = ultima_medida()
                    fuerzas = []
                    if d is not None:
                        if referencia is None:
                            referencia = d['z_cm']*.01
                            yaw0 = d['yaw']
                        if anterior and d['ms'] != anterior['ms']:
                            dt = (d['ms']-anterior['ms'])*.001
                            if 0 < dt <= .25:
                                velocidad += .3*((d['z_cm']-anterior['z_cm'])*.01/dt-velocidad)
                            else:
                                velocidad = 0.
                        anterior = d
                        delta = a.referencia_cm*.01-referencia
                        referencia += max(-a.rampa_cm_s*.01*.02,min(a.rampa_cm_s*.01*.02,delta))
                        r = (ctypes.c_float*3)(*rotacion(d,yaw0))
                        w = (ctypes.c_float*3)(-math.radians(d['gx']),-math.radians(d['gy']),math.radians(d['gz']))
                        out = (ctypes.c_float*8)()
                        lib.paso(d['z_cm']*.01,velocidad,r,w,referencia,prev,out)
                        fuerzas = list(out)
                        validas += 1
                    else:
                        prev = (ctypes.c_float*8)()
                        anterior = None
                        referencia = yaw0 = None
                        velocidad = 0.
                    total += 1
                    csvout.writerow([round(ahora-inicio,4),int(d is not None),d['z_cm'] if d else '',referencia*100 if referencia is not None else '',velocidad if d else '']+fuerzas)
                    pub.publish(Float32MultiArray(data=fuerzas))
                    m = Marker(type=Marker.TEXT_VIEW_FACING,ns='isaac_sin_pwm',id=0,action=Marker.ADD)
                    m.header.frame_id = 'mundo'
                    m.header.stamp = node.get_clock().now().to_msg()
                    m.pose.orientation.w = 1.
                    m.pose.position.z = .55
                    m.scale.z = .025
                    m.color.r,m.color.g,m.color.b,m.color.a = .2,1.,1.,1.
                    m.lifetime.sec = 2
                    m.text = 'ISAAC_PPO/SOLO_CALCULO/SIN_PWM\n'
                    if fuerzas:
                        m.text += f'Ref_modelo:{referencia*100:.2f}cm\nCAD1..4[N]:'+','.join(f'{x:.2f}' for x in fuerzas[:4])
                        m.text += '\nCAD5..8[N]:'+','.join(f'{x:.2f}' for x in fuerzas[4:])
                    else:
                        m.text += 'Esperando_IMU_y_laser_validos'
                    m.text += '\nMapa_CAD_y_banco:NO_VERIFICADOS'
                    visual.publish(MarkerArray(markers=[m]))
                    rclpy.spin_once(node,timeout_sec=0)
                    siguiente += .02
                    time.sleep(max(0,siguiente-time.monotonic()))
        reporte = {'checkpoint':meta['checkpoint'],'checkpoint_sha256':meta['checkpoint_sha256'],
                   'header_sha256':meta['header_sha256'],'duracion_s':a.segundos,'referencia_modelo_cm':a.referencia_cm,
                   'rampa_referencia_cm_s':a.rampa_cm_s,'muestras_validas':validas,'muestras_totales':total,
                   'salidas_motor':False,'valida_estabilidad_fisica':False,'mapa_marco_y_motores_verificados':False,
                   'velocidad_vertical':'estimada por diferencia y filtro 0.3; no sensor de velocidad identificado',
                   'nota':'Cálculo con la masa/guía del modelo original. Las fuerzas no se convierten en PWM.'}
        salida.with_suffix('.json').write_text(json.dumps(reporte,ensure_ascii=False,indent=2)+'\n')
        print(f'Terminado SIN PWM: {validas}/{total} muestras con sensores válidos.',flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
