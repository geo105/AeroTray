#!/usr/bin/env python3
"""Compara recepcion USB y deteccion en la misma serie; nunca mueve servos."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import statistics
import time

import cv2
from vision_ball_plate import FuenteUSB, argumentos, crear_detector, procesar, panel
from calibracion_ball_plate import cargar, crear_args


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--baudrate', type=int, default=921600, choices=(115200, 460800, 921600))
    p.add_argument('--frames', type=int, default=50)
    p.add_argument('--salida', type=Path, required=True)
    p.add_argument('--guardar-frames', action='store_true', help='Conservar PNG de cada imagen para diagnostico')
    a = p.parse_args()
    if not 1 <= a.frames <= 1000:
        p.error('frames debe estar entre 1 y 1000')
    a.salida.mkdir(parents=True, exist_ok=True)
    src = FuenteUSB(a.port, a.baudrate)
    det = crear_detector()
    config = cargar()
    comunes = ['--area-min', '100', '--area-max', '1200', '--sin-ventana']
    if config['ancho_cm'] is not None:
        comunes += ['--ancho-cm', str(config['ancho_cm']), '--alto-cm', str(config['alto_cm'])]
    perfiles = {'brillo_95': argumentos(['--modo', 'clara', '--umbral', '95', *comunes]),
                'contraste_local': argumentos(['--modo', 'metal', *comunes]),
                'calibrado': crear_args(config)}
    perfiles['calibrado'].memoria_aruco = False
    muestras = []
    comienzo = time.monotonic()
    try:
        for i in range(a.frames):
            r = {'n': i+1}
            t = time.monotonic()
            try:
                im = src.read()
                r['usb_ms'] = (time.monotonic()-t)*1000
                if a.guardar_frames:
                    cv2.imwrite(str(a.salida/f'frame_{i+1:03d}.png'), im)
                for nombre, args in perfiles.items():
                    t_proceso = time.monotonic()
                    foto, mask, estado = procesar(im, det, args)
                    r[nombre] = {**estado, 'proceso_ms': (time.monotonic()-t_proceso)*1000}
                    if nombre == 'contraste_local' and (i % 10 == 0 or i == a.frames-1):
                        cv2.imwrite(str(a.salida/'actual.jpg'), panel(foto, estado, r['usb_ms']/1000, 0))
                        cv2.imwrite(str(a.salida/'original.jpg'), im)
                        cv2.imwrite(str(a.salida/'mascara.png'), mask)
            except (RuntimeError, OSError, ValueError) as e:
                r['error'] = str(e)
                print(f'# Foto {i+1}: {e}', flush=True)
            muestras.append(r)
            if (i+1) % 10 == 0:
                print(f'# {i+1}/{a.frames} fotos', flush=True)
    finally:
        fin = time.monotonic()
        velocidad_real = src.port.baudrate
        src.close()
    validas = [r for r in muestras if 'error' not in r]
    ms = [r['usb_ms'] for r in validas]
    resumen = {'fecha_local': datetime.now().astimezone().isoformat(), 'baudrate': velocidad_real,
               'solicitadas': a.frames, 'jpeg_validos': len(validas), 'errores': a.frames-len(validas),
               'usb_mediana_ms': statistics.median(ms) if ms else None,
               'fps_ciclo_total': len(validas)/(fin-comienzo),
               'nota': 'FPS incluyen tres detectores por cuadro. USB mide solicitud-respuesta, no latencia total de control.'}
    for nombre in perfiles:
        geom = [r[nombre] for r in validas if r[nombre]['geometria_valida']]
        resumen[nombre] = {'cuatro_esquinas': len(geom), 'pelota_detectada': sum(x['bola'] is not None for x in geom),
                          'proceso_mediana_ms': statistics.median([r[nombre]['proceso_ms'] for r in validas]) if validas else None}
    resultado = {'resumen': resumen, 'muestras': muestras}
    nombre = f"usb_{a.baudrate}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    (a.salida/nombre).write_text(json.dumps(resultado, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(resumen, indent=2, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
