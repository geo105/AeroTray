"""Configuración del panel; la escala solo se guarda con medidas entre centros."""
import json
from pathlib import Path
from vision_ball_plate import argumentos

RUTA = Path(__file__).resolve().parents[1] / 'estado_camara' / 'calibracion_panel.json'
DEFECTO = dict(modo='metal', contraste=25, ventana_local=41, area_min=100.,
               area_max=1200., circularidad=.55, auto_contraste=True,
               ancho_cm=None, alto_cm=None)


def validar(config):
    c = {**DEFECTO, **config}
    desconocidos = set(c) - set(DEFECTO)
    if desconocidos:
        raise ValueError(f'Parámetros desconocidos: {sorted(desconocidos)}')
    # Reutilizar las mismas restricciones del visor sin terminar el proceso UI.
    argv = []
    for key, value in c.items():
        if key == 'auto_contraste':
            if not isinstance(value, bool):
                raise ValueError('Auto contraste debe ser verdadero o falso')
            if value:
                argv.append('--auto-contraste')
        elif value is not None:
            argv += ['--' + key.replace('_', '-'), str(value)]
    try:
        args = argumentos(argv)
    except SystemExit as e:
        raise ValueError('Revisa contraste, áreas, circularidad y ambas distancias entre centros') from e
    return {k: getattr(args, k) for k in DEFECTO}


def cargar(ruta=RUTA):
    return validar(json.loads(ruta.read_text()) if ruta.exists() else {})


def guardar(config, ruta=RUTA):
    c = validar(config)
    ruta.parent.mkdir(parents=True, exist_ok=True)
    temporal = ruta.with_suffix('.tmp')
    temporal.write_text(json.dumps(c, indent=2, ensure_ascii=False) + '\n')
    temporal.replace(ruta)
    return c


def crear_args(config):
    args = argumentos([])
    for k, v in validar(config).items():
        setattr(args, k, v)
    args.memoria_aruco = True
    return args
