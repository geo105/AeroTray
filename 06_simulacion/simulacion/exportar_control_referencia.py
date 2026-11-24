"""Exporta geometria, limites y ganancias nominales del controlador simulado."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
xml=ROOT/'simulacion/aerotray_conjunto_cad.xml'
model=ET.parse(xml).getroot()
sites={x.attrib['name']:np.fromstring(x.attrib['pos'],sep=' ') for x in model.iter('site')}
motors=list(model.find('actuator'))
A=np.array([[1,sites[x.attrib['site']][1],-sites[x.attrib['site']][0],np.fromstring(x.attrib['gear'],sep=' ')[5]] for x in motors]).T
inverse=np.linalg.pinv(A)
cfg=json.loads((ROOT/'simulacion/control_referencia/resultados.json').read_text())['controlador']
def f(x):
    s=f'{float(x):.10g}'
    return s+('f' if '.' in s or 'e' in s else '.0f')
def matrix(name,m):return f'constexpr float {name}[{len(m)}][{len(m[0])}]={{\n'+',\n'.join('  {'+','.join(f(x) for x in row)+'}' for row in m)+'\n};\n'
text='// Generado por simulacion/exportar_control_referencia.py; geometria CAD PROVISIONAL.\n#pragma once\nnamespace simref {\n'
text+=matrix('A',A)+matrix('INV',inverse)
for name,key in [('KP','kp_actitud'),('KD','kd_actitud'),('KI','ki_actitud')]:text+=f'constexpr float {name}[3]={{'+','.join(f(x) for x in cfg[key])+'};\n'
for name,key in [('MASA','masa_nominal_kg'),('KP_Z','kp_altura'),('KD_Z','kd_altura'),('KI_Z','ki_altura')]:text+=f'constexpr float {name}={f(cfg[key])};\n'
text+='constexpr float G=9.80665f;\nconstexpr float MAX_N=6.0f; // Supuesto de simulacion, no empuje medido.\n}\n'
out=ROOT/'firmware/dron_octo_esp32/parametros_sim.h';out.write_text(text)
metadata={'origen':str(xml),'sha256_modelo':hashlib.sha256(xml.read_bytes()).hexdigest(),'origen_control':str(ROOT/'simulacion/control_simulacion.py'),'configuracion':cfg,'mapa_motores_fisico_verificado':False,'curva_empuje_pwm_medida':False,'recorrido_simulado_m':0.07662945938,'objetivo_usuario_m':0.30,'nota':'El objetivo del usuario esta fuera del recorrido simulado. No habilita motores.'}
(ROOT/'firmware/dron_octo_esp32/parametros_sim.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+'\n')
print(out)
