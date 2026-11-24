"""Corridas emparejadas de reserva en Gazebo, sin seleccionar nuevas ganancias."""
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from control import ROOT


def run(item):
    seed,gains=item
    stem=f'reserva_{seed}_'+('base' if gains is None else 'candidato')
    command=[sys.executable,str(ROOT/'probar_gazebo.py'),'--segundos','12','--seed',str(seed),'--salida',stem]
    if gains is not None: command+=['--ball-gains',*map(str,gains)]
    p=subprocess.run(command,capture_output=True,text=True,timeout=100)
    if p.returncode: raise RuntimeError(stem+'\n'+p.stdout[-1000:]+p.stderr[-1000:])
    return json.loads((ROOT/'resultados'/f'{stem}.json').read_text())


def main():
    gains=json.loads((ROOT/'resultados/ajuste_ball_plate.json').read_text())['perfil_simulacion']
    cases=[(seed,gain) for seed in [81243,91357,104729] for gain in [None,gains]]
    with ThreadPoolExecutor(max_workers=2) as pool: results=list(pool.map(run,cases))
    baseline=results[::2]; candidate=results[1::2]
    mean=lambda rows,key:sum(x[key] for x in rows)/len(rows)
    base_error=mean(baseline,'rms_bola_mm'); new_error=mean(candidate,'rms_bola_mm')
    accepted=bool(new_error<base_error and all(not x['cortes'] and not x['bola_fuera_placa'] for x in candidate))
    report={'semillas':[81243,91357,104729],'segundos_por_caso':12,'casos':results,
        'rms_bola_original_mm':base_error,'rms_bola_candidato_mm':new_error,
        'reduccion_error_porcentaje':100*(1-new_error/base_error),
        'aceptado_en_gazebo_solo_simulacion':accepted,
        'nota':'Reserva distinta de la búsqueda; misma semilla/perturbación por pareja. Planta de pelota reducida y dinámica provisional. No acredita mejora física.'}
    (ROOT/'resultados/reserva_gazebo.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='casos'},ensure_ascii=False,indent=2))


if __name__=='__main__': main()
