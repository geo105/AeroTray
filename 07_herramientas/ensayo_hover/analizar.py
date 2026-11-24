#!/usr/bin/env python3
"""Audita el CSV completo y resume angulos/PWM. No infiere hover sin altura."""
import argparse
import csv
import json
import math
from pathlib import Path


def analyze(path):
    rows = list(csv.DictReader(open(path/"telemetria.csv")))
    active = [r for r in rows if r["modo"] in ("RAMPA", "CONTROL")]
    events = [json.loads(s) for s in (path/"eventos.jsonl").read_text().splitlines()]
    seq_lost = 0
    reboots = 0
    previous = None
    for r in rows:
        seq = int(r["seq"])
        if previous is not None:
            delta = (seq-previous)&0xffffffff
            if delta == 0 or delta > 0x7fffffff:
                reboots += 1
            elif delta > 1:
                seq_lost += delta-1
        previous = seq
    meta = json.loads((path/"sesion.json").read_text())
    report = {"tipo": meta.get("tipo", "sesion de consola"),
              "usar_para_identificar_planta": meta.get("usar_para_identificar_planta"),
              "muestras": len(rows), "muestras_activas": len(active), "huecos_secuencia": seq_lost,
              "reinicios": reboots, "crc_o_formato_invalidos": meta.get("crc_o_formato_invalidos"),
              "tx_descartadas_max": max([int(r["tx_descartadas"]) for r in rows], default=0),
              "altura_valida": False, "hover_confirmado": False,
              "marcas_manuales": [e for e in events if e.get("comando") in ("hover", "marca")],
              "limite": "Guias y rozamiento alteran la sustentacion. No se confirma hover ni altura con angulos/IMU.",
              "cortes": [e for e in events if e["evento"] == "modo" and e.get("modo")=="OFF" and e.get("motivo_corte")]}
    if meta.get("usar_para_identificar_planta") is False:
        report["limite"] = "Prueba logica sin alimentacion de motores. No usar para identificar la planta ni estimar sustentacion."
    rev2=[r for r in rows if r.get("revision_accel")=="2"]
    if rev2:
        report["proteccion_accel"]={"revision":2,"raw_max_g":max(float(r["accel_norma_g"]) for r in rev2),
                                    "filtrada_max_g":max(float(r["accel_filtrada_g"]) for r in rev2),
                                    "fuera_max_ms":max(int(r["accel_fuera_us"]) for r in rev2)/1000,
                                    "tipos_corte_registrados":sorted(set(int(r["accel_tipo"]) for r in rev2))}
    if active:
        tilt = [math.degrees(math.acos(max(-1., min(1., math.cos(math.radians(float(r["roll_deg"])))*math.cos(math.radians(float(r["pitch_deg"]))))))) for r in active]
        report.update({"tilt_max_deg": max(tilt), "tilt_rms_deg": math.sqrt(sum(t*t for t in tilt)/len(tilt)),
                       "roll_rms_deg": math.sqrt(sum(float(r["roll_deg"])**2 for r in active)/len(active)),
                       "pitch_rms_deg": math.sqrt(sum(float(r["pitch_deg"])**2 for r in active)/len(active)),
                       "pwm_por_motor": {f"M{i}": {"min": min(int(r[f"M{i}_us"]) for r in active),
                                                            "max": max(int(r[f"M{i}_us"]) for r in active),
                                                            "media": sum(int(r[f"M{i}_us"]) for r in active)/len(active)} for i in range(1,9)},
                       "intervalo_max_us": max(int(r["intervalo_us"]) for r in active),
                       "lectura_imu_max_us": max(int(r["lectura_imu_us"]) for r in active),
                       "fraccion_recorte_rp": sum(float(r["escala_rp"])<.999 for r in active)/len(active)})
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("sesion",type=Path)
    args=ap.parse_args()
    report=analyze(args.sesion)
    target=args.sesion/"informe.json"
    target.write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n")
    print(target)
    print(json.dumps(report,indent=2,ensure_ascii=False))


if __name__=="__main__":
    main()
