"""Calculos que se hacen SIN encender el robot: rampa (vuelco, traccion), nivelacion del plato,
consumo y autonomia de cada bateria. Usa los parametros del modelo del paper (scripts_modelo/params.py).

El centro de masa de la base todavia no esta medido: se calcula con las dos hipotesis
(ensayo de vuelco del 05/10: x = 180 mm; CAD: x = 81 mm). La prueba P1 de hoy decide cual vale.
Uso: python3 calculos_sin_encender.py  [--xg 0.120 --hg 0.150  para usar el valor medido hoy]
"""
import argparse, json, os, sys
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, "..", "laptop2", "AeroTray_Paper_Final_ES_Overleaf", "AeroTray_Paper_ES", "scripts_modelo"))
from params import P, g  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--xg", type=float, help="x del CM de la base medido hoy (m, por delante del eje motriz)")
ap.add_argument("--hg", type=float, help="altura del CM de la base medida hoy (m)")
a = ap.parse_args()

Mb, hb = P["M_base"], P["hG_base"]
Mm = P["m"] + P["mc"]                 # modulo aereo (3,6 kg)
lJ, hJ, L = P["lJ"], P["hJ0"], P["Lwb"]
r, w0, ts, Crr, istall = P["r"], P["w0"], P["tau_s"], P["Crr"], P["i_stall"]
kF = 7.6                              # N por motor a 2000 us (identificado al despegar)
hipotesis = {"ensayo 05/10 (x=180 mm)": (0.180, hb), "CAD (x=81 mm)": (0.081, hb)}
if a.xg is not None:
    hipotesis = {f"MEDIDO HOY (x={a.xg*1000:.0f} mm)": (a.xg, a.hg if a.hg else hb)}

F_IDLE = 0.12                          # fraccion del peso que sostienen los rotores a 1300 us (ralenti)


def cg(f, xb, hb_):
    me = Mm * (1 - f); M = Mb + me
    return M, (Mb * xb + me * lJ) / M, (Mb * hb_ + me * hJ) / M


def vuelco_subida(f, xb, hb_):      # gira hacia atras sobre el eje motriz (rampa de frente, subiendo)
    M, x, h = cg(f, xb, hb_); return np.degrees(np.arctan2(x, h))


def vuelco_bajada(f, xb, hb_):      # gira hacia adelante sobre el eje omni (bajando de frente)
    M, x, h = cg(f, xb, hb_); return np.degrees(np.arctan2(L - x, h))


def mu_subida(f, b, xb, hb_):        # friccion necesaria en las ruedas motrices para subir
    M, x, h = cg(f, xb, hb_); b = np.radians(b)
    N_r = M * g * ((L - x) * np.cos(b) + h * np.sin(b)) / L
    return M * g * (np.sin(b) + Crr * np.cos(b)) / max(N_r, 1e-9)


def ucol(f):
    return P["u0"] + P["uspan"] * (f * Mm * g / (8 * kF)) ** (1 / P["n_th"])


def tabla_rampa():
    out = {}
    for nom, (xb, hb_) in hipotesis.items():
        filas = []
        for etiqueta, f in (("dron apagado", 0.0), ("ralenti 1300 us", F_IDLE), ("modo rampa", 0.5), ("flotando", 1.0)):
            filas.append(dict(modo=etiqueta, f=f, colectivo_us=round(1300 if f == F_IDLE else (0 if f == 0 else ucol(f))),
                              vuelco_subida_deg=round(float(vuelco_subida(f, xb, hb_)), 1),
                              vuelco_bajada_deg=round(float(vuelco_bajada(f, xb, hb_)), 1),
                              mu_subir_10=round(float(mu_subida(f, 10, xb, hb_)), 2),
                              mu_subir_20=round(float(mu_subida(f, 20, xb, hb_)), 2)))
        out[nom] = filas
    return out


def nivelacion():
    """Recorrido del servo que se gasta solo en nivelar el plato."""
    lim_deg = 220 * P["servo_deg_per_us"] * P["r_arm"] / P["R_horn"] * 1.0   # grados de plato por 220 us
    lim_deg = np.degrees(np.radians(220 * P["servo_deg_per_us"]) * P["r_arm"] / P["R_horn"])
    filas = []
    for beta in (0, 5, 10, 15, 20):
        sin_dron = 100 * beta / lim_deg
        con_dron = (100 * 0.55 / lim_deg, 100 * 1.7 / lim_deg)   # inclinacion residual medida del dron
        filas.append(dict(pendiente_deg=beta, solo_servos_pct=round(min(sin_dron, 999), 0),
                          con_dron_pct=f"{con_dron[0]:.0f}-{con_dron[1]:.0f}",
                          sesgo_sin_dron_us=round(beta / lim_deg * 220)))
    return round(float(lim_deg), 1), filas


def baterias():
    """Corriente y autonomia por fuente. [EST] = estimado; medir hoy con multimetro/pinza (P7)."""
    P_motor_hover = 62.5                     # W por motor a f=1 (bateria.py del paper)
    def I_rama(f, V=11.2):                   # 4 motores por rama; potencia ~ f^1.5 (teoria de momento)
        return 4 * P_motor_hover * f ** 1.5 / V + 0.3   # +0,3 A de los ESC en reposo [EST]
    Q_lipo = 3.3 * 0.8                        # Ah utiles (80 % de descarga)
    lipo = []
    for nom, f in (("ralenti 1300 us (sobre la base)", F_IDLE), ("modo rampa ~1540 us", 0.5), ("flotando ~1750 us", 1.0)):
        I = I_rama(f); lipo.append(dict(modo=nom, I_por_rama_A=round(I, 1), autonomia_min=round(60 * Q_lipo / I, 0)))
    # base TETRIX: corriente de un TorqueNADO ~ i0 + istall * par/par_bloqueo
    def I_base(beta, f, xb=0.081):
        M, _, _ = cg(f, xb, hb); par = r * M * g * (np.sin(np.radians(beta)) + Crr * np.cos(np.radians(beta))) / 2
        return 2 * (0.4 + istall * par / ts)    # 0,4 A en vacio por motor [EST]
    Q_nimh = 3.0 * 0.8
    base = []
    for nom, beta in (("llano", 0), ("rampa 10 grados", 10), ("rampa 20 grados", 20)):
        I = I_base(beta, F_IDLE); base.append(dict(tramo=nom, I_A=round(I, 2), autonomia_min_continuo=round(60 * Q_nimh / I, 0)))
    logica = dict(cargas="Raspberry Pi 3B+ + PS3 Eye + ESP32-S3 + NodeMCU", P_W=6.0,
                  bateria="3S 18650 1,6 Ah (17,8 Wh)", autonomia_h=round(17.8 * 0.85 / 6.0, 1))
    servos = dict(cargas="2 S3003 con PID activo", P_W=2.5, bateria="3S 18650 1,6 Ah", autonomia_h=round(17.8 * 0.85 / 2.5, 1))
    myrio = dict(cargas="myRIO-1900 + encoders", P_W=6.0, bateria="power bank 10 Ah a 3,7 V (37 Wh) [EST]",
                 autonomia_h=round(37 * 0.85 * 0.9 / 6.0, 1))
    return dict(lipo_por_rama=lipo, base_nimh=base, logica=logica, servos=servos, myrio=myrio)


if __name__ == "__main__":
    res = {"rampa": tabla_rampa()}
    lim, niv = nivelacion(); res["recorrido_plato_deg"] = lim; res["nivelacion"] = niv
    res["baterias"] = baterias()
    json.dump(res, open(os.path.join(AQUI, "calculos_sin_encender.json"), "w"), indent=1, ensure_ascii=False)
    for nom, filas in res["rampa"].items():
        print(f"\n== Rampa, CM de la base: {nom}")
        print(f"{'modo':32s} {'col.':>6s} {'vuelco sub.':>11s} {'vuelco baj.':>11s} {'mu 10':>6s} {'mu 20':>6s}")
        for x in filas:
            print(f"{x['modo']:32s} {x['colectivo_us']:6d} {x['vuelco_subida_deg']:11.1f} {x['vuelco_bajada_deg']:11.1f} {x['mu_subir_10']:6.2f} {x['mu_subir_20']:6.2f}")
    print(f"\n== Nivelacion (recorrido del plato +-{lim} grados con +-220 us)")
    for x in niv: print(x)
    print("\n== Baterias"); print(json.dumps(res["baterias"], indent=1, ensure_ascii=False))


# ---------------------------------------------------------------- ciclo real de la rampa
def ciclo_rampa(xb=0.081, duty=0.15):
    """Rampa real: 1 m de subida a 10 deg, 0,5 m de meseta y 1 m de bajada a 10 deg.
    Velocidad de regimen de la base con el modelo del motor TorqueNADO (par lineal con la velocidad)."""
    tramos = [("subida", 1.0, 10.0), ("meseta", 0.5, 0.0), ("bajada", 1.0, -10.0)]
    out = []
    for modo, f in (("dron en ralenti", F_IDLE), ("modo rampa", 0.5)):
        t_tot = 0.0; Q_base = 0.0; filas = []
        for nom, Ltr, b in tramos:
            M, _, _ = cg(f, xb, hb)
            par = r * M * g * (np.sin(np.radians(b)) + Crr * np.cos(np.radians(b))) / 2   # por rueda
            v = max(r * w0 * (duty - par / ts), 1e-3)   # m/s en lazo abierto (el PI de LabVIEW la regula)
            t = Ltr / v; I = 2 * (0.4 + istall * max(par, 0) / ts)
            filas.append(dict(tramo=nom, largo_m=Ltr, pendiente_deg=b, v_cm_s=round(100 * v, 1), t_s=round(t, 1), I_base_A=round(I, 2)))
            t_tot += t; Q_base += I * t / 3600
        I_dron = 2 * (4 * 62.5 * f ** 1.5 / 11.2 + 0.3)
        out.append(dict(modo=modo, tramos=filas, t_total_s=round(t_tot, 1), Q_base_mAh=round(1000 * Q_base, 1),
                        Q_dron_mAh=round(1000 * I_dron * t_tot / 3600, 1),
                        ciclos_por_carga_NiMH=int(3000 * 0.8 / (1000 * Q_base)),
                        ciclos_por_carga_LiPo=int(2 * 3300 * 0.8 / (1000 * I_dron * t_tot / 3600))))
    quiebres = dict(altura_m=round(1.0 * np.sin(np.radians(10)), 3), cambio_de_cabeceo_por_quiebre_deg=10,
                    quiebres_por_ciclo=4, duracion_quiebre_s_subiendo=round(L / (out[0]["tramos"][0]["v_cm_s"] / 100), 1))
    return out, quiebres


if __name__ == "__main__":
    c, q = ciclo_rampa()
    res = json.load(open(os.path.join(AQUI, "calculos_sin_encender.json")))
    res["ciclo_rampa"] = c; res["quiebres_rampa"] = q
    json.dump(res, open(os.path.join(AQUI, "calculos_sin_encender.json"), "w"), indent=1, ensure_ascii=False)
    print("\n== Ciclo de rampa real (1 m a 10 deg + 0,5 m + 1 m a -10 deg), ciclo de trabajo 15 %, CM CAD")
    for x in c:
        print(f"-- {x['modo']}: {x['t_total_s']} s por ciclo; base {x['Q_base_mAh']} mAh, dron {x['Q_dron_mAh']} mAh;"
              f" ciclos por carga: NiMH {x['ciclos_por_carga_NiMH']}, LiPo {x['ciclos_por_carga_LiPo']}")
        for t in x["tramos"]: print("   ", t)
    print(q)
