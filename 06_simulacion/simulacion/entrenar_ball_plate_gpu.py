#!/usr/bin/env python3
"""Entrenamiento y optimizacion en GPU (2048 sistemas en paralelo) de Ball & Plate
acoplado con inclinaciones del dron y rampa de subida/bajada en el carrito.

Usa las medidas reales obtenidas en el laboratorio:
- Bandeja entre centros ArUco: 18.7 x 14.0 cm (limites +-9.35 cm, +-7.0 cm)
- Camara USB rapida: 12 FPS (dt = 83.3 ms) + 1 frame de retardo + 5% frames perdidos
- Servos GPIO41/42: rampa de velocidad real y limite +-8..12 deg
- Acoplamiento con el Dron y el Carrito:
  1) Inclinaciones dinamicas del dron (oscilaciones y escalones de +-3 a +-7 deg)
  2) Subida y bajada en rampa vertical del dron/carrito (aceleracion vertical z_ddot)
  3) Rampa de inclinacion del carrito movil (+-6 deg) y desnivel mecanico
Exporta el controlador PID+Filtro optimo a herramientas/pesos_ball_plate.json
y genera la grafica comparativa en simulacion/respuesta_ball_plate_dron_rampa.png.
"""
import json
from pathlib import Path
import time
import numpy as np
import torch
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT_JSON = ROOT / "herramientas/pesos_ball_plate.json"
OUT_PLOT = ROOT / "simulacion/respuesta_ball_plate_dron_rampa.png"


def simular_lote(kp, kd, ki, max_deg=10.0, n_envs=2048, pasos=180, seed=42, device="cuda"):
    gen = torch.Generator(device=device).manual_seed(seed)
    dt = 1.0 / 12.0  # 12 FPS reales medidos en el panel USB
    pos = torch.empty((n_envs, 2), device=device)
    pos[:, 0].uniform_(-6.0, 6.0, generator=gen)
    pos[:, 1].uniform_(-4.5, 4.5, generator=gen)
    vel = torch.empty((n_envs, 2), device=device).uniform_(-3.5, 3.5, generator=gen)

    # Parametros fisicos aleatorios por entorno (Domain Randomization)
    # 5/7 * g * sin(1 deg) * factor_biela ~ 7.5 cm/s^2 por grado de servo
    ganancia_servo = torch.empty((n_envs, 2), device=device).uniform_(6.0, 9.2, generator=gen)
    friccion = torch.empty((n_envs, 1), device=device).uniform_(0.18, 0.60, generator=gen)
    desnivel_rampa_deg = torch.empty((n_envs, 2), device=device).uniform_(-2.2, 2.2, generator=gen)
    amp_dron_deg = torch.empty((n_envs, 2), device=device).uniform_(1.0, 4.0, generator=gen)
    frec_dron = torch.empty((n_envs, 2), device=device).uniform_(1.2, 3.5, generator=gen)
    fase_dron = torch.empty((n_envs, 2), device=device).uniform_(0.0, 6.28, generator=gen)

    servo_actual = torch.zeros((n_envs, 2), device=device)
    servo_cola = torch.zeros((n_envs, 2), device=device)
    pos_prev = pos.clone()
    vel_est = torch.zeros((n_envs, 2), device=device)
    integ = torch.zeros((n_envs, 2), device=device)
    ult_med = pos.clone()

    caidas = torch.zeros(n_envs, dtype=torch.bool, device=device)
    err_hist = []

    for step in range(pasos):
        t = step * dt
        drop = torch.rand((n_envs, 1), generator=gen, device=device) < 0.05
        ruido = torch.randn((n_envs, 2), generator=gen, device=device) * 0.04
        med = torch.where(drop, ult_med, pos + ruido)
        ult_med = med

        a = dt / (0.12 + dt)
        v_raw = ((med - pos_prev) / dt).clamp(-80.0, 80.0)
        vel_est = vel_est + a * (v_raw - vel_est)
        pos_prev = med.clone()

        # Solo acumular integral cerca del centro para corregir rampa/desnivel sin sobreimpulso
        cerca = (med.abs() < 5.0).float()
        integ = (integ + cerca * med * dt).clamp(-4.5, 4.5)
        mando = (-kp * med - kd * vel_est - ki * integ).clamp(-max_deg, max_deg)

        cmd_aplicado = servo_cola.clone()
        servo_cola = mando.clone()
        delta = (cmd_aplicado - servo_actual).clamp(-7.5, 7.5)
        servo_actual = servo_actual + delta

        sub_dt = dt / 10.0
        for s in range(10):
            ts = t + s * sub_dt
            # Inclinacion dinamica del dron + rampa del carrito
            inclinacion_dron = amp_dron_deg * torch.sin(frec_dron * ts + fase_dron)
            # Factor de gravedad efectiva por subida/bajada en rampa del carrito/dron (1 +- 0.15 g)
            g_eff_scale = 1.0 + 0.15 * float(np.sin(2.0 * np.pi * ts / 3.0))
            acc = g_eff_scale * ganancia_servo * (servo_actual + desnivel_rampa_deg + 0.35 * inclinacion_dron) - friccion * vel
            vel = vel + acc * sub_dt
            pos = pos + vel * sub_dt
            fuera = (pos[:, 0].abs() > 8.85) | (pos[:, 1].abs() > 6.45)
            caidas |= fuera

        if step >= pasos - 36:
            err_hist.append(torch.linalg.vector_norm(pos, dim=1))

    err_final = torch.stack(err_hist, dim=0).mean(dim=0)
    supervivencia = float((~caidas).float().mean().item())
    rms_cm = float(torch.sqrt((err_final[~caidas] ** 2).mean()).item()) if (~caidas).any() else 99.0
    return supervivencia, rms_cm


def generar_grafica_escenarios(kp, kd, ki, max_deg):
    dt = 1.0 / 12.0
    pasos = 144  # 12 segundos
    t_arr = np.arange(pasos) * dt

    def correr_un_caso(modo):
        pos = np.array([5.2, -3.8], dtype=float)
        vel = np.array([0.0, 0.0], dtype=float)
        pos_prev = pos.copy()
        vel_est = np.zeros(2)
        integ = np.zeros(2)
        servo_act = np.zeros(2)
        servo_cola = np.zeros(2)
        hist_xy, hist_dron, hist_z = [], [], []
        for step in range(pasos):
            t = step * dt
            # Perfil de rampa de altura del dron (subida 0->6.5 cm, meseta, bajada)
            if t < 3.0:
                z_cm = 1.0 + (5.5 / 3.0) * t
                z_acc = 0.12 if t < 0.5 else (-0.12 if t > 2.5 else 0.0)
            elif t < 7.0:
                z_cm = 6.5
                z_acc = 0.0
            elif t < 10.0:
                z_cm = 6.5 - (5.0 / 3.0) * (t - 7.0)
                z_acc = -0.12 if t < 7.5 else (0.12 if t > 9.5 else 0.0)
            else:
                z_cm = 1.5
                z_acc = 0.0

            # Inclinacion del dron + rampa del carrito: escalon de rampa + oscilacion
            rampa_carrito = np.array([2.0, -1.5]) if t >= 2.0 else np.array([0.5, -0.5])
            osc_dron = np.array([3.5 * np.sin(2.2 * t), 2.8 * np.cos(1.8 * t)]) if (4.0 <= t <= 8.5) else np.zeros(2)
            inc_total = rampa_carrito + 0.35 * osc_dron

            a = dt / (0.12 + dt)
            v_raw = np.clip((pos - pos_prev) / dt, -80, 80)
            vel_est = vel_est + a * (v_raw - vel_est)
            pos_prev = pos.copy()

            if modo == "antiguo":
                mando = np.clip(-0.35 * pos - 0.22 * vel_est, -2.0, 2.0)
            else:
                if np.all(np.abs(pos) < 5.0):
                    integ = np.clip(integ + pos * dt, -4.5, 4.5)
                mando = np.clip(-kp * pos - kd * vel_est - ki * integ, -max_deg, max_deg)

            cmd_ap = servo_cola.copy()
            servo_cola = mando.copy()
            lim_vel = 1.6 if modo == "antiguo" else 7.5
            servo_act += np.clip(cmd_ap - servo_act, -lim_vel, lim_vel)

            for _ in range(10):
                acc = (1.0 + z_acc) * 7.6 * (servo_act + inc_total) - 0.32 * vel
                vel += acc * (dt / 10.0)
                pos += vel * (dt / 10.0)
                pos[0] = np.clip(pos[0], -9.35, 9.35)
                pos[1] = np.clip(pos[1], -7.0, 7.0)

            hist_xy.append(pos.copy())
            hist_dron.append(inc_total[0])
            hist_z.append(z_cm)
        return np.array(hist_xy), np.array(hist_dron), np.array(hist_z)

    xy_old, dron_inc, z_rampa = correr_un_caso("antiguo")
    xy_new, _, _ = correr_un_caso("nuevo")

    fig, axs = plt.subplots(1, 3, figsize=(15, 4.5), dpi=140)
    fig.suptitle("Simulacion Acoplada: Dron en Rampa (Subida/Bajada + Inclinaciones) + Respuesta del Ball & Plate", fontsize=12.5, fontweight="bold")

    axs[0].plot(t_arr, z_rampa, color="#1565c0", lw=2.2, label="Altura Dron/Carrito Z (cm)")
    ax0b = axs[0].twinx()
    ax0b.plot(t_arr, dron_inc, color="#ef6c00", lw=1.6, ls="--", label="Perturbacion Rampa+Dron (°)")
    axs[0].set_title("1. Rampa Subida/Bajada e Inclinacion del Dron")
    axs[0].set_xlabel("Tiempo (s)")
    axs[0].set_ylabel("Altura Z (cm)", color="#1565c0")
    ax0b.set_ylabel("Angulo perturbacion (°)", color="#ef6c00")
    axs[0].grid(True, alpha=0.3)

    err_old = np.hypot(xy_old[:, 0], xy_old[:, 1])
    err_new = np.hypot(xy_new[:, 0], xy_new[:, 1])
    axs[1].plot(t_arr, err_old, color="#c62828", lw=2, ls="--", label="Anterior (±2°, sin Ki -> se va al borde)")
    axs[1].plot(t_arr, err_new, color="#2e7d32", lw=2.3, label=f"Optimizado GPU (±{max_deg:g}°, Kp={kp:.2f}, Kd={kd:.2f}, Ki={ki:.2f})")
    axs[1].axhline(0.0, color="black", lw=0.8)
    axs[1].set_title("2. Distancia de la Pelota al Centro (0, 0) cm")
    axs[1].set_xlabel("Tiempo (s)")
    axs[1].set_ylabel("Error radial ||(X, Y)|| (cm)")
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(loc="upper right", fontsize=8.5)

    axs[2].plot(xy_old[:, 0], xy_old[:, 1], color="#c62828", ls="--", lw=1.8, label="Trayectoria anterior (±2°)")
    axs[2].plot(xy_new[:, 0], xy_new[:, 1], color="#2e7d32", lw=2.2, label="Trayectoria optimizada GPU")
    axs[2].scatter([xy_new[0, 0]], [xy_new[0, 1]], color="#1565c0", s=55, zorder=5, label="Inicio (5.2, -3.8) cm")
    axs[2].scatter([0], [0], color="gold", edgecolor="black", s=90, marker="*", zorder=6, label="Meta Centro (0, 0)")
    axs[2].set_xlim(-9.5, 9.5)
    axs[2].set_ylim(-7.2, 7.2)
    axs[2].set_title("3. Vista Superior de la Bandeja (18.7 x 14.0 cm)")
    axs[2].set_xlabel("X bandeja (cm)")
    axs[2].set_ylabel("Y bandeja (cm)")
    axs[2].grid(True, alpha=0.3)
    axs[2].legend(loc="lower left", fontsize=8.5)

    plt.tight_layout()
    plt.savefig(OUT_PLOT)
    print(f"Grafica guardada en {OUT_PLOT}")


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    t0 = time.monotonic()
    print(f"Entrenando Ball & Plate acoplado con Dron + Rampa en {device.upper()} (2048 entornos en paralelo)...", flush=True)

    surv_old, rms_old = simular_lote(0.35, 0.22, 0.0, max_deg=2.0, device=device)
    print(f"  [Anterior +-2 deg] -> Supervivencia en bandeja: {surv_old*100:.1f}% | Error final RMS: {rms_old:.2f} cm", flush=True)

    mejor = (0.0, 99.0, 0.55, 0.80, 0.50)
    kps = torch.linspace(0.50, 1.20, 8)
    kds = torch.linspace(0.35, 0.85, 6)
    kis = (0.0, 0.25, 0.50)
    total = len(kps) * len(kds) * len(kis)
    idx = 0
    for kp in kps:
        for kd in kds:
            for ki in kis:
                idx += 1
                surv, rms = simular_lote(float(kp), float(kd), float(ki), max_deg=10.0, n_envs=2048, device=device)
                score = surv * 100.0 - rms
                best_score = mejor[0] * 100.0 - mejor[1]
                if score > best_score:
                    mejor = (surv, rms, float(kp), float(kd), float(ki))
                    print(
                        f"  [{idx:3d}/{total}] Nuevo optimo -> Kp={float(kp):.2f}, Kd={float(kd):.2f}, Ki={float(ki):.2f} "
                        f"| Supervivencia={surv*100:.1f}% | Error RMS={rms:.2f} cm",
                        flush=True,
                    )
                elif idx % 24 == 0:
                    print(
                        f"  [{idx:3d}/{total}] Evaluando lote GPU (2048 bandejas)... Mejor actual: "
                        f"Supervivencia={mejor[0]*100:.1f}%, RMS={mejor[1]:.2f} cm",
                        flush=True,
                    )

    surv_opt, rms_opt, kp_opt, kd_opt, ki_opt = mejor
    dt_s = time.monotonic() - t0
    print(f"  [Optimizado GPU +-10 deg: Kp={kp_opt:.2f}, Kd={kd_opt:.2f}, Ki={ki_opt:.2f}] -> Supervivencia: {surv_opt*100:.1f}% | Error RMS: {rms_opt:.2f} cm ({dt_s:.1f} s)", flush=True)

    resultado = {
        "fecha": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "dispositivo": device,
        "bandejas_paralelas": 2048,
        "escala_aruco_cm": [18.7, 14.0],
        "fps_camara_simulado": 12.0,
        "escenario": "Dron en rampa subida/bajada + oscilaciones de inclinacion +-4 deg + rampa carrito +-2.2 deg",
        "anterior_2deg": {"kp": 0.35, "kd": 0.22, "ki": 0.0, "max_deg": 2.0, "supervivencia": round(surv_old, 4), "rms_cm": round(rms_old, 3)},
        "optimizado_gpu_10deg": {"kp": round(kp_opt, 3), "kd": round(kd_opt, 3), "ki": round(ki_opt, 3), "max_deg": 10.0, "supervivencia": round(surv_opt, 4), "rms_cm": round(rms_opt, 3)},
    }
    OUT_JSON.write_text(json.dumps(resultado, indent=2, ensure_ascii=False) + "\n")
    generar_grafica_escenarios(kp_opt, kd_opt, ki_opt, 10.0)


if __name__ == "__main__":
    main()
