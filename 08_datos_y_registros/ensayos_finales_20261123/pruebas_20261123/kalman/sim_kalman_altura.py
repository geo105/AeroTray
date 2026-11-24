"""Simulacion del filtro de Kalman altura = laser + acelerometro (para elegir sigma_a y R).
Verdad: el dron sube y baja en la guia (escalones suaves de 0-14 cm). Laser: ruido 52 mm, ~7 medidas/s utiles
(35 % de 20 Hz) con 5 % de medidas locas. Acelerometro: filtrado a 10 Hz, ruido sigma_acc y sesgo constante."""
import numpy as np
def verdad(T, dt):
    t = np.arange(0, T, dt); z = 0.46 + 0.07 * (1 - np.cos(2 * np.pi * t / 8)) * (t > 4)   # sube 14 cm en 4 s
    z += 0.01 * np.sin(2 * np.pi * 0.8 * t)                                                 # oscilacion pequena
    v = np.gradient(z, dt); a = np.gradient(v, dt); return t, z, v, a
def correr(sig_acc_real=0.6, sesgo=0.3, R=0.052**2, q_a=0.6, q_b=0.02, seed=0, T=30):
    rng = np.random.default_rng(seed); dt = 0.005; t, z, v, a = verdad(T, dt)
    x = np.array([z[0], 0.0, 0.0]); P = np.diag([0.05**2, 0.1**2, 0.5**2])
    est, med, raw_t, raw = [], [], [], []; buf = []
    for k in range(len(t)):
        am = a[k] + sesgo + rng.normal(0, sig_acc_real)
        F = np.array([[1, dt, -0.5 * dt * dt], [0, 1, -dt], [0, 0, 1]])
        x = F @ x + np.array([0.5 * dt * dt * am, dt * am, 0])
        G = np.array([0.5 * dt * dt, dt, 0]); P = F @ P @ F.T + np.outer(G, G) * q_a**2 + np.diag([0, 0, (q_b**2) * dt])
        if k % 10 == 0 and rng.random() < 0.35:                     # medida aceptada del laser (20 Hz * 35 %)
            zl = z[k] + rng.normal(0, 0.052) + (rng.uniform(-0.3, 0.3) if rng.random() < 0.05 else 0)
            S = P[0, 0] + R; inn = zl - x[0]
            if inn * inn < 9 * S:                                     # puerta de 3 sigma
                K = P[:, 0] / S; x = x + K * inn; P = P - np.outer(K, P[0, :])
            raw_t.append(t[k]); raw.append(zl); buf = (buf + [zl])[-3:]; med.append(np.median(buf))
        est.append(x[0])
    est = np.array(est); m = t > 2
    zr = np.interp(raw_t, t, z); rawe = np.array(raw) - zr; mede = np.array(med) - zr
    return 1000 * np.sqrt(np.mean((est[m] - z[m]) ** 2)), 1000 * np.std(rawe), 1000 * np.std(mede)
if __name__ == "__main__":
    print("q_a  sigma_acc_real  | error RMS Kalman | laser crudo | mediana de 3 (mm)")
    for q in (0.3, 0.6, 1.0, 2.0):
        for sar in (0.3, 0.6, 1.5):
            k, r, md = correr(sig_acc_real=sar, q_a=q)
            print(f"{q:4.1f}  {sar:4.1f}            | {k:6.1f}           | {r:6.1f}      | {md:6.1f}")
