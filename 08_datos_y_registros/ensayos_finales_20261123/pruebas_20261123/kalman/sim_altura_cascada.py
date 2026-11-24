"""Compara dos controles de altura en la guia: (A) PID sobre la altura (el actual) y (B) cascada:
lazo externo P de posicion -> consigna de velocidad limitada; lazo interno PI de velocidad (velocidad del Kalman,
que usa el acelerometro) -> colectivo. Planta: z'' = b*u_motor - friccion de la guia (Coulomb + adherencia),
motor de 70 ms, topes de la guia en 0 y 140 mm, perturbacion lenta (bateria que cae) y medida del Kalman con ruido."""
import numpy as np
b = 0.0257 * 1000          # mm/s2 por us
def sim(modo, fric=300.0, stick=400.0, ruido_z=12, ruido_v=25, T=40, seed=0, ref=60.0, caida_us=-15):
    rng = np.random.default_rng(seed); dt = 0.01; z = 0.0; v = 0.0; thr = 0.0; I = 0.0; out = []
    zr = 0.0                                          # referencia con rampa suave (2 cm/s) hacia ref
    for k in range(int(T / dt)):
        t = k * dt
        zr = min(ref, zr + 20 * dt)
        ze = z + rng.normal(0, ruido_z); ve = v + rng.normal(0, ruido_v)
        if modo == "PID":
            e = zr - ze; I = np.clip(I + e * dt, -60 / 0.12, 60 / 0.12); u = np.clip(0.25 * e + 0.12 * I - 0.14 * ve, -60, 60)
        else:
            vref = np.clip(1.5 * (zr - ze), -40, 40)            # mm/s, como mucho 4 cm/s
            ev = vref - ve; I = np.clip(I + ev * dt, -60 / 0.35, 60 / 0.35)
            u = np.clip(0.18 * ev + 0.35 * I, -60, 60)
        pert = caida_us * min(1, t / 30)                          # la bateria cae 15 us equivalentes en 30 s
        thr += dt / 0.07 * ((u + pert) - thr)
        F = b * thr                                               # mm/s2 netos de empuje (sobre el peso)
        if abs(v) < 1e-3 and abs(F) < stick: a = 0.0; v = 0.0    # adherencia: no arranca
        else: a = F - fric * np.sign(v if abs(v) > 1e-3 else F)
        v += a * dt; z += v * dt
        if z < 0: z, v = 0.0, max(v, 0)
        if z > 140: z, v = 140.0, min(v, 0)
        out.append(z)
    o = np.array(out[int(15 / dt):]); return o.std(), np.ptp(o), abs(o.mean() - ref)
if __name__ == "__main__":
    print("friccion | PID actual: desv / pico-pico / error medio (mm) | cascada: idem")
    for f, s in ((0, 0), (150, 200), (300, 400), (600, 800)):
        a = sim("PID", f, s); c = sim("CAS", f, s)
        print(f"{f:4.0f}/{s:4.0f} | {a[0]:5.1f} / {a[1]:5.1f} / {a[2]:5.1f}       | {c[0]:5.1f} / {c[1]:5.1f} / {c[2]:5.1f}")
