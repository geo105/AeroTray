"""Control y sensores de simulación. Nunca abre un puerto ni envía PWM real."""
from pathlib import Path
import json
import numpy as np

ROOT = Path(__file__).resolve().parent
CFG = json.loads((ROOT / 'config.json').read_text())
G = 9.81


def rotation(rpy):
    r, p, y = rpy
    cr, cp, cy = np.cos([r, p, y]); sr, sp, sy = np.sin([r, p, y])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def model_force(pwm, cfg=CFG):
    a, c = cfg['actuadores'], cfg['control']
    return a['fmax_nominal_N'] * np.maximum(0, (np.asarray(pwm)-c['inicio_giro_us']) /
        (a['pwm_max_modelo']-c['inicio_giro_us']))**a['exponente']


def pulse_for_force(force, cfg=CFG):
    a, c = cfg['actuadores'], cfg['control']
    pulse = c['inicio_giro_us'] + (a['pwm_max_modelo']-c['inicio_giro_us']) * (
        np.maximum(0, force)/a['fmax_nominal_N'])**(1/a['exponente'])
    # lroundf del firmware: positivo, medio hacia arriba.
    return np.floor(np.clip(pulse, c['min_us'], c['max_us'])+.5).astype(int)


class Sensors:
    """IMU en brazo 8 y rayo ToF hacia el suelo desde brazo 1.

    Posiciones/orientación y ruido ToF son supuestos editables. La IMU incluye
    aceleración tangencial y centrípeta. El estimador copia el filtro del ZIP.
    """
    def __init__(self, seed=73021, noise=True, cfg=CFG):
        self.cfg = cfg; self.rng = np.random.default_rng(seed); self.noise = noise
        self.angles = np.zeros(3); self.norm = cfg['sensores']['accel_scale']
        self.accel = np.array([0., 0., self.norm]); self.gyro = np.zeros(3)
        self.tof = np.nan; self.height = np.nan; self.speed = 0.
        self.tof_time = -1.; self.valid = False; self.samples = 0

    def update_imu(self, R, omega, alpha, pivot_accel, dt):
        s = self.cfg['sensores']; lever = np.asarray(s['imu_pos_m'])
        specific = (R.T @ (pivot_accel + np.array([0., 0., G])) +
                    np.cross(alpha, lever) + np.cross(omega, np.cross(omega, lever))) / G
        self.accel = specific * s['accel_scale']
        self.gyro = omega.copy()
        if self.noise:
            self.accel += self.rng.normal(0, s['accel_sd_g'])
            self.gyro += self.rng.normal(0, np.deg2rad(s['gyro_sd_dps']))
        self.norm = float(np.linalg.norm(self.accel))
        r, p, _ = self.angles
        side = np.sin(r)*self.gyro[1] + np.cos(r)*self.gyro[2]
        cp = max(.25, np.cos(p))
        self.angles += dt*np.array([self.gyro[0]+np.sin(p)/cp*side,
            np.cos(r)*self.gyro[1]-np.sin(r)*self.gyro[2], side/cp])
        if abs(self.norm-s['accel_scale']) < .15:
            measured = np.array([np.arctan2(self.accel[1], self.accel[2]),
                np.arctan2(-self.accel[0], np.hypot(self.accel[1], self.accel[2]))])
            self.angles[:2] += dt/(.25+dt)*(measured-self.angles[:2])

    def update_tof(self, R, pivot_z, t, dropout=False):
        s = self.cfg['sensores']
        if self.tof_time >= 0 and t-self.tof_time < s['tof_periodo_s']-1e-7:
            return
        dt = s['tof_periodo_s'] if self.tof_time < 0 else t-self.tof_time
        self.tof_time = t
        sensor_z = pivot_z + (R @ np.asarray(s['tof_pos_m']))[2]
        direction_z = (R @ np.asarray(s['tof_direccion']))[2]
        distance = -sensor_z/direction_z if direction_z < -.1 else np.nan
        if self.noise:
            distance += self.rng.normal(0, s['tof_ruido_m'])
        self.valid = bool(not dropout and np.isfinite(distance) and
            s['tof_rango_m'][0] <= distance <= s['tof_rango_m'][1])
        self.tof = distance if self.valid else np.nan
        if self.valid:
            # Se usa actitud ESTIMADA, nunca la verdadera, para compensar el brazo.
            estimated_R = rotation(self.angles)
            z = (-distance*(estimated_R @ np.asarray(s['tof_direccion']))[2] -
                 (estimated_R @ np.asarray(s['tof_pos_m']))[2] - self.cfg['pivot_altura_m'])
            if self.samples:
                weight=dt/(s['tof_vel_tau_s']+dt)
                self.speed += weight*((z-self.height)/dt-self.speed)
            self.height = z; self.samples += 1


class FirmwareControl:
    """P de ángulo + PI de velocidad y mezclador del ZIP, sin PI de yaw.

    `altura_ideal` usa fuerzas, sin equivalencia PWM validada, para estudiar
    arquitectura. `altura` respeta el techo original de 1500 us.
    """
    def __init__(self, mode='banco', gain=1., cfg=CFG):
        self.cfg = cfg; self.mode = mode; self.gain = gain
        self.integral = np.zeros(2); self.iz = 0.; self.scale = 1.
        self.pwm = np.full(8, cfg['control']['base_us'], dtype=int)
        self.request = model_force(self.pwm, cfg)
        self.cut = ''; self.target = cfg['altura']['objetivo_m']
        self.reference = 0.
        self.xy = np.array(cfg['motores_flu_m'])[:, :2]

    def update(self, sensors, dt, residual=None):
        c = self.cfg['control']; angle = sensors.angles; rates = sensors.gyro
        if not self.cut:
            if not np.isfinite(np.r_[angle, rates, sensors.norm]).all(): self.cut = 'NO_FINITO'
            elif np.linalg.norm(angle[:2]) >= np.deg2rad(c['limite_tilt_deg']): self.cut = 'INCLINACION'
            elif abs(angle[2]) >= np.deg2rad(c['limite_yaw_deg']): self.cut = 'YAW_RELATIVO'
            elif abs(rates[2]) >= np.deg2rad(c['limite_r_dps']): self.cut = 'VELOCIDAD_YAW'
            elif not c['limite_accel_g'][0] <= sensors.norm <= c['limite_accel_g'][1]: self.cut = 'ACELERACION'
            elif self.mode != 'banco' and not sensors.valid: self.cut = 'TOF_INVALIDO'
        if self.cut:
            self.pwm[:] = 1000; self.request[:] = 0.; return self.request
        phi_dot, theta_dot = np.clip(-self.gain*np.array(c['kp_angulo'])*angle[:2],
                                     -np.deg2rad(25), np.deg2rad(25))
        qref = theta_dot/max(.5, np.cos(angle[0]))
        pref = phi_dot-np.sin(angle[0])*np.tan(angle[1])*qref
        error = np.array([pref, qref])-rates[:2]
        torque = np.array(c['kp_velocidad'])*error+np.array(c['ki_velocidad'])*self.integral
        if residual is not None: torque += np.clip(residual, -.025, .025)
        base = float(model_force(c['base_us'], self.cfg))
        hi = float(model_force(c['max_us'], self.cfg))
        lo = float(model_force(c['min_us'], self.cfg))
        if self.mode != 'banco':
            h = self.cfg['altura']
            rate=h['rampa_referencia_m_s']
            self.reference+=float(np.clip(self.target-self.reference,-rate*dt,rate*dt))
            ez = self.reference-sensors.height
            weight = (self.cfg['masa_rotatoria_kg']+self.cfg['masa_carro_vertical_kg'])*G
            total = (weight+h['kp_N_m']*ez-h['kd_N_s_m']*sensors.speed+self.iz)/max(.5, rotation(angle)[2,2])
            if self.mode == 'altura_ideal': lo, hi = 0., self.cfg['actuadores']['fmax_nominal_N']
            base = float(np.clip(total/8, lo, hi))
            self.iz = float(np.clip(self.iz+dt*(h['ki_N_ms']*ez+3*(8*base-total)), -8, 8))
        delta = (self.xy[:,1]*torque[0]-self.xy[:,0]*torque[1])/(4*self.cfg['radio_m']**2)
        self.scale = 1.
        for d in delta:
            if d > 1e-7: self.scale = min(self.scale, (hi-base)/d)
            elif d < -1e-7: self.scale = min(self.scale, (lo-base)/d)
        self.scale = float(np.clip(self.scale, 0, 1))
        force = np.clip(base+self.scale*delta, lo, hi)
        if self.mode == 'altura_ideal':
            self.request = force; self.pwm[:] = -1
        else:
            self.pwm = pulse_for_force(force, self.cfg)
            self.request = model_force(self.pwm, self.cfg)
        if self.scale > .999:
            self.integral = np.clip(self.integral+dt*error, -.3, .3)
        return self.request


def real_rows():
    rows = []
    for line in (ROOT/'datos/evidencia.txt').read_text().splitlines():
        a = line.split(',')
        if len(a) == 19 and a[1] == 'CONTROL':
            row = [float(a[0])]+[float(v) for v in a[2:]]
            if not np.isfinite(row).all(): raise ValueError('Dato no finito en evidencia')
            rows.append(row)
    if not rows: raise ValueError('Evidencia sin muestras CONTROL')
    return np.array(rows)
