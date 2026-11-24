"""Adaptador sin ROS para el diagnóstico de motores. No abre puertos."""
import math
import time

PINES = [25, 26, 27, 14, 21, 19, 5, 17]
VERSIONES = {'dsm-motores-0.1': 1300, 'dsm-motores-0.2': 1950, 'dsm-motores-0.3': 1950}
ESTADOS = ['DESARMADO', 'ARMANDO', 'LISTO', 'PRUEBA MOTOR']


class DiagnosticoMotores:
    def __init__(self):
        self.pwm = 1200
        self.segundos = 2.0
        self.observaciones = {}
        self.pruebas = {}
        self.invalidar()

    def invalidar(self):
        self.estado = None
        self.recibido = 0.0

    def actualizar(self, dato):
        if dato.get('version') not in VERSIONES:
            return False
        for campo in ('ms', 'estado', 'motor', 'pwm_us'):
            v = dato.get(campo)
            if type(v) not in (int, float) or not math.isfinite(v) or int(v) != v:
                return False
        if not (0 <= dato['estado'] <= 3 and 0 <= dato['motor'] <= 8):
            return False
        if not 0 <= dato['pwm_us'] <= VERSIONES[dato['version']]:
            return False
        if dato['ms'] < 0:
            return False
        if 'restante_ms' in dato and (type(dato['restante_ms']) is not int or not 0 <= dato['restante_ms'] <= 5000):
            return False
        grupo = dato.get('todos', False)
        if type(grupo) is not bool:
            return False
        if grupo and not (dato['version'] == 'dsm-motores-0.3' and dato['estado'] == 3 and dato['motor'] == 0):
            return False
        if dato['estado'] == 3 and not (dato['pwm_us'] >= 1000 and (grupo or 1 <= dato['motor'] <= 8)):
            return False
        anterior = self.estado
        self.estado = dict(dato)
        self.recibido = time.monotonic()
        if dato['estado'] == 3 and not grupo:
            if not anterior or anterior['estado'] != 3 or anterior['motor'] != dato['motor'] or anterior['pwm_us'] != dato['pwm_us']:
                self.observaciones.pop(dato['motor'], None)
            self.pruebas[dato['motor']] = dict(dato)
        return True

    def vigente(self):
        return self.estado is not None and time.monotonic() - self.recibido < 1.0

    def preparar(self, texto):
        """Devuelve (comando serie o None, evento local o None)."""
        p = texto.split()
        if not p:
            return None, None
        if p[0] in ('!', 'stop', 'desarmar', 'ver', 'hb') and len(p) == 1:
            return p[0], None
        if p[0] == 'pwm' and len(p) == 2:
            us = int(p[1])
            if not 1000 <= us <= 1950:
                raise ValueError('PWM permitido: 1000..1950 us; por defecto 1200 us')
            self.pwm = us
            return None, {'seleccion_pwm_us': us}
        if p[0] == 'tiempo' and len(p) == 2:
            s = float(p[1])
            if not math.isfinite(s) or not .1 <= s <= 5:
                raise ValueError('Duración permitida: 0.1..5 segundos')
            self.segundos = s
            return None, {'seleccion_duracion_s': s}
        if p[0] == 'observado' and len(p) == 3:
            n = int(p[1])
            if n not in self.pruebas or p[2] not in ('cw', 'ccw', 'no-gira'):
                raise ValueError('Usar observado N cw|ccw|no-gira después de una prueba confirmada')
            evento = {'motor': n, 'gpio': PINES[n-1], 'sentido_observado': p[2],
                      'medido_por_sensor': False, 'prueba_confirmada': self.pruebas[n]}
            self.observaciones[n] = evento
            return None, evento
        if not self.vigente():
            raise ValueError('Sin diagnóstico de motores identificado y reciente; no se envía armado/prueba')
        if p == ['armar']:
            if self.estado['estado'] != 0:
                raise ValueError('Primero desarmar; esperar LISTO después de armar')
            return 'armar', None
        if p[0] in ('nivel_imu', 'guardar', 'cero_z', 'ver_adc', 'ver_sensores', 'test_laser', 'estado_montaje') and len(p) == 1:
            if self.estado['version'] != 'dsm-motores-0.3' or self.estado['estado'] != 0:
                raise ValueError('Sensores: requiere diagnóstico 0.3 y motores DESARMADOS (! primero)')
            return p[0], None
        if p[0] == 'todos' and 1 <= len(p) <= 3:
            if self.estado['version'] != 'dsm-motores-0.3':
                raise ValueError('La prueba de todos requiere cargar el diagnóstico 0.3')
            us = int(p[1]) if len(p) >= 2 else self.pwm
            segundos = float(p[2]) if len(p) == 3 else self.segundos
            if not (1000 <= us <= 1950 and math.isfinite(segundos) and .1 <= segundos <= 5):
                raise ValueError('todos PWM(1000..1950) segundos(0.1..5), sin hélices')
            if self.estado['estado'] != 2:
                raise ValueError('Esperar LISTO antes de probar todos')
            return f'todos {us} {round(segundos*1000)}', None
        if p[0] == 'probar' and 2 <= len(p) <= 4:
            n = int(p[1])
            us = int(p[2]) if len(p) >= 3 else self.pwm
            segundos = float(p[3]) if len(p) == 4 else self.segundos
            limite = VERSIONES[self.estado['version']]
            if not (1 <= n <= 8 and 1000 <= us <= limite and math.isfinite(segundos) and .1 <= segundos <= 5):
                raise ValueError(f'probar N(1..8) PWM(1000..{limite}) segundos(0.1..5)')
            if self.estado['estado'] != 2:
                raise ValueError('Esperar estado LISTO; solo un motor por prueba')
            return f'motor {n} {us} {round(segundos*1000)}', None
        raise ValueError('Modo identificación: armar | probar N [PWM [segundos]] | todos [PWM [segundos]] | pwm US | tiempo S | observado N cw|ccw|no-gira | !')

    def telemetria(self):
        r = self.estado
        motores = [0 if r['estado'] == 0 else 1000] * 8
        if r['estado'] == 3:
            if r.get('todos', False):
                motores = [r['pwm_us']] * 8
            else:
                motores[r['motor']-1] = r['pwm_us']
        # Estos NaN no se publican como IMU/altura/baterías medidas.
        d = {k: math.nan for k in ('pitch', 'roll', 'yaw', 'gx', 'gy', 'gz',
                                  'sp_pitch', 'sp_roll', 'z_cm', 'vbat_a', 'vbat_b')}
        d.update(ms=r['ms'], estado=[0, 1, 1, 3][r['estado']],
                 diagnostico=True, estado_texto='PRUEBA TODOS' if r.get('todos') else ESTADOS[r['estado']],
                 todos=r.get('todos', False),
                 motor_prueba=r['motor'], restante_ms=r.get('restante_ms'),
                 **{f'm{i+1}': us for i, us in enumerate(motores)})
        campos_imu = ('pitch','roll','yaw','gx','gy','gz')
        def numero(k):
            return type(r.get(k)) in (int,float) and math.isfinite(r[k])
        d['imu_ok'] = r.get('imu_ok') is True and all(numero(k) for k in campos_imu)
        if d['imu_ok']:
            for k in campos_imu:d[k]=r[k]
        for k in ('vbat_a','vbat_b'):
            if numero(k):d[k]=r[k]
        d['z_valida'] = r.get('z_valida') is True and numero('z_cm') and d['imu_ok']
        if d['z_valida']:d['z_cm']=r['z_cm']
        for k in ('laser_mm','laser_estado','laser_edad_ms'):
            if numero(k):d[k]=r[k]
        return d
