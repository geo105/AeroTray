#!/usr/bin/env python3
"""Puente ESP32 del dron octo <-> ROS 2 (Jazzy) <-> RViz.

Este script ES tu monitor serie: muestra los mensajes '#' del ESP32, te deja
escribir comandos (iniciar, +, -, aterrizar, !, ...) y al mismo tiempo publica
la telemetria en ROS 2 para verla en RViz y guardarla en CSV.

Solo un programa puede abrir el puerto serie a la vez: no abras el monitor de
Arduino mientras corre este puente.

Uso:
  source /opt/ros/jazzy/setup.bash
  python3 herramientas/puente_dron_ros2.py               # autodetecta /dev/ttyUSB*
  python3 herramientas/puente_dron_ros2.py --wifi        # sin cable, por WiFi AP 'DSM-Octo' (clave octo2026)
  python3 herramientas/puente_dron_ros2.py --puerto /dev/ttyUSB0
  python3 herramientas/puente_dron_ros2.py --simular     # sin hardware, para probar RViz

Temas publicados:
  /tf (mundo -> base_link)       actitud del dron
  /dron/imu        sensor_msgs/Imu
  /dron/motores_us std_msgs/Float32MultiArray  [M1..M8] en microsegundos
  /dron/actitud    geometry_msgs/Vector3Stamped (x=roll, y=pitch, z=yaw) en grados, convencion firmware
  /dron/vbat       std_msgs/Float32MultiArray  [nodo A, nodo B] en voltios
  /dron/estado     std_msgs/String
  /dron/marcadores visualization_msgs/MarkerArray (brazos, motores, empuje, texto)
Tema suscrito:
  /dron/comando    std_msgs/String  -> se reenvia al ESP32
"""
import argparse
import csv
import glob
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Vector3Stamped, Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Float32, Bool, String, ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from diagnostico_motores_rviz import DiagnosticoMotores, PINES

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NOMBRES_ESTADO = ["DESARMADO", "ARMADO", "CONTROL", "PRUEBA_MOTOR", "CAL_ESC", "ATERRIZANDO"]
ANG_ESC = [0, 90, 180, 270, 45, 135, 225, 315]  # grados, horario visto desde arriba, M1 = frente
R_BRAZO = 0.35  # m (Rev D)
# Ubicaciones declaradas, sin cambiar la orientacion de los datos de la IMU.
# La distancia al centro NO se midio: ubicacion visual aproximada en el brazo.
BRAZO_IMU = 8
BRAZO_LASER = 1
CAMPOS = ["ms", "estado", "pitch", "roll", "yaw", "gx", "gy", "gz", "sp_pitch", "sp_roll",
          "z_cm", "vbat_a", "vbat_b"] + [f"m{i}" for i in range(1, 9)]


def quat_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def color_pwm(us):
    f = min(max((us - 1000.0) / 800.0, 0.0), 1.0)
    return ColorRGBA(r=f, g=1.0 - 0.7 * f, b=0.1, a=1.0)


class PuenteDron(Node):
    def __init__(self, args):
        super().__init__("puente_dron")
        self.args = args
        self.diag = DiagnosticoMotores() if args.diagnostico_motores else None
        self.sensores = None
        self.fuerzas = None
        self.ultimo_aviso = ''
        self.sombra_proc = None
        self.ultima_q = (0.,0.,0.,1.)
        self.ultima_z = 0.
        self.event_lock = threading.Lock()
        self.tf = TransformBroadcaster(self)
        self.pub_imu = self.create_publisher(Imu, "/dron/imu", 10)
        self.pub_mot = self.create_publisher(Float32MultiArray, "/dron/motores_us", 10)
        self.pub_act = self.create_publisher(Vector3Stamped, "/dron/actitud", 10)
        self.pub_bat = self.create_publisher(Float32MultiArray, "/dron/vbat", 10)
        self.pub_z = self.create_publisher(Float32, '/dron/altura_cm', 10)
        self.pub_z_valida = self.create_publisher(Bool, '/dron/altura_valida', 10)
        self.pub_est = self.create_publisher(String, "/dron/estado", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "/dron/marcadores", 10)
        self.create_subscription(String, "/dron/comando", lambda m: self.enviar(m.data), 10)

        self.lock = threading.Lock()
        self.ser = None
        self.sock = None
        self.destino_udp = (args.ip_wifi, args.puerto_wifi)
        self.ultimo = None
        self.n_tlm = 0
        self.t_ultimo_tlm = 0.0
        self.silencio = False
        self.vivo = True
        self.lector = None

        os.makedirs(os.path.join(RAIZ, "registros"), exist_ok=True)
        prefijo = ('demo_identificacion_motores' if args.simular else 'identificacion_motores') if self.diag else ('demo_rviz' if args.simular else 'vuelo')
        nombre = datetime.now().strftime(prefijo + "_%Y%m%d_%H%M%S_%f.csv")
        self.ruta_csv = os.path.join(RAIZ, "registros", nombre)
        self.f_csv = open(self.ruta_csv, "w", newline="")
        self.w_csv = csv.writer(self.f_csv)
        self.w_csv.writerow(["t_pc_s"] + CAMPOS)
        self.ruta_eventos = self.ruta_csv[:-4] + '.jsonl'
        self.f_eventos = open(self.ruta_eventos, 'w')
        self.evento({'tipo': 'sesion', 'modo': prefijo, 'simulado': args.simular})

        if args.simular:
            self.t0_sim = time.time()
            self.create_timer(0.02, self.paso_simulado)
            print("# [puente] MODO SIMULADO: telemetria inventada, sin ESP32.")
        elif args.wifi:
            self.abrir_wifi()
            self.lector = threading.Thread(target=self.hilo_lectura_wifi, daemon=True)
            self.lector.start()
            if not args.sin_watchdog:
                self.create_timer(0.25, lambda: self.enviar("hb", eco=False))
        else:
            self.abrir_serie()
            self.lector = threading.Thread(target=self.hilo_lectura, daemon=True)
            self.lector.start()
            if not args.sin_watchdog:
                self.create_timer(0.25, lambda: self.enviar("hb", eco=False))
        self.create_timer(1.0, self.estado_periodico)
        if not args.simular:
            self.create_timer(0.5, lambda: self.enviar('ver' if self.diag else 'estado_sensores', eco=False))
        if self.diag:
            self.crear_menus_motores()
        else:
            self.crear_menu_banco()
        print(f"# [puente] registrando CSV en {self.ruta_csv}")

    def evento(self, dato):
        with self.event_lock:
            self.f_eventos.write(json.dumps({'t_pc_s': time.time(), **dato}, ensure_ascii=False, allow_nan=False) + '\n')
            self.f_eventos.flush()

    # ------------------------------------------------------------ wifi / serie
    def abrir_wifi(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)
        print(f"# [puente] MODO WIFI UDP hacia {self.destino_udp[0]}:{self.destino_udp[1]} "
              "(conecta tu PC a la red WiFi 'DSM-Octo', clave 'octo2026').")
        self.enviar('ver' if self.diag else "p 3", eco=False)
        if not self.args.sin_watchdog and not self.diag:
            self.enviar("watchdog 1", eco=False)

    def abrir_serie(self):
        if serial is None:
            sys.exit("Falta pyserial: sudo apt install python3-serial")
        puerto = self.args.puerto
        if not puerto:
            candidatos = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
            if not candidatos:
                sys.exit("No hay /dev/ttyUSB* ni /dev/ttyACM*. Conecta el ESP32 del dron por USB (o usa --wifi).")
            puerto = candidatos[0]
        try:
            self.ser = serial.Serial(puerto, self.args.baud, timeout=0.1, write_timeout=0.5, exclusive=True)
        except serial.SerialException as e:
            sys.exit(f"No se pudo abrir {puerto}: {e}\n"
                     "Si dice 'Permission denied': sudo usermod -aG dialout $USER y cierra/abre sesion.\n"
                     "Si dice 'busy': cierra el monitor serie de Arduino u otro programa.")
        print(f"# [puente] conectado a {puerto} @ {self.args.baud}. El ESP32 se reinicia al abrir el puerto.")
        time.sleep(1.8)
        self.puerto_fijo = puerto
        self.enviar('ver' if self.diag else "p 3", eco=False)
        if not self.args.sin_watchdog and not self.diag:
            self.enviar("watchdog 1", eco=False)

    def enviar(self, texto, eco=True):
        texto = texto.strip()
        if not texto:
            return
        if texto.split()[0] == 'vista_yaw':
            if texto not in ('vista_yaw 0','vista_yaw 1'):
                print('# Usar vista_yaw 1 (giro IMU) o vista_yaw 0 (frente fijo)',flush=True)
                return
            self.args.mostrar_yaw = texto.endswith('1')
            self.evento({'tipo':'vista','mostrar_yaw':self.args.mostrar_yaw})
            print('# Giro en RViz: '+('IMU activado' if self.args.mostrar_yaw else 'frente fijo'),flush=True)
            return
        if texto.split()[0] == 'isaac_sombra':
            try:
                partes = texto.split()
                segundos = float(partes[1]) if len(partes)==2 else 15.
                if len(partes)>2 or not math.isfinite(segundos) or not 1<=segundos<=60:
                    raise ValueError('isaac_sombra segundos(1..60), solo calculo SIN PWM')
                if self.args.simular or not self.diag or not self.diag.vigente() or self.diag.estado['estado']!=0:
                    raise ValueError('Usar con sensores reales y motores DESARMADOS (! primero)')
                if self.sombra_proc is not None and self.sombra_proc.poll() is None:
                    raise ValueError('El observador de Isaac ya está funcionando')
                self.sombra_proc = subprocess.Popen([sys.executable,os.path.join(RAIZ,'herramientas/ver_isaac_sin_pwm.py'),
                                                     '--segundos',str(segundos)])
                print('# Isaac: solo calculo PPO en N; SIN PWM ni validacion de estabilidad fisica',flush=True)
            except ValueError as e:
                print(f'# [puente] {e}',flush=True)
            return
        if self.diag:
            try:
                texto, evento = self.diag.preparar(texto)
                if evento:
                    self.evento({'tipo': 'operador', **evento})
                if texto is None:
                    return
            except (ValueError, OverflowError) as e:
                self.ultimo_aviso = str(e)
                print(f'# [identificación] {e}', flush=True)
                return
        if self.args.simular:
            if eco:
                print(f"# [sim] (comando ignorado en simulacion) {texto}")
            return
        payload = b"!" if texto == "!" else (texto + "\n").encode()
        if eco:
            self.evento({'tipo': 'comando', 'texto': texto})
        with self.lock:
            if self.sock is not None:
                try:
                    self.sock.sendto(payload, self.destino_udp)
                except OSError as e:
                    if eco:
                        print(f"# [puente] ERROR UDP: {e}")
                return
            if self.ser is None:
                return
            try:
                self.ser.write(payload)
            except serial.SerialException as e:
                print(f"# [puente] ERROR escribiendo: {e}")

    def hilo_lectura_wifi(self):
        buf = b""
        while self.vivo:
            try:
                datos, origen = self.sock.recvfrom(1500)
                if origen != self.destino_udp:
                    continue
                buf += datos
            except socket.timeout:
                continue
            except OSError as e:
                if self.vivo:
                    print(f"# [puente] error UDP: {e}")
                break
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                self.procesar_linea(linea.decode(errors="replace").strip())

    def hilo_lectura(self):
        buf = b""
        while self.vivo:
            try:
                if self.ser is None or not self.ser.is_open:
                    time.sleep(0.5)
                    if os.path.exists(self.puerto_fijo):
                        self.ser = serial.Serial(self.puerto_fijo, self.args.baud, timeout=0.1, write_timeout=0.5, exclusive=True)
                        print(f"# [puente] reconectado a {self.puerto_fijo}; no se rearma automáticamente")
                        time.sleep(1.5)
                        buf = b''
                        self.enviar('ver' if self.diag else "p 3", eco=False)
                    continue
                buf += self.ser.read(256)
            except (serial.SerialException, OSError) as e:
                print(f"# [puente] USB desconectado ({e}); esperando reconexion...")
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                if self.diag:
                    self.diag.invalidar()
                self.sensores = None
                time.sleep(1.0)
                continue
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                self.procesar_linea(linea.decode(errors="replace").strip())

    def procesar_linea(self, s):
        if not s:
            return
        if s.startswith('{'):
            try:
                dato = json.loads(s, parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
            except ValueError:
                return
            if not isinstance(dato, dict):
                return
            if dato.get('version') in ('dsm-adc-0.1','dsm-sensores-0.1','dsm-montaje-0.1'):
                self.evento({'tipo':'sensores_detalle','dato':dato})
                print('# Diagnostico sin PWM: '+json.dumps(dato),flush=True)
                return
            if self.diag:
                if self.diag.actualizar(dato):
                    self.ultimo_aviso = ''
                    self.evento({'tipo': 'diagnostico', 'dato': dato})
                    self.publicar(self.diag.telemetria())
                else:
                    self.diag.invalidar()
                    self.ultimo_aviso = 'Firmware incorrecto o diagnóstico inválido: cargar motores'
                return
            if dato.get('version') == 'dsm-octo-0.4-banco':
                self.sensores = (time.monotonic(), dato)
            elif dato.get('version') in ('dsm-ppo-red-banco', 'dsm-control-sim-banco'):
                self.fuerzas = (time.monotonic(), dato)
            self.evento({'tipo': 'telemetria_json', 'dato': dato})
            return
        if self.diag:
            if s.startswith('#'):
                self.evento({'tipo':'mensaje_firmware','texto':s})
                print(s, flush=True)
            return
        partes = s.split(",")
        if len(partes) == len(CAMPOS) and partes[0].isdigit():
            try:
                valores = [float(p) for p in partes]
            except ValueError:
                print(s)
                return
            if not all(math.isfinite(v) for v in valores) or not (0 <= valores[1] < len(NOMBRES_ESTADO) and valores[1].is_integer()):
                return
            self.publicar(dict(zip(CAMPOS, valores)))
        else:
            print(s, flush=True)

    # ------------------------------------------------------------ simulacion
    def paso_simulado(self):
        t = time.time() - self.t0_sim
        if self.diag:
            dato = dict(version='dsm-motores-0.3', ms=int(t*1000), estado=3,
                        motor=int(t//3)%8+1, pwm_us=self.diag.pwm,
                        restante_ms=int((3-t%3)*1000))
            self.diag.actualizar(dato)
            self.publicar(self.diag.telemetria())
            return
        estado = 2 if (t % 20) > 4 else 1
        pitch = 6 * math.sin(0.7 * t)
        roll = 4 * math.sin(0.45 * t + 1)
        base = 1420 if estado == 2 else 1000
        m = []
        for a in ANG_ESC:
            ar = math.radians(a)
            dif = -8 * pitch * math.cos(ar) + 8 * roll * math.sin(ar)
            m.append(base + (dif if estado == 2 else 0))
        self.publicar({"ms": t * 1000, "estado": estado, "pitch": pitch, "roll": roll,
                       "yaw": 10 * math.sin(0.1 * t), "gx": 0, "gy": 0, "gz": 0,
                       "sp_pitch": 0, "sp_roll": 0, "z_cm": 0, "vbat_a": 12.1, "vbat_b": 12.0,
                       **{f"m{i + 1}": m[i] for i in range(8)}})

    # ------------------------------------------------------------ ROS
    def publicar(self, d):
        self.ultimo = d
        self.n_tlm += 1
        self.t_ultimo_tlm = time.time()
        self.w_csv.writerow([f"{time.time():.3f}"] + [d[c] for c in CAMPOS])
        ahora = self.get_clock().now().to_msg()
        self.ultimo_stamp = ahora

        # Convencion firmware: pitch>0 = frente (M1) arriba, roll>0 = derecha (M2) arriba.
        # ROS base_link: x = frente (M1), y = izquierda (M4), z = arriba.
        # Por defecto fijamos yaw=0 en RViz para que M1 apunte siempre al frente y no gire por deriva del giroscopio Z.
        diagnostico = d.get('diagnostico', False)
        actitud_valida = not diagnostico or d.get('imu_ok',False)
        if actitud_valida:
            roll_r = -math.radians(d['roll'])
            pitch_r = -math.radians(d['pitch'])
            yaw_r = math.radians(d['yaw']) * (-1. if self.args.invertir_yaw else 1.) if self.args.mostrar_yaw else 0.
            self.ultima_q = quat_rpy(roll_r,pitch_r,yaw_r)
        q = self.ultima_q

        tf = TransformStamped()
        tf.header.stamp = ahora
        tf.header.frame_id = "mundo"
        tf.child_frame_id = "base_link"
        # El origen representa la pose de despegue. Sin sumar 30 cm ficticios.
        if not diagnostico or d.get('z_valida',False):
            self.ultima_z = d['z_cm']/100.
        z_ok = d.get('z_valida',False) if diagnostico else bool(self.args.simular or (self.sensores and time.monotonic()-self.sensores[0]<2 and self.sensores[1].get('z_valida')))
        self.pub_z_valida.publish(Bool(data=z_ok))
        if z_ok:
            self.pub_z.publish(Float32(data=float(d['z_cm'])))
        tf.transform.translation.z = self.ultima_z
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self.tf.sendTransform(tf)

        imu = Imu()
        imu.header.stamp = ahora
        imu.header.frame_id = "base_link"
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q
        imu.angular_velocity.x = -math.radians(d["gx"])
        imu.angular_velocity.y = -math.radians(d["gy"])
        imu.angular_velocity.z = math.radians(d["gz"])
        if actitud_valida:
            self.pub_imu.publish(imu)

        act = Vector3Stamped()
        act.header.stamp = ahora
        act.vector.x, act.vector.y, act.vector.z = d["roll"], d["pitch"], d["yaw"]
        if actitud_valida:
            self.pub_act.publish(act)

        motores = [d[f"m{i}"] for i in range(1, 9)]
        self.pub_mot.publish(Float32MultiArray(data=[float(x) for x in motores]))
        if all(math.isfinite(d[k]) for k in ('vbat_a','vbat_b')):
            self.pub_bat.publish(Float32MultiArray(data=[float(d["vbat_a"]), float(d["vbat_b"])]))
        est = d.get('estado_texto') or (NOMBRES_ESTADO[int(d["estado"])] if 0 <= int(d["estado"]) < len(NOMBRES_ESTADO) else "?")
        self.pub_est.publish(String(data=est))
        self.pub_mk.publish(self.marcadores(motores, est, d, ahora))

    def marcadores(self, motores, est, d, stamp, desactualizado=False):
        arr = MarkerArray()
        brazos = Marker(type=Marker.LINE_LIST, ns="brazos", id=0, action=Marker.ADD)
        brazos.header.frame_id = "base_link"
        brazos.header.stamp = stamp
        brazos.scale.x = 0.02
        brazos.color = ColorRGBA(r=0.85, g=0.1, b=0.1, a=1.0)
        brazos.pose.orientation.w = 1.0
        for i, a in enumerate(ANG_ESC):
            ar = math.radians(a)
            x, y = R_BRAZO * math.cos(ar), -R_BRAZO * math.sin(ar)
            brazos.points += [Point(), Point(x=x, y=y)]

            mot = Marker(type=Marker.CYLINDER, ns="motores", id=i, action=Marker.ADD)
            mot.header = brazos.header
            mot.pose.position.x, mot.pose.position.y = x, y
            mot.pose.orientation.w = 1.0
            mot.scale.x = mot.scale.y = 0.06
            mot.scale.z = 0.04
            mot.color = ColorRGBA(r=.4, g=.4, b=.4, a=1.) if desactualizado else color_pwm(motores[i])
            arr.markers.append(mot)

            emp = Marker(type=Marker.ARROW, ns="empuje", id=i, action=Marker.ADD)
            emp.header = brazos.header
            largo = max(motores[i] - 1000.0, 0.0) / 1000.0 * 0.4
            emp.points = [Point(x=x, y=y, z=0.03), Point(x=x, y=y, z=0.03 + largo)]
            emp.scale.x, emp.scale.y, emp.scale.z = 0.015, 0.03, 0.03
            emp.color = mot.color
            arr.markers.append(emp)

            txt = Marker(type=Marker.TEXT_VIEW_FACING, ns="nombres", id=i, action=Marker.ADD)
            txt.header = brazos.header
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = 1.18 * x, 1.18 * y, 0.05
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.028
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f"M{i + 1}/GPIO{PINES[i]}\n{motores[i]:.0f}us"
            if self.diag:
                observado = self.diag.observaciones.get(i+1, {}).get('sentido_observado', 'pendiente')
                txt.text += f'\ngiro:{observado}'
            arr.markers.append(txt)
        arr.markers.append(brazos)

        frente = Marker(type=Marker.ARROW, ns="frente", id=0, action=Marker.ADD)
        frente.header = brazos.header
        frente.points = [Point(), Point(x=0.25)]
        frente.scale.x, frente.scale.y, frente.scale.z = 0.02, 0.04, 0.05
        frente.color = ColorRGBA(r=0.1, g=0.5, b=1.0, a=1.0)
        arr.markers.append(frente)

        for indice, (nombre, brazo, color) in enumerate((
                ('IMU', BRAZO_IMU, (0.1, 0.8, 1.0)),
                ('LASER', BRAZO_LASER, (0.8, 0.3, 1.0)))):
            angulo = math.radians(ANG_ESC[brazo - 1])
            x = .72 * R_BRAZO * math.cos(angulo)
            y = -.72 * R_BRAZO * math.sin(angulo)
            sensor = Marker(type=Marker.CUBE, ns='sensores_montaje', id=indice, action=Marker.ADD)
            sensor.header = brazos.header
            sensor.pose.position.x, sensor.pose.position.y, sensor.pose.position.z = x, y, .04
            sensor.pose.orientation.w = 1.
            sensor.scale.x, sensor.scale.y, sensor.scale.z = .035, .025, .012
            sensor.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.)
            arr.markers.append(sensor)
            etiqueta = Marker(type=Marker.TEXT_VIEW_FACING, ns='sensores_etiquetas', id=indice, action=Marker.ADD)
            etiqueta.header = brazos.header
            etiqueta.pose.position.x, etiqueta.pose.position.y, etiqueta.pose.position.z = x, y, .12
            etiqueta.pose.orientation.w = 1.
            etiqueta.scale.z = .022
            etiqueta.color = sensor.color
            etiqueta.text = f'{nombre}:BRAZO_{brazo}\nubicacion_aproximada'
            arr.markers.append(etiqueta)

        info = Marker(type=Marker.TEXT_VIEW_FACING, ns="info", id=0, action=Marker.ADD)
        info.header.frame_id = "mundo"
        info.header.stamp = stamp
        info.pose.position.z = 0.85
        info.pose.orientation.w = 1.0
        info.scale.z = 0.035
        info.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0)
        if self.diag:
            restante = d.get('restante_ms')
            tiempo = f'{restante/1000:.1f} s restantes' if restante is not None else 'temporizador en firmware'
            seleccionado = 'TODOS' if d.get('todos') else f'M{d["motor_prueba"]}'
            info.text = (f'IDENTIFICACION|SIN_HELICES\n{est}|{seleccionado}|{tiempo}\n'
                         f'Seleccion:{self.diag.pwm}us/{self.diag.segundos:g}s\nClic_derecho:menu_del_motor\n'
                         'Flechas:PWM|Giro:observacion_manual')
            if d.get('imu_ok'):
                info.text += f'\nIMU:OK|P:{d["pitch"]:+.1f}|R:{d["roll"]:+.1f}|Y:{d["yaw"]:+.1f}'
            else:
                info.text += '\nIMU:SIN_DATOS_VALIDOS(pose_conservada)'
            info.text += '\nGiro_vista:'+('IMU' if self.args.mostrar_yaw else 'FRENTE_FIJO')
            if all(math.isfinite(d[k]) for k in ('vbat_a','vbat_b')):
                info.text += f'\nBatA:{d["vbat_a"]:.2f}V|BatB:{d["vbat_b"]:.2f}V'
            if d.get('z_valida'):
                info.text += f'\nZ:{d["z_cm"]:+.1f}cm_desde_cero|Laser:{d.get("laser_mm","?")}mm'
            else:
                info.text += f'\nZ:SIN_DATO(pose_conservada)\nLaser:estado={d.get("laser_estado","?")}|{d.get("laser_mm","?")}mm'
        else:
            info.text = (f"{est}|P:{d['pitch']:+.1f}|R:{d['roll']:+.1f}|Y:{d['yaw']:+.1f}\nZ:{d['z_cm']:+.1f}cm_desde_cero\n"
                         f"BatA:{d['vbat_a']:.1f}V|BatB:{d['vbat_b']:.1f}V\nFlechas:PWM(no_empuje_medido)")
            if self.sensores and time.monotonic()-self.sensores[0] < 2:
                sensores = self.sensores[1]
                info.text += (f'\nObjetivo:{sensores.get("objetivo_cm", 30):g}cm|IMU:{sensores.get("imu_ok")} '
                              f'|Z_valida:{sensores.get("z_valida")}\nMotores_bloqueados:{sensores.get("motores_bloqueados")}')
            else:
                info.text += '\nZ_valida/bloqueo:sin_confirmacion'
            if self.fuerzas and time.monotonic()-self.fuerzas[0] < 1:
                fuerzas = self.fuerzas[1]
                info.text += '\nBanco_SIN_PWM|Fuerzas[N]:\n' + ','.join(f'{f:.2f}' for f in fuerzas.get('fuerzas_N', []))
            objetivo = 30.0
            if self.sensores and time.monotonic()-self.sensores[0] < 2:
                objetivo = float(self.sensores[1].get('objetivo_cm', objetivo))
            referencia = Marker(type=Marker.LINE_STRIP, ns='objetivo_altura', id=0, action=Marker.ADD)
            referencia.header = info.header
            referencia.pose.orientation.w = 1.0
            referencia.scale.x = .008
            referencia.color = ColorRGBA(r=.1, g=.8, b=1., a=1.)
            referencia.points = [Point(x=.45*math.cos(i*math.tau/64), y=.45*math.sin(i*math.tau/64), z=objetivo/100) for i in range(65)]
            arr.markers.append(referencia)
        if self.args.simular:
            info.text = 'DEMO|DATOS_INVENTADOS|SIN_HARDWARE\n' + info.text
        info.text += '\nMontaje:IMU_M8|LASER_M1\nZ:en_punto_laser(sin_brazo_de_palanca)'
        if desactualizado:
            info.text = 'SIN_TELEMETRIA_RECIENTE\nULTIMOS_VALORES(no_estado_actual)\n' + info.text
            info.color = ColorRGBA(r=1., g=.15, b=.1, a=1.)
        if self.ultimo_aviso:
            import textwrap
            info.text += '\n' + '\n'.join(textwrap.wrap(self.ultimo_aviso, 42))
        # En este equipo OGRE dibuja los espacios con un avance excesivo.
        # Usar separadores visibles mantiene las etiquetas dentro de la vista.
        info.text = info.text.replace(' ', '_')
        arr.markers.append(info)
        return arr

    def crear_menus_motores(self):
        from interactive_markers.interactive_marker_server import InteractiveMarkerServer
        from interactive_markers.menu_handler import MenuHandler
        from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
        self.menu_server = InteractiveMarkerServer(self, '/dron/identificacion')
        self.menus = []
        for n, angulo in enumerate(ANG_ESC, 1):
            marcador = InteractiveMarker()
            marcador.header.frame_id = 'base_link'
            marcador.name = f'identificar_M{n}'
            marcador.description = f'M{n}: menu identificacion'
            marcador.pose.position.x = R_BRAZO * math.cos(math.radians(angulo))
            marcador.pose.position.y = -R_BRAZO * math.sin(math.radians(angulo))
            marcador.pose.position.z = .025
            marcador.pose.orientation.w = 1.
            marcador.scale = .12
            control = InteractiveMarkerControl()
            control.interaction_mode = InteractiveMarkerControl.MENU
            control.always_visible = True
            esfera = Marker(type=Marker.SPHERE)
            esfera.pose.orientation.w = 1.
            esfera.scale.x = esfera.scale.y = esfera.scale.z = .07
            esfera.color = ColorRGBA(r=.2, g=.6, b=1., a=.45)
            control.markers.append(esfera)
            marcador.controls.append(control)
            self.menu_server.insert(marcador)
            menu = MenuHandler()
            menu.insert('Armar ESC a 1000 us (esperar LISTO)', callback=lambda _: self.enviar('armar'))
            menu.insert(f'Probar M{n} con PWM y tiempo seleccionados', callback=lambda _, n=n: self.enviar(f'probar {n}'))
            menu.insert('Probar TODOS con PWM y tiempo seleccionados (sin helices)', callback=lambda _: self.enviar('todos'))
            submenu = menu.insert('Elegir PWM (us), no arranca el motor')
            for us in (1000, 1100, 1200, 1300, 1400, 1500, 1600, 1800, 1950):
                menu.insert(str(us), parent=submenu, callback=lambda _, us=us: self.enviar(f'pwm {us}'))
            duracion = menu.insert('Elegir duracion (s)')
            for segundos in (.5, 1, 2, 3, 5):
                menu.insert(str(segundos), parent=duracion, callback=lambda _, s=segundos: self.enviar(f'tiempo {s}'))
            observado = menu.insert('Registrar giro observado desde arriba')
            for giro in ('cw', 'ccw', 'no-gira'):
                menu.insert(giro, parent=observado, callback=lambda _, n=n, g=giro: self.enviar(f'observado {n} {g}'))
            menu.insert('PARAR / desarmar (!)', callback=lambda _: self.enviar('!'))
            menu.insert('Nivelar IMU (desarmado, quieto y horizontal)', callback=lambda _: self.enviar('nivel_imu'))
            menu.insert('Guardar calibracion IMU (desarmado)', callback=lambda _: self.enviar('guardar'))
            menu.insert('Cero de altura (desarmado)', callback=lambda _: self.enviar('cero_z'))
            menu.insert('Ver PPO Isaac 15 s SIN PWM (desarmado)', callback=lambda _: self.enviar('isaac_sombra 15'))
            menu.insert('Leer ADC de baterias (desarmado)', callback=lambda _: self.enviar('ver_adc'))
            menu.insert('Estado IMU / calibracion / laser (desarmado)', callback=lambda _: self.enviar('ver_sensores'))
            menu.insert('Probar / reiniciar laser (desarmado)', callback=lambda _: self.enviar('test_laser'))
            menu.insert('Mostrar giro del cuerpo (yaw IMU, solo vista)', callback=lambda _: self.enviar('vista_yaw 1'))
            menu.insert('Fijar frente en pantalla (solo vista)', callback=lambda _: self.enviar('vista_yaw 0'))
            menu.apply(self.menu_server, marcador.name)
            self.menus.append(menu)
        self.menu_server.applyChanges()

    def crear_menu_banco(self):
        from interactive_markers.interactive_marker_server import InteractiveMarkerServer
        from interactive_markers.menu_handler import MenuHandler
        from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
        self.menu_server = InteractiveMarkerServer(self, '/dron/identificacion')
        marcador = InteractiveMarker()
        marcador.header.frame_id = 'base_link'
        marcador.name = 'banco_sensores'
        marcador.description = 'Control en banco SIN PWM'
        marcador.pose.orientation.w = 1.
        marcador.scale = .15
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.always_visible = True
        centro = Marker(type=Marker.CUBE)
        centro.pose.orientation.w = 1.
        centro.scale.x = centro.scale.y = .10
        centro.scale.z = .04
        centro.color = ColorRGBA(r=.2, g=.7, b=.9, a=1.)
        control.markers.append(centro)
        marcador.controls.append(control)
        self.menu_server.insert(marcador)
        self.menu_banco = MenuHandler()
        for titulo, comando in [('Leer estado de sensores', 'estado_sensores'),
                                ('Nivelar IMU (equipo quieto y nivelado)', 'nivel_imu'),
                                ('Probar laser', 'test_laser'),
                                ('Fijar cero de altura (en pose de despegue)', 'cero_z'),
                                ('Objetivo +30 cm (sin despegue)', 'zref 30'),
                                ('Calcular PI-D SIN PWM', 'sim_control 1'),
                                ('Calcular PPO SIN PWM (modelo guiado)', 'ppo_control 1'),
                                ('PARAR (!)', '!')]:
            self.menu_banco.insert(titulo, callback=lambda _, c=comando: self.enviar(c))
        self.menu_banco.apply(self.menu_server, marcador.name)
        self.menu_server.applyChanges()

    def estado_periodico(self):
        if self.ultimo is None:
            if not self.args.simular:
                print("# [puente] aun sin telemetria (normal unos segundos tras conectar)")
            if self.diag:
                msg = Marker(type=Marker.TEXT_VIEW_FACING, ns='info', id=0, action=Marker.ADD)
                msg.header.frame_id = 'mundo'
                msg.pose.position.z = .6
                msg.pose.orientation.w = 1.
                msg.scale.z = .04
                msg.color = ColorRGBA(r=1., g=.2, b=.1, a=1.)
                import textwrap
                msg.text = 'SIN_DIAGNOSTICO_DE_MOTORES\nArmado/pruebas:BLOQUEADOS\n' + '\n'.join(textwrap.wrap(self.ultimo_aviso, 36)).replace(' ', '_')
                self.pub_mk.publish(MarkerArray(markers=[msg]))
            return
        self.f_csv.flush()
        d = self.ultimo
        if time.time() - self.t_ultimo_tlm > 1.0:
            print("# [puente] AVISO: no llega telemetria desde hace > 1 s")
        est = d.get('estado_texto') or NOMBRES_ESTADO[int(d['estado'])]
        self.pub_mk.publish(self.marcadores([d[f'm{i}'] for i in range(1,9)], est, d,
                                          self.ultimo_stamp, time.time()-self.t_ultimo_tlm > 1.0))
        if self.silencio:
            return
        if int(d["estado"]) in (2, 3, 5):
            m = " ".join(f"{d[f'm{i}']:.0f}" for i in range(1, 9))
            if self.diag:
                print(f'[identificacion] {est} | M1..M8 [us]: {m} | giro: observacion humana', flush=True)
                return
            print(f"[tlm] {est} p={d['pitch']:+.1f} r={d['roll']:+.1f} "
                  f"y={d['yaw']:+.0f} | A={d['vbat_a']:.2f}V B={d['vbat_b']:.2f}V | M1..M8 {m}", flush=True)

    def cerrar(self):
        self.vivo = False
        if self.sombra_proc is not None and self.sombra_proc.poll() is None:
            self.sombra_proc.terminate()
        if self.sock is not None:
            try:
                self.sock.sendto(b"!", self.destino_udp)
            except Exception:
                pass
            self.sock.close()
        if self.ser is not None:
            try:
                self.ser.write(b"!")
            except Exception:
                pass
            self.ser.close()
        if self.lector is not None:
            self.lector.join(timeout=2.0)
        if hasattr(self,'menu_server'):
            self.menu_server.shutdown()
        self.f_csv.close()
        self.evento({'tipo': 'cierre', 'parada_solicitada': True})
        self.f_eventos.close()
        print(f"# [puente] CSV guardado: {self.ruta_csv}")


def hilo_teclado(nodo):
    print("# [puente] Escribe comandos del ESP32 y Enter ('?' ayuda, '!' parada). "
          "'silencio' oculta [tlm], 'salir' cierra (manda '!').")
    if nodo.diag:
        print('# Identificación SIN HÉLICES: armar; esperar LISTO; probar 1 [1200 [2]]; '
              'todos [1200 [2]]; pwm 1250; tiempo 2; observado 1 cw; !. También: clic derecho en cada motor en RViz.')
    for linea in sys.stdin:
        linea = linea.strip()
        if linea == "salir":
            break
        if linea == "silencio":
            nodo.silencio = not nodo.silencio
            print(f"# [puente] lineas [tlm] {'ocultas' if nodo.silencio else 'visibles'}")
            continue
        nodo.enviar(linea)
    nodo.vivo = False
    rclpy.try_shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", default="")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wifi", action="store_true", help="usar WiFi UDP (red 'DSM-Octo', clave 'octo2026', 192.168.4.1:4210)")
    ap.add_argument("--ip-wifi", default="192.168.4.1")
    ap.add_argument("--puerto-wifi", type=int, default=4210)
    ap.add_argument("--simular", action="store_true", help="telemetria falsa para probar RViz sin hardware")
    ap.add_argument("--sin-watchdog", action="store_true", help="no mandar 'hb' ni activar watchdog en el ESP32")
    ap.add_argument("--mostrar-yaw", action="store_true", help="rotar tambien en Z (yaw) dentro de RViz")
    ap.add_argument("--invertir-yaw", action="store_true")
    ap.add_argument('--diagnostico-motores', action='store_true', help='identificar motores sin hélices; diagnóstico independiente, con menús RViz')
    args = ap.parse_args()
    if args.diagnostico_motores and args.sin_watchdog:
        ap.error('El diagnóstico requiere latido; no admite --sin-watchdog')
    if args.diagnostico_motores and not (args.puerto or args.wifi or args.simular):
        ap.error('Indica --puerto del NodeMCU o --wifi; no se autodetecta durante la identificación')

    rclpy.init()
    nodo = PuenteDron(args)
    threading.Thread(target=hilo_teclado, args=(nodo,), daemon=True).start()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        nodo.cerrar()
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
