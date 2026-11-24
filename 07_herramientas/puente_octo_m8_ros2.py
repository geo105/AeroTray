#!/usr/bin/env python3
"""Puente del dron octo (firmware captura_octo_pid_m8_suave y su variante con laser) -> ROS 2 -> RViz.

SOLO ESCUCHA: nunca manda nada por su cuenta al ESP32. Ese firmware apaga los motores si recibe una
orden que no conoce, asi que NO se debe usar el puente antiguo (puente_dron_ros2.py) con el.
Lo unico que se envia es lo que escribes aqui (cal, vista, arm BANCO_FIJO, iniciar, marca, parar, log)
y '!' para parar.

Uso:
  source /opt/ros/jazzy/setup.bash
  python3 herramientas/puente_octo_m8_ros2.py                      # dron por USB (autodetecta)
  python3 herramientas/puente_octo_m8_ros2.py --puerto /dev/ttyUSB0
  python3 herramientas/puente_octo_m8_ros2.py --repetir archivo.txt  # reproduce un registro grabado

Convencion (la del firmware): +X = M8 (frente), +Y = M7 (izquierda), +Z arriba. Bajar M8 = pitch +.

Temas:
  /tf                 mundo -> base_link (inclinacion y altura del laser), base_link -> laser_link
  /dron/imu           sensor_msgs/Imu
  /dron/actitud       Vector3Stamped  x=roll y=pitch z=yaw [grados]
  /dron/motores_us    Float32MultiArray [M1..M8]
  /dron/accel_g       Float32   modulo de la aceleracion [g]
  /dron/escala_rp     Float32   1 = sin recorte del mezclador
  /dron/estado        String    OFF / VISTA / ARMADO / RAMPA / CONTROL
  /dron/laser         sensor_msgs/Range [m]   (solo con el firmware que tiene laser)
  /dron/altura_cm     Float32   altura sobre el cero del laser
  /dron/marcadores    MarkerArray (brazos, motores, empuje, haz del laser, texto, huella de inclinacion)
"""
import argparse
import glob
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped, Vector3Stamped
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import ColorRGBA, Float32, Float32MultiArray, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
L, D = 0.350, 0.24748737
POS_X = [D, -D, -D, D, 0.0, -L, 0.0, L]          # igual que el firmware, M1..M8
POS_Y = [-D, -D, D, D, -L, 0.0, L, 0.0]
PINES = [25, 26, 27, 14, 21, 19, 5, 17]
FASES = ("OFF", "VISTA", "ARMADO", "RAMPA", "CONTROL")
N_BASE = 19                                  # columnas del firmware sin laser
CORTE_INCLINACION = 12.0


def quat_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def color_pwm(us):
    f = min(max((us - 1180.0) / 320.0, 0.0), 1.0)      # 1180..1500 us: verde -> rojo
    return ColorRGBA(r=f, g=1.0 - 0.7 * f, b=0.1, a=1.0)


def interpretar(linea):
    """Devuelve un dict con la telemetria, o None si la linea no es telemetria."""
    p = linea.split(",")
    if len(p) not in (N_BASE, N_BASE + 2) or p[1] not in FASES or not p[0].isdigit():
        return None
    try:
        v = [float(x) for x in p[2:]]
    except ValueError:
        return None
    if not all(math.isfinite(x) for x in v):
        return None
    d = {"ms": int(p[0]), "fase": p[1], "roll": v[0], "pitch": v[1], "yaw": v[2],
         "p": v[3], "q": v[4], "r": v[5], "escala_rp": v[6], "escala_yaw": v[7],
         "motores": v[8:16], "accel_g": v[16], "laser_mm": None, "laser_ok": False}
    if len(p) == N_BASE + 2:
        d["laser_mm"], d["laser_ok"] = v[17], v[18] > 0.5
    return d


class Puente(Node):
    def __init__(self, args):
        super().__init__("puente_octo_m8")
        self.args = args
        self.tf = TransformBroadcaster(self)
        self.pub_imu = self.create_publisher(Imu, "/dron/imu", 10)
        self.pub_act = self.create_publisher(Vector3Stamped, "/dron/actitud", 10)
        self.pub_mot = self.create_publisher(Float32MultiArray, "/dron/motores_us", 10)
        self.pub_acc = self.create_publisher(Float32, "/dron/accel_g", 10)
        self.pub_esc = self.create_publisher(Float32, "/dron/escala_rp", 10)
        self.pub_est = self.create_publisher(String, "/dron/estado", 10)
        self.pub_laser = self.create_publisher(Range, "/dron/laser", 10)
        self.pub_z = self.create_publisher(Float32, "/dron/altura_cm", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "/dron/marcadores", 10)
        self.ser = None
        self.vivo = True
        self.ultimo = None
        self.t_ultimo = 0.0
        self.n = 0
        self.cero_mm = None                    # primera lectura valida del laser = altura 0
        self.z = 0.0
        self.mm_min = self.mm_max = None       # minimo y maximo que ha medido el laser
        self.huella = deque(maxlen=300)        # ultimos 30 s de (roll, pitch)
        self.ultimo_evento = ""
        os.makedirs(os.path.join(RAIZ, "registros", "octo_m8"), exist_ok=True)
        self.ruta = os.path.join(RAIZ, "registros", "octo_m8",
                                 datetime.now().strftime("octo_m8_%Y%m%d_%H%M%S.txt"))
        self.f = None if args.repetir else open(self.ruta, "w")
        if args.repetir:
            threading.Thread(target=self.hilo_repetir, daemon=True).start()
            print(f"# [puente] reproduciendo {args.repetir} (sin ESP32)")
        else:
            self.abrir()
            threading.Thread(target=self.hilo_serie, daemon=True).start()
            print(f"# [puente] guardando todo en {self.ruta}")
        self.t_inicio = time.time()
        self.paso_vista = 0 if (args.vista and not args.repetir) else 9
        self.create_timer(1.0, self.vigilar)

    # ------------------------------------------------------------ entrada
    def abrir(self):
        import serial
        puerto = self.args.puerto or next(iter(sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))), "")
        if not puerto:
            sys.exit("No hay ningun /dev/ttyUSB* ni /dev/ttyACM*. Conecta el ESP32 del dron o usa --puerto.")
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = puerto, self.args.baud, 0.2
        s.dtr = s.rts = False                  # no reiniciar el ESP32 al conectar
        s.open()
        self.ser = s
        print(f"# [puente] conectado a {puerto} a {self.args.baud} baudios. Este puente no envia nada solo.")

    def hilo_serie(self):
        buf = b""
        while self.vivo:
            try:
                dato = self.ser.read(4096)
            except Exception as e:             # cable desconectado
                print(f"# [puente] se perdio el puerto serie: {e}")
                self.vivo = False
                return
            if not dato:
                continue
            buf += dato
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                self.linea(linea.decode("utf-8", "replace").strip())

    def hilo_repetir(self):
        t_prev = None
        while self.vivo:
            for linea in open(self.args.repetir, errors="replace"):
                if not self.vivo:
                    return
                linea = linea.strip()
                d = interpretar(linea)
                if d:
                    if t_prev is not None and 0 < d["ms"] - t_prev < 2000:
                        time.sleep((d["ms"] - t_prev) / 1000.0 / self.args.velocidad)
                    t_prev = d["ms"]
                self.linea(linea)
            if not self.args.bucle:
                print("# [puente] fin del registro (Ctrl+C para cerrar)")
                return
            t_prev = None

    def enviar(self, texto):
        if not self.ser:
            print("# [puente] reproduciendo un registro: no hay ESP32 al que enviar")
            return
        self.ser.write(b"!" if texto == "!" else texto.encode() + b"\n")
        if self.f:
            self.f.write(f"# [pc] > {texto}\n")

    # ------------------------------------------------------------ salida
    def linea(self, s):
        if not s:
            return
        if self.f:
            self.f.write(s + "\n")
        d = interpretar(s)
        if d is None:
            if s.startswith("#"):
                self.ultimo_evento = s[1:].strip()
                if self.paso_vista == 1 and "CAL OK" in s:      # --vista: tras calibrar, pedir la vista
                    self.paso_vista = 9
                    self.enviar("vista")
                    print("# [puente] vista pedida: ya puedes mover el dron (motores apagados, 1000 us)")
            if not s.startswith("t_us,") and s.count(",") < 15:   # no llenar la pantalla con el volcado del log
                print(s, flush=True)
            return
        self.publicar(d)

    def publicar(self, d):
        self.ultimo, self.t_ultimo = d, time.time()
        self.n += 1
        ahora = self.get_clock().now().to_msg()
        rad = math.radians
        q = quat_rpy(rad(d["roll"]), rad(d["pitch"]), rad(d["yaw"]) if self.args.mostrar_yaw else 0.0)

        if d["laser_ok"]:
            # Altura vertical del centro: el laser esta en el brazo 8 y se inclina con el dron.
            cr, cp = math.cos(rad(d["roll"])), math.cos(rad(d["pitch"]))
            vertical = (d["laser_mm"] * cr * cp + 1000.0 * (self.args.laser_x * math.sin(rad(d["pitch"]))
                                                           - self.args.laser_y * math.sin(rad(d["roll"])) * cp))
            if self.cero_mm is None:
                self.cero_mm = vertical
            self.z = (vertical - self.cero_mm) / 1000.0
            self.mm_min = d["laser_mm"] if self.mm_min is None else min(self.mm_min, d["laser_mm"])
            self.mm_max = d["laser_mm"] if self.mm_max is None else max(self.mm_max, d["laser_mm"])
            self.pub_z.publish(Float32(data=self.z * 100.0))
        if d["laser_mm"] is not None:
            r = Range()
            r.header.stamp, r.header.frame_id = ahora, "laser_link"
            r.radiation_type, r.field_of_view = Range.INFRARED, 0.44        # VL53L0X: cono de 25 grados
            r.min_range, r.max_range = 0.03, 2.0
            r.range = d["laser_mm"] / 1000.0 if d["laser_ok"] else float("inf")
            self.pub_laser.publish(r)

        t1 = TransformStamped()
        t1.header.stamp, t1.header.frame_id, t1.child_frame_id = ahora, "mundo", "base_link"
        t1.transform.translation.z = self.args.altura_base + self.z
        t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w = q
        t2 = TransformStamped()                # el laser mira hacia abajo: su eje X apunta a -Z del dron
        t2.header.stamp, t2.header.frame_id, t2.child_frame_id = ahora, "base_link", "laser_link"
        t2.transform.translation.x, t2.transform.translation.y = self.args.laser_x, self.args.laser_y
        t2.transform.translation.z = -0.03     # cuelga debajo del brazo
        t2.transform.rotation.y, t2.transform.rotation.w = math.sqrt(0.5), math.sqrt(0.5)
        self.tf.sendTransform([t1, t2])

        imu = Imu()
        imu.header.stamp, imu.header.frame_id = ahora, "base_link"
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = rad(d["p"]), rad(d["q"]), rad(d["r"])
        imu.linear_acceleration_covariance[0] = -1.0     # el firmware solo manda el modulo
        self.pub_imu.publish(imu)
        a = Vector3Stamped()
        a.header.stamp = ahora
        a.vector.x, a.vector.y, a.vector.z = d["roll"], d["pitch"], d["yaw"]
        self.pub_act.publish(a)
        self.pub_mot.publish(Float32MultiArray(data=[float(m) for m in d["motores"]]))
        self.pub_acc.publish(Float32(data=d["accel_g"]))
        self.pub_esc.publish(Float32(data=d["escala_rp"]))
        self.pub_est.publish(String(data=d["fase"]))
        self.huella.append((d["roll"], d["pitch"]))
        self.pub_mk.publish(self.marcadores(d, ahora))

    def marcadores(self, d, stamp, viejo=False):
        arr = MarkerArray()

        def nuevo(tipo, ns, i, marco="base_link"):
            m = Marker(type=tipo, ns=ns, id=i, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = marco, stamp
            m.pose.orientation.w = 1.0
            arr.markers.append(m)
            return m

        gris = ColorRGBA(r=.4, g=.4, b=.4, a=1.)
        brazos = nuevo(Marker.LINE_LIST, "brazos", 0)
        brazos.scale.x = 0.02
        brazos.color = ColorRGBA(r=0.85, g=0.1, b=0.1, a=1.0)
        for i in range(8):
            x, y, us = POS_X[i], POS_Y[i], d["motores"][i]
            brazos.points += [Point(), Point(x=x, y=y)]
            mot = nuevo(Marker.CYLINDER, "motores", i)
            mot.pose.position.x, mot.pose.position.y = x, y
            mot.scale.x = mot.scale.y = 0.06
            mot.scale.z = 0.04
            mot.color = gris if viejo else color_pwm(us)
            emp = nuevo(Marker.ARROW, "empuje", i)
            largo = max(us - 1000.0, 0.0) / 1000.0 * 0.6
            emp.points = [Point(x=x, y=y, z=0.03), Point(x=x, y=y, z=0.03 + largo)]
            emp.scale.x, emp.scale.y, emp.scale.z = 0.015, 0.03, 0.03
            emp.color = mot.color
            txt = nuevo(Marker.TEXT_VIEW_FACING, "nombres", i)
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = 1.18 * x, 1.18 * y, 0.05
            txt.scale.z = 0.028
            txt.color = ColorRGBA(r=1., g=1., b=1., a=1.)
            txt.text = f"M{i + 1}/GPIO{PINES[i]}\n{us:.0f}us"
        frente = nuevo(Marker.ARROW, "frente", 0)
        frente.points = [Point(), Point(x=0.25)]
        frente.scale.x, frente.scale.y, frente.scale.z = 0.02, 0.04, 0.05
        frente.color = ColorRGBA(r=0.1, g=0.5, b=1.0, a=1.0)
        imu = nuevo(Marker.CUBE, "imu", 0)       # IMU y laser: mismo soporte, raiz del brazo de M8, por debajo
        imu.pose.position.x, imu.pose.position.y, imu.pose.position.z = self.args.laser_x, self.args.laser_y, -0.02
        rot = nuevo(Marker.TEXT_VIEW_FACING, "imu_nombre", 0)
        rot.pose.position.x, rot.pose.position.y, rot.pose.position.z = self.args.laser_x, self.args.laser_y, -0.07
        rot.scale.z = 0.025
        rot.color = ColorRGBA(r=.1, g=.8, b=1., a=1.)
        rot.text = "IMU + LASER"
        imu.scale.x, imu.scale.y, imu.scale.z = .035, .025, .012
        imu.color = ColorRGBA(r=.1, g=.8, b=1., a=1.)

        # Haz del laser: flecha desde el sensor hasta donde mide (en su propio marco, eje X).
        haz = nuevo(Marker.ARROW, "laser", 0, "laser_link")
        if d["laser_mm"] is not None and d["laser_ok"]:
            haz.points = [Point(), Point(x=float(d["laser_mm"]) / 1000.0)]
            haz.scale.x, haz.scale.y, haz.scale.z = 0.008, 0.02, 0.03
            haz.color = ColorRGBA(r=.9, g=.2, b=1., a=1.)
        else:
            haz.action = Marker.DELETE

        # Huella: punta del eje vertical del dron proyectada, para ver cuanto y hacia donde se inclina.
        # Escala: 1 grado = 2 cm. El circulo rojo es el corte de 12 grados.
        z0 = 0.002
        huella = nuevo(Marker.LINE_STRIP, "huella", 0, "mundo")
        huella.scale.x = 0.004
        huella.color = ColorRGBA(r=1., g=.9, b=.1, a=.9)
        huella.points = [Point(x=float(0.02 * p), y=float(-0.02 * r), z=z0) for r, p in self.huella]
        for i, (grados, col) in enumerate(((2., (.2, .9, .2)), (5., (1., .7, .1)), (CORTE_INCLINACION, (1., .1, .1)))):
            c = nuevo(Marker.LINE_STRIP, "limites", i, "mundo")
            c.scale.x = 0.003
            c.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=.8)
            c.points = [Point(x=0.02 * grados * math.cos(k * math.pi / 24), y=0.02 * grados * math.sin(k * math.pi / 24), z=z0)
                        for k in range(49)]

        info = nuevo(Marker.TEXT_VIEW_FACING, "info", 0, "mundo")
        info.pose.position.z = self.args.altura_base + 0.55
        info.scale.z = 0.045
        incl = math.degrees(math.acos(max(-1., min(1., math.cos(math.radians(d["roll"])) * math.cos(math.radians(d["pitch"]))))))
        alerta = viejo or incl > 8 or d["escala_rp"] < 0.999 or not 0.55 < d["accel_g"] < 1.55
        info.color = ColorRGBA(r=1., g=.3, b=.2, a=1.) if alerta else ColorRGBA(r=1., g=1., b=1., a=1.)
        laser = "sin laser en este firmware" if d["laser_mm"] is None else (
            f"{d['laser_mm']:.0f} mm  (altura {self.z * 100:+.1f} cm)\nmin {self.mm_min:.0f} mm   max {self.mm_max:.0f} mm   "
            f"recorrido {self.mm_max - self.mm_min:.0f} mm" if d["laser_ok"] and self.mm_min is not None else "sin lectura valida")
        info.text = (("SIN DATOS\n" if viejo else "") +
                     f"{d['fase']}\nroll {d['roll']:+.2f}  pitch {d['pitch']:+.2f}  incl {incl:.2f} (corte {CORTE_INCLINACION:.0f})\n"
                     f"acel {d['accel_g']:.3f} g (corte 0.55-1.55)  recorte RP {d['escala_rp']:.2f}\n"
                     f"laser: {laser}\n{self.ultimo_evento[:60]}")
        return arr

    def vigilar(self):
        if self.f:
            self.f.flush()
        if self.paso_vista == 0 and time.time() - self.t_inicio > 5.0:
            self.paso_vista = 1
            self.enviar("cal")
            print("# [puente] --vista: calibrando 5 s. NO MUEVAS EL DRON, dejalo nivelado.")
        if self.ultimo is None and self.paso_vista == 9:
            print("# [puente] aun sin telemetria. El firmware solo la manda en VISTA, ARMADO, RAMPA o CONTROL "
                  "(escribe cal, espera 5 s, y luego vista).")
            return
        viejo = self.ultimo is not None and time.time() - self.t_ultimo > 1.0
        if self.ultimo is None:
            return
        if viejo:
            self.pub_mk.publish(self.marcadores(self.ultimo, self.get_clock().now().to_msg(), True))
        elif self.args.resumen:
            d = self.ultimo
            print(f"[tlm] {d['fase']} roll {d['roll']:+.2f} pitch {d['pitch']:+.2f} acel {d['accel_g']:.3f} g "
                  f"| M1..M8 {' '.join('%.0f' % m for m in d['motores'])}"
                  + (f" | laser {d['laser_mm']:.0f} mm (min {self.mm_min:.0f}, max {self.mm_max:.0f})"
                     if d["laser_ok"] and self.mm_min is not None else
                     (" | laser sin lectura valida" if d["laser_mm"] is not None else "")), flush=True)


def hilo_teclado(nodo):
    for linea in sys.stdin:
        texto = linea.strip()
        if texto in ("salir", "exit"):
            nodo.vivo = False
            return
        if texto == "cero":
            nodo.cero_mm = nodo.mm_min = nodo.mm_max = None
            print("# [puente] la siguiente lectura del laser sera la altura 0; minimo y maximo borrados")
        elif texto:
            nodo.enviar(texto)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", default="")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--repetir", default="", help="reproducir un TXT grabado en vez de conectar al ESP32")
    ap.add_argument("--velocidad", type=float, default=1.0, help="velocidad de reproduccion")
    ap.add_argument("--bucle", action="store_true", help="repetir el registro sin parar")
    ap.add_argument("--mostrar-yaw", action="store_true", help="girar tambien en Z (yaw relativo, deriva)")
    ap.add_argument("--vista", action="store_true",
                    help="enviar 'cal' y luego 'vista' al conectar, para ver los sensores con los motores apagados")
    ap.add_argument("--resumen", action="store_true", help="imprimir una linea de telemetria por segundo")
    ap.add_argument("--altura-base", type=float, default=0.30, help="altura a la que se dibuja el dron [m]")
    ap.add_argument("--laser-x", type=float, default=0.12, help="posicion del laser en el dron, hacia M8 [m]")
    ap.add_argument("--laser-y", type=float, default=0.0, help="posicion del laser en el dron, hacia M7 [m]")
    args = ap.parse_args()
    rclpy.init()
    nodo = Puente(args)
    threading.Thread(target=hilo_teclado, args=(nodo,), daemon=True).start()
    print("# Escribe aqui las ordenes del dron (cal, vista, arm BANCO_FIJO, iniciar, marca, parar, log). "
          "'!' = parada. 'cero' = poner el laser en 0. 'salir' = cerrar.")
    try:
        while rclpy.ok() and nodo.vivo:
            rclpy.spin_once(nodo, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    nodo.vivo = False
    if nodo.f:
        nodo.f.close()
    if nodo.ser:
        nodo.ser.close()


if __name__ == "__main__":
    main()
