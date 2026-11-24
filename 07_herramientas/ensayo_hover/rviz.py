"""Visualizacion M8. La altura del dibujo es fija y NO una medicion."""
import math
import time
from collections import deque
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from sensor_msgs.msg import Imu, Range
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import puente_octo_m8_ros2 as base

base.CORTE_INCLINACION = 15.0


class Display(base.Puente):
    def __init__(self):
        Node.__init__(self, "ensayo_hover_1600")
        self.args = SimpleNamespace(altura_base=.30, laser_x=.12, laser_y=0., mostrar_yaw=False)
        self.tf = TransformBroadcaster(self)
        pubs = {"imu": (Imu, "imu"), "act": (Vector3Stamped, "actitud"),
                "mot": (Float32MultiArray, "motores_us"), "acc": (Float32, "accel_g"),
                "esc": (Float32, "escala_rp"), "est": (String, "estado"),
                "laser": (Range, "laser"), "z": (Float32, "altura_cm"),
                "mk": (MarkerArray, "marcadores"), "altura_valida": (Bool, "altura_valida"),
                "acc_filtrada": (Float32,"accel_filtrada_g")}
        for attr, (cls, name) in pubs.items():
            setattr(self, "pub_"+attr, self.create_publisher(cls, "/dron/"+name, 10))
        self.n = 0
        self.cero_mm = self.mm_min = self.mm_max = None
        self.z = 0.
        self.huella = deque(maxlen=300)
        self.ultimo_evento = ""
        self.cal_ok = False
        self.datos = None

    def update(self, v, stale=False):
        self.cal_ok = bool(v[7] & 1)
        self.datos = v
        d = {"ms": v[4], "fase": ("SIN DATOS: ULTIMA MUESTRA" if stale else base.FASES[v[6]]),
             "roll": v[10] if self.cal_ok else 0., "pitch": v[11] if self.cal_ok else 0.,
             "yaw": v[12] if self.cal_ok else 0., "p": v[13], "q": v[14], "r": v[15],
             "escala_rp": v[22], "escala_yaw": v[23], "motores": v[24:32],
             "accel_g": math.sqrt(sum(n*n for n in v[16:19]))/16384,
             "laser_mm": None, "laser_ok": False}
        self.stale = stale
        self.publicar(d)
        self.pub_altura_valida.publish(Bool(data=False))
        self.pub_z.publish(Float32(data=float("nan")))
        self.pub_acc_filtrada.publish(Float32(data=float(v[44]) if v[2]==2 else float("nan")))
        rclpy.spin_once(self, timeout_sec=0)

    def publicar(self, d):
        self.ultimo, self.t_ultimo = d, time.monotonic()
        stamp = self.get_clock().now().to_msg()
        q = base.quat_rpy(math.radians(d["roll"]), math.radians(d["pitch"]), 0.)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "mundo", "base_link"
        t.transform.translation.z = self.args.altura_base  # dibujo fijo, no altura medida
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf.sendTransform(t)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, "base_link"
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q
        if not self.cal_ok or self.stale:
            imu.orientation_covariance[0] = -1.
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = [math.radians(d[k]) for k in ("p", "q", "r")]
        # Aceleracion raw se conserva en CSV. No publicar vector en marco M8 sin corregir.
        imu.linear_acceleration_covariance[0] = -1.
        self.pub_imu.publish(imu)
        a = Vector3Stamped()
        a.header.stamp, a.header.frame_id = stamp, "base_link"
        a.vector.x, a.vector.y, a.vector.z = ([d[k] for k in ("roll", "pitch", "yaw")]
                                           if self.cal_ok and not self.stale else [float("nan")]*3)
        self.pub_act.publish(a)
        self.pub_mot.publish(Float32MultiArray(data=[float(p) for p in d["motores"]]))
        self.pub_acc.publish(Float32(data=d["accel_g"]))
        self.pub_esc.publish(Float32(data=d["escala_rp"]))
        self.pub_est.publish(String(data=d["fase"]))
        if self.cal_ok and not self.stale:
            self.huella.append((d["roll"], d["pitch"]))
        self.pub_mk.publish(self.marcadores(d, stamp))

    def marcadores(self, d, stamp, viejo=False):
        arr = base.Puente.marcadores(self, d, stamp, viejo or self.stale)
        for m in arr.markers:
            if m.ns == "imu_nombre":
                m.text = "IMU: brazo M8\nposicion aproximada"
            elif m.ns == "laser":
                m.action = Marker.DELETE
            elif m.ns == "info":
                v = self.datos
                if v[2]==2:
                    m.text=m.text.replace("(corte 0.55-1.55)","(proteccion ACCEL rev2)")
                    m.text+=(f"\nACCEL raw {v[43]:.2f} g / filt {v[44]:.2f} g; fuera {v[45]/1000:.0f} ms"
                             "\nFiltro: 0.55-1.55 g durante 40 ms; inmediato raw <=0.20 / >=1.90 g")
                m.text += (f"\nALTURA INVALIDA: ToF M1 desactivado. Z dibujo fija."
                           f"\nColectivo pedido {v[8]} us; max motor 1600 us. Corte {v[42]}."
                           "\n! / ESPACIO = APAGAR; armar + arrancar = inicio")
                if not self.cal_ok:
                    m.text = "SIN CALIBRAR: angulos no validos\n" + m.text
        tof = Marker(type=Marker.CUBE, ns="tof_m1", id=0, action=Marker.ADD)
        tof.header.frame_id, tof.header.stamp = "base_link", stamp
        tof.pose.orientation.w = 1.
        tof.pose.position.x, tof.pose.position.y, tof.pose.position.z = .12/math.sqrt(2), -.12/math.sqrt(2), -.02
        tof.scale.x, tof.scale.y, tof.scale.z = .025, .025, .012
        tof.color.r, tof.color.b, tof.color.a = 1., 1., 1.
        label = Marker(type=Marker.TEXT_VIEW_FACING, ns="tof_m1", id=1, action=Marker.ADD)
        label.header = tof.header
        label.pose.orientation.w = 1.
        label.pose.position.x, label.pose.position.y, label.pose.position.z = tof.pose.position.x, tof.pose.position.y, -.10
        label.scale.z = .023
        label.color = tof.color
        label.text = "ToF: brazo M1\nDESACTIVADO (pos. aprox.)"
        arr.markers.extend([tof, label])
        return arr
