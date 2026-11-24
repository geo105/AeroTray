#!/usr/bin/env python3
"""Nodo ROS 2 + RViz para el sistema Ball & Plate (DSM).

Ejecuta la visión computacional y el lazo de control PD/PID directamente en el
PC, recibiendo imágenes a 2 MBaud del ESP32-S3-CAM y enviando consignas a los
servos GPIO41 / GPIO42.

Publica:
  /ball_plate/imagen      sensor_msgs/Image (imagen con detecciones)
  /ball_plate/pelota      geometry_msgs/PointStamped (X, Y en metros, Z=0)
  /ball_plate/actitud     geometry_msgs/Vector3Stamped (inclinación en grados)
  /ball_plate/servos      std_msgs/Float32MultiArray [GPIO41, GPIO42] en grados
  /ball_plate/marcadores  visualization_msgs/MarkerArray (bandeja 3D + esfera)
  /tf                     mundo -> bandeja -> pelota

Suscripciones:
  /ball_plate/comando     std_msgs/String (arm, centro, off, !)

Uso:
  source /opt/ros/jazzy/setup.bash
  python3 herramientas/puente_ball_plate_ros2.py --rviz
"""
import argparse
import math
import os
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Vector3Stamped
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Float32MultiArray, Header, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ / "herramientas"))

from vision_ball_plate import FuenteUSB, crear_detector, procesar
from control_ball_plate import ControlPD
from calibracion_ball_plate import cargar, crear_args


def quat_desde_rpy(roll_rad, pitch_rad, yaw_rad=0.0):
    cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
    cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
    cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class PuenteBallPlate(Node):
    def __init__(self, args):
        super().__init__("puente_ball_plate")
        self.args = args
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_img = self.create_publisher(Image, "/ball_plate/imagen", qos)
        self.pub_pelota = self.create_publisher(PointStamped, "/ball_plate/pelota", 10)
        self.pub_actitud = self.create_publisher(Vector3Stamped, "/ball_plate/actitud", 10)
        self.pub_servos = self.create_publisher(Float32MultiArray, "/ball_plate/servos", 10)
        self.pub_marcadores = self.create_publisher(MarkerArray, "/ball_plate/marcadores", 10)
        self.pub_estado = self.create_publisher(String, "/ball_plate/estado", 10)

        self.create_subscription(String, "/ball_plate/comando", self.recibir_comando, 10)

        # Configuración de visión y control
        cfg = cargar()
        self.cfg_vision = crear_args(cfg)
        self.cfg_vision.memoria_aruco = True
        self.detector = crear_detector()

        # Control PD al centro
        self.control_activo = not args.sin_control
        self.mapa = [(1, 1), (0, 1)]  # GPIO41 -> +Y, GPIO42 -> +X
        self.controlador = ControlPD(
            self.mapa,
            kp=args.kp,
            kd=args.kd,
            ki=args.ki,
            max_deg=args.max_deg,
            centros=(args.c41, args.c42),
        )
        self.ultimos_angulos = [args.c41, args.c42]
        self.fallos_consecutivos = 0

        # Conexión USB al ESP32-S3
        self.get_logger().info(f"Conectando a cámara en {args.puerto} a {args.baudrate} baud...")
        self.src = FuenteUSB(args.puerto, args.baudrate)
        firmware = self.src.leer_estado(3)
        self.get_logger().info(f"Firmware cámara: {firmware.get('version')} | Baud: {firmware.get('baudrate')}")

        # Configurar servos en el firmware
        if firmware.get("version") == "camara-diag-0.5":
            self.src.port.write(b"hb\n")
            self.src.port.write(f"servo_cfg {args.max_deg:.1f} {args.c41:.1f} {args.c42:.1f}\n".encode())
            time.sleep(0.05)
            if self.control_activo:
                self.src.port.write(f"servo_arm {args.c41:.1f} {args.c42:.1f}\n".encode())
                time.sleep(0.05)
                self.get_logger().info("Servos armados para control al centro.")

        self.activo = True
        self.hilo = threading.Thread(target=self.bucle_principal, daemon=True)
        self.hilo.start()

    def recibir_comando(self, msg):
        cmd = msg.data.strip()
        self.get_logger().info(f"Comando recibido: {cmd}")
        if cmd in ("arm", "armar"):
            self.src.port.write(b"hb\n")
            self.src.port.write(f"servo_arm {self.args.c41:.1f} {self.args.c42:.1f}\n".encode())
            self.control_activo = True
        elif cmd in ("off", "parar", "stop", "!"):
            self.control_activo = False
            self.src.port.write(b"servo_off\n")
        elif cmd in ("centro", "centrar"):
            self.control_activo = False
            self.src.port.write(b"servo_centro\n")

    def bucle_principal(self):
        t_prev = time.monotonic()
        fps_suave = 0.0
        n_frame = 0

        while self.activo and rclpy.ok():
            t0 = time.monotonic()
            try:
                frame = self.src.read()
            except Exception as e:
                self.fallos_consecutivos += 1
                if self.control_activo and self.fallos_consecutivos > 15:
                    self.get_logger().warn(f"Pérdida prolongada de señal ({e}); apagando servos.")
                    self.src.port.write(b"servo_off\n")
                    self.control_activo = False
                time.sleep(0.02)
                continue

            dt_captura = time.monotonic() - t0
            foto, mask, r = procesar(frame, self.detector, self.cfg_vision, dibujar=True)
            ahora = time.monotonic()
            dt_ciclo = ahora - t_prev
            t_prev = ahora
            if dt_ciclo > 0:
                fps_suave = 0.9 * fps_suave + 0.1 * (1.0 / dt_ciclo)

            # Control PD
            valido = r.get("control_valido", False)
            bola = r.get("bola")

            if self.control_activo:
                self.src.port.write(b"hb\n")
                if valido and bola is not None:
                    self.fallos_consecutivos = 0
                    bx, by = bola["xy"]
                    # Acotar suavemente si toca el marco para no abortar
                    bx_safe = max(-8.5, min(8.5, bx))
                    by_safe = max(-6.1, min(6.1, by))
                    try:
                        angulos = self.controlador.paso([bx_safe, by_safe], ahora)
                        self.ultimos_angulos = angulos
                        self.src.port.write(f"servo_set {angulos[0]:.2f} {angulos[1]:.2f}\n".encode())
                    except ValueError as e:
                        self.get_logger().warn(f"Aviso control: {e}")
                else:
                    self.fallos_consecutivos += 1
                    if self.fallos_consecutivos <= 15:
                        # Mantener última consigna durante oclusiones breves (<0.8 s)
                        self.src.port.write(f"servo_set {self.ultimos_angulos[0]:.2f} {self.ultimos_angulos[1]:.2f}\n".encode())
                    else:
                        self.get_logger().warn("Pelota no detectada durante >0.8 s: pausando servos.")
                        self.src.port.write(b"servo_centro\n")
                        self.control_activo = False

            # Publicar a ROS 2
            stamp = self.get_clock().now().to_msg()
            self.publicar_ros(stamp, foto, r, self.ultimos_angulos, fps_suave)
            n_frame += 1

    def publicar_ros(self, stamp, foto, r, angulos, fps):
        # 1. Imagen anotada
        try:
            msg_img = self.bridge.cv2_to_imgmsg(foto, encoding="bgr8")
            msg_img.header.stamp = stamp
            msg_img.header.frame_id = "camara"
            self.pub_img.publish(msg_img)
        except Exception:
            pass

        # Ángulos respecto al centro horizontal (grados y radianes)
        pitch_deg = angulos[0] - self.args.c41  # Y
        roll_deg = angulos[1] - self.args.c42   # X
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)

        # 2. TF: mundo -> bandeja
        qx, qy, qz, qw = quat_desde_rpy(roll_rad, pitch_rad, 0.0)
        tf_b = TransformStamped()
        tf_b.header.stamp = stamp
        tf_b.header.frame_id = "mundo"
        tf_b.child_frame_id = "bandeja"
        tf_b.transform.translation.x = 0.0
        tf_b.transform.translation.y = 0.0
        tf_b.transform.translation.z = 0.05
        tf_b.transform.rotation.x = qx
        tf_b.transform.rotation.y = qy
        tf_b.transform.rotation.z = qz
        tf_b.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_b)

        # 3. Posición de la pelota
        bola = r.get("bola")
        bx_m, by_m = 0.0, 0.0
        if bola is not None:
            bx_cm, by_cm = bola["xy"]
            bx_m = bx_cm / 100.0
            by_m = by_cm / 100.0

            msg_p = PointStamped()
            msg_p.header.stamp = stamp
            msg_p.header.frame_id = "bandeja"
            msg_p.point.x = bx_m
            msg_p.point.y = by_m
            msg_p.point.z = 0.01  # Radio de la esfera (~1 cm)
            self.pub_pelota.publish(msg_p)

            # TF: bandeja -> pelota
            tf_p = TransformStamped()
            tf_p.header.stamp = stamp
            tf_p.header.frame_id = "bandeja"
            tf_p.child_frame_id = "pelota"
            tf_p.transform.translation.x = bx_m
            tf_p.transform.translation.y = by_m
            tf_p.transform.translation.z = 0.01
            tf_p.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf_p)

        # 4. Inclinación y servos
        msg_act = Vector3Stamped()
        msg_act.header.stamp = stamp
        msg_act.header.frame_id = "bandeja"
        msg_act.vector.x = roll_deg
        msg_act.vector.y = pitch_deg
        msg_act.vector.z = 0.0
        self.pub_actitud.publish(msg_act)

        msg_srv = Float32MultiArray()
        msg_srv.data = [float(angulos[0]), float(angulos[1])]
        self.pub_servos.publish(msg_srv)

        # 5. Marcadores RViz 3D
        self.publicar_marcadores(stamp, bx_m, by_m, bola is not None, roll_deg, pitch_deg, fps, r.get("estado", ""))

    def publicar_marcadores(self, stamp, bx, by, hay_bola, roll, pitch, fps, estado):
        ma = MarkerArray()

        # 0. Bandeja rectangular (18.7 x 14 cm, grosor 6 mm)
        m_bandeja = Marker()
        m_bandeja.header.stamp = stamp
        m_bandeja.header.frame_id = "bandeja"
        m_bandeja.ns = "estructura"
        m_bandeja.id = 0
        m_bandeja.type = Marker.CUBE
        m_bandeja.action = Marker.ADD
        m_bandeja.pose.position.z = -0.003
        m_bandeja.pose.orientation.w = 1.0
        m_bandeja.scale.x = 0.187
        m_bandeja.scale.y = 0.140
        m_bandeja.scale.z = 0.006
        m_bandeja.color = ColorRGBA(r=0.15, g=0.16, b=0.20, a=0.95)
        ma.markers.append(m_bandeja)

        # 1. Marco perimetral rojo (identificativo del hardware DSM)
        m_marco = Marker()
        m_marco.header.stamp = stamp
        m_marco.header.frame_id = "bandeja"
        m_marco.ns = "estructura"
        m_marco.id = 1
        m_marco.type = Marker.LINE_STRIP
        m_marco.action = Marker.ADD
        m_marco.pose.orientation.w = 1.0
        m_marco.scale.x = 0.004  # 4 mm grosor de línea
        m_marco.color = ColorRGBA(r=0.9, g=0.15, b=0.1, a=1.0)
        hx, hy = 0.187 / 2, 0.140 / 2
        for px, py in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy), (-hx, -hy)):
            pt = Point(); pt.x = px; pt.y = py; pt.z = 0.001
            m_marco.points.append(pt)
        ma.markers.append(m_marco)

        # 2. Esfera metálica rodando en la bandeja
        m_bola = Marker()
        m_bola.header.stamp = stamp
        m_bola.header.frame_id = "bandeja"
        m_bola.ns = "pelota"
        m_bola.id = 2
        m_bola.type = Marker.SPHERE
        m_bola.action = Marker.ADD if hay_bola else Marker.DELETE
        m_bola.pose.position.x = bx
        m_bola.pose.position.y = by
        m_bola.pose.position.z = 0.010  # radio 10 mm
        m_bola.pose.orientation.w = 1.0
        m_bola.scale.x = 0.020
        m_bola.scale.y = 0.020
        m_bola.scale.z = 0.020
        m_bola.color = ColorRGBA(r=0.2, g=0.85, b=1.0, a=0.95)  # Cian metálico brillante
        ma.markers.append(m_bola)

        # 3. Diana de objetivo al centro (0, 0)
        m_centro = Marker()
        m_centro.header.stamp = stamp
        m_centro.header.frame_id = "bandeja"
        m_centro.ns = "referencia"
        m_centro.id = 3
        m_centro.type = Marker.LINE_LIST
        m_centro.action = Marker.ADD
        m_centro.pose.orientation.w = 1.0
        m_centro.scale.x = 0.002
        m_centro.color = ColorRGBA(r=0.2, g=0.9, b=0.3, a=0.8)
        c_pts = [(-0.015, 0.0), (0.015, 0.0), (0.0, -0.015), (0.0, 0.015)]
        for px, py in c_pts:
            pt = Point(); pt.x = px; pt.y = py; pt.z = 0.001
            m_centro.points.append(pt)
        ma.markers.append(m_centro)

        # 4. Texto informativo sobre la bandeja
        m_txt = Marker()
        m_txt.header.stamp = stamp
        m_txt.header.frame_id = "bandeja"
        m_txt.ns = "texto"
        m_txt.id = 4
        m_txt.type = Marker.TEXT_VIEW_FACING
        m_txt.action = Marker.ADD
        m_txt.pose.position.x = 0.0
        m_txt.pose.position.y = 0.095
        m_txt.pose.position.z = 0.04
        m_txt.scale.z = 0.018
        m_txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        ctrl_txt = "ACTIVO" if self.control_activo else "PAUSA/PARADO"
        m_txt.text = f"FPS: {fps:.1f} | Control: {ctrl_txt}\nPelota: X={bx*100:+.1f} cm, Y={by*100:+.1f} cm\nServos: X={roll:+.1f}°, Y={pitch:+.1f}°"
        ma.markers.append(m_txt)

        self.pub_marcadores.publish(ma)

    def cerrar(self):
        self.activo = False
        try:
            self.src.port.write(b"servo_centro\n")
            time.sleep(0.08)
            self.src.port.write(b"servo_off\n")
            time.sleep(0.05)
            self.src.close()
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser(description="Puente Ball & Plate ROS 2 + RViz")
    p.add_argument("--puerto", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    p.add_argument("--baudrate", type=int, default=2000000)
    p.add_argument("--kp", type=float, default=0.55)
    p.add_argument("--kd", type=float, default=0.80)
    p.add_argument("--ki", type=float, default=0.50)
    p.add_argument("--max-deg", type=float, default=10.0)
    p.add_argument("--c41", type=float, default=90.0)
    p.add_argument("--c42", type=float, default=90.0)
    p.add_argument("--sin-control", action="store_true", help="Solo publicar telemetría sin mover servos")
    p.add_argument("--rviz", action="store_true", help="Abrir RViz2 automáticamente")
    args = p.parse_args()

    rclpy.init()
    nodo = PuenteBallPlate(args)

    rviz_proc = None
    if args.rviz:
        rviz_cfg = RAIZ / "herramientas" / "ball_plate.rviz"
        print(f"==> Iniciando RViz2 con {rviz_cfg}...")
        rviz_proc = subprocess.Popen(["rviz2", "-d", str(rviz_cfg)])

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        print("\nDeteniendo nodo...")
    finally:
        nodo.cerrar()
        nodo.destroy_node()
        rclpy.shutdown()
        if rviz_proc:
            rviz_proc.terminate()


if __name__ == "__main__":
    main()
