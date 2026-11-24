"""Arranca la fisica y comprueba que su reloj llega a ROS 2 mediante ros_gz."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "88"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["GZ_PARTITION"] = f"dsm_instalacion_{os.getpid()}"

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

processes = []
rclpy.init()
node = rclpy.create_node("verificar_instalacion_gazebo")
clocks = []
subscription = node.create_subscription(
    Clock, "/clock", lambda msg: clocks.append(msg.clock.sec + msg.clock.nanosec * 1e-9),
    qos_profile_sensor_data,
)
try:
    with tempfile.TemporaryFile(mode="w+") as logfile:
        world = Path(__file__).with_name("prueba.sdf")
        for command in (
            ["gz", "sim", "-s", "-r", str(world)],
            ["ros2", "run", "ros_gz_bridge", "parameter_bridge", "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        ):
            processes.append(subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if len(clocks) >= 2 and clocks[-1] > clocks[0]:
                print(f"Correcto: Gazebo simula y ROS 2 recibe su reloj ({clocks[-1]:.3f} s).")
                break
        else:
            logfile.seek(0)
            print(logfile.read())
            raise SystemExit("No se recibio un reloj de simulacion que avance.")
finally:
    node.destroy_node()
    rclpy.shutdown()
    for proc in processes:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    for proc in processes:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
