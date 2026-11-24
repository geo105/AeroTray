"""Verifica comunicacion real entre los demos C++ y Python de ROS 2."""
import os
import signal
import subprocess
import tempfile

env = dict(os.environ, ROS_DOMAIN_ID="87", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
processes = []
try:
    with tempfile.TemporaryFile(mode="w+") as talker_log, tempfile.TemporaryFile(mode="w+") as listener_log:
        talker = subprocess.Popen(
            ["ros2", "run", "demo_nodes_cpp", "talker"],
            env=env, stdout=talker_log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        processes.append(talker)
        listener = subprocess.Popen(
            ["ros2", "run", "demo_nodes_py", "listener"],
            env=env, stdout=listener_log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        processes.append(listener)
        try:
            listener.wait(timeout=18)
        except subprocess.TimeoutExpired:
            pass
        for proc in processes:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        talker_log.seek(0)
        listener_log.seek(0)
        sent = talker_log.read()
        received = listener_log.read()
        if "Publishing:" not in sent or "I heard:" not in received:
            print("Salida del publicador:\n", sent)
            print("Salida del receptor:\n", received)
            raise SystemExit("No se pudo verificar la comunicacion ROS 2.")
        print("Correcto: el nodo C++ publica y el nodo Python recibe mensajes por DDS.")
finally:
    for proc in processes:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()
