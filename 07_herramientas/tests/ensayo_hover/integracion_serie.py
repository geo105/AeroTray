#!/usr/bin/env python3
"""ESP32 y teclado simulados con PTY; prueba la consola completa sin hardware."""
import json
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"tests/ensayo_hover"))
from test_protocolo import packet
sys.path.insert(0,str(ROOT/"herramientas"))
from ensayo_hover.analizar import analyze


def main():
    serial_master,serial_slave=pty.openpty()
    key_master,key_slave=pty.openpty()
    port=os.ttyname(serial_slave)
    with tempfile.TemporaryDirectory() as tmp:
        directory=Path(tmp)/"sesion"
        with open(Path(tmp)/"consola.txt","w+") as out:
            process=subprocess.Popen([sys.executable,str(ROOT/"herramientas/ensayo_hover/consola.py"),
                                      "--puerto",port,"--registro",str(directory)],
                                     stdin=key_slave,stdout=out,stderr=out)
            commands=b""
            seq=0
            try:
                # Esperar apertura y creacion del registro antes de enviar binario.
                deadline=time.monotonic()+3
                while not (directory/"sesion.json").exists() and time.monotonic()<deadline:
                    if process.poll() is not None:
                        raise AssertionError("consola termino al inicializar")
                    time.sleep(.005)
                assert (directory/"sesion.json").exists()

                def cycle(seconds,stream=True):
                    nonlocal seq,commands
                    until=time.monotonic()+seconds
                    while time.monotonic()<until:
                        if stream:
                            os.write(serial_master,packet(seq,version=2)); seq+=1
                        if select.select([serial_master],[],[],.005)[0]:
                            commands+=os.read(serial_master,8192)

                cycle(.4)
                assert b"hb\n" in commands
                assert b"armar" not in commands and b"arrancar" not in commands and b"cal\n" not in commands
                commands=b""
                os.write(key_master,b"!") # Sin Enter
                cycle(.1)
                assert b"!\n" in commands,commands
                # Detener telemetria: la consola no mantiene heartbeat ciegamente.
                commands=b""
                cycle(.65,False)
                assert b"!\n" in commands
                commands=b""
                cycle(.2,False)
                assert b"hb\n" not in commands
                cycle(.15)
                os.write(key_master,b"salir\n")
                cycle(.4)
                process.wait(timeout=3)
                assert process.returncode==0
                meta=json.loads((directory/"sesion.json").read_text())
                assert meta["muestras"]>50 and meta["huecos_secuencia"]==0
                assert meta["protocolo_observado"]==2 and meta["revision_accel_observada"]==2
                assert meta["ultimo_modo"]=="OFF" and meta["ultimo_pwm_us"]==[1000]*8
                report=analyze(directory)
                assert not report["hover_confirmado"] and report["muestras_activas"]==0
                print("Integracion PTY: telemetria, registro, ! sin Enter, timeout, cierre OFF y analisis OK")
            except BaseException:
                out.flush();out.seek(0);print(out.read()[-5000:])
                raise
            finally:
                if process.poll() is None:
                    process.terminate();process.wait(timeout=3)
    for fd in (serial_master,serial_slave,key_master,key_slave):
        os.close(fd)


if __name__=="__main__":
    main()
