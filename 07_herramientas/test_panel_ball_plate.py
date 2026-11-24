"""Comprueba el receptor y la aplicación de calibración sin abrir Tk ni USB."""
import multiprocessing as mp
from pathlib import Path
import tempfile
import time
import unittest
from calibracion_ball_plate import DEFECTO, guardar
from panel_ball_plate import trabajo_panel


class PanelWorkerTest(unittest.TestCase):
    def test_demo_aplica_calibracion_y_no_accede_a_usb(self):
        ctx=mp.get_context('spawn')
        ordenes=ctx.Queue();salida=ctx.Queue(1);parar=ctx.Event();apagar=ctx.Event()
        gen=ctx.Value('i',0);viva=ctx.Value('d',time.monotonic())
        config={**DEFECTO,'ancho_cm':13.6,'alto_cm':9.2}
        im=Path(__file__).parent/'pruebas_datos/bola_metalica.jpg'
        with tempfile.TemporaryDirectory() as d:
            d=Path(d)
            perfil=d/'perfil.json';guardar(config,perfil)
            p=ctx.Process(target=trabajo_panel,args=('ninguno',im,ordenes,salida,parar,apagar,gen,viva,d,d/'log.jsonl',perfil))
            p.start()
            try:
                datos=salida.get(timeout=5)
                self.assertTrue(datos['demo'])
                self.assertFalse(datos.get('lista',False))
                self.assertIsNotNone(datos['r']['bola'])
                self.assertEqual(datos['mascara'].ndim,2)
                import json
                ordenes.put((0,'vision_cfg '+json.dumps({**config,'contraste':30})))
                for _ in range(10):
                    datos=salida.get(timeout=2)
                    if datos['calibracion']['contraste']==30:break
                self.assertEqual(datos['calibracion']['contraste'],30)
                self.assertEqual(datos['r']['centros_cm'],[13.6,9.2])
            finally:
                parar.set();p.join(timeout=3)
                if p.is_alive():p.terminate();p.join()
            self.assertEqual(p.exitcode,0)

    def test_sin_video_publica_posicion_y_permite_mostrar_y_ocultar(self):
        ctx=mp.get_context('spawn')
        ordenes=ctx.Queue();salida=ctx.Queue(1);parar=ctx.Event();apagar=ctx.Event()
        gen=ctx.Value('i',0);viva=ctx.Value('d',time.monotonic())
        video=ctx.Value('b',False);fotos=ctx.Value('b',False)
        im=Path(__file__).parent/'pruebas_datos/bola_metalica.jpg'
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);perfil=d/'perfil.json'
            guardar({**DEFECTO,'ancho_cm':13.6,'alto_cm':9.2},perfil)
            p=ctx.Process(target=trabajo_panel,args=('ninguno',im,ordenes,salida,parar,apagar,gen,viva,d,d/'log.jsonl',perfil,video,fotos))
            p.start()
            try:
                datos=salida.get(timeout=5)
                self.assertIsNone(datos['foto']);self.assertIsNone(datos['mascara'])
                self.assertIsNotNone(datos['r']['bola'])
                video.value=True
                for _ in range(10):
                    datos=salida.get(timeout=2)
                    if datos['foto'] is not None:break
                self.assertIsNotNone(datos['foto'])
                video.value=False
                for _ in range(10):
                    datos=salida.get(timeout=2)
                    if datos['foto'] is None:break
                self.assertIsNone(datos['foto'])
                self.assertTrue(datos['r']['control_valido'])
            finally:
                parar.set();p.join(timeout=3)
                if p.is_alive():p.terminate();p.join()
            self.assertEqual(p.exitcode,0)
