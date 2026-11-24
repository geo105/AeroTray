"""Integración ROS sin hardware: mensajes, menús, bloqueo por identidad y alturas."""
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rclpy
from puente_dron_ros2 import PuenteDron


class Captura:
    def __init__(self):
        self.mensajes = []

    def publish(self, mensaje):
        self.mensajes.append(mensaje)


class SerieFalsa:
    def __init__(self):
        self.escritos = []

    def write(self, dato):
        self.escritos.append(dato)

    def close(self):
        pass


class PuenteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def crear(self, diagnostico=True):
        args = SimpleNamespace(diagnostico_motores=diagnostico, simular=True, wifi=False,
                               sin_watchdog=False, ip_wifi='192.168.4.1', puerto_wifi=4210,
                               mostrar_yaw=False, invertir_yaw=False)
        n = PuenteDron(args)
        self.addCleanup(n.destroy_node)
        self.addCleanup(n.cerrar)
        for atributo in ('pub_imu', 'pub_mot', 'pub_act', 'pub_bat', 'pub_est', 'pub_mk','pub_z','pub_z_valida'):
            setattr(n, atributo, Captura())
        n.tf = SimpleNamespace(sendTransform=lambda t: setattr(n, 'transformacion', t))
        return n

    def test_diag_no_publica_sensores_inexistentes_y_menus(self):
        n = self.crear()
        self.assertEqual(len(n.menus), 8)
        n.procesar_linea(json.dumps(dict(version='dsm-motores-0.2', ms=5000, estado=3,
                                        motor=5, pwm_us=1450, restante_ms=1500)))
        self.assertEqual(n.pub_mot.mensajes[-1].data[4], 1450)
        self.assertEqual(n.pub_imu.mensajes, [])
        self.assertEqual(n.pub_bat.mensajes, [])
        self.assertEqual(n.pub_act.mensajes, [])
        self.assertEqual(n.transformacion.transform.translation.z, 0.)
        info = [m.text for m in n.pub_mk.mensajes[-1].markers if m.ns == 'info'][0]
        self.assertIn('SIN_HELICES', info)
        self.assertIn('1.5_s_restantes', info)
        n.t_ultimo_tlm -= 3
        n.estado_periodico()
        info = [m.text for m in n.pub_mk.mensajes[-1].markers if m.ns == 'info'][0]
        self.assertIn('SIN_TELEMETRIA_RECIENTE', info)

    def test_no_arma_firmware_equivocado_y_escribe_un_solo_motor(self):
        n = self.crear(); n.args.simular = False; n.ser = SerieFalsa()
        n.procesar_linea('{"version":"dsm-octo-0.4-banco"}')
        n.enviar('armar')
        self.assertEqual(n.ser.escritos, [])
        n.procesar_linea(json.dumps(dict(version='dsm-motores-0.2', ms=5000, estado=2,
                                        motor=0, pwm_us=1000)))
        n.enviar('probar 5 1375 1.5')
        self.assertEqual(n.ser.escritos, [b'motor 5 1375 1500\n'])
        n.enviar('iniciar 1300')
        self.assertEqual(len(n.ser.escritos), 1)
        n.diag.invalidar(); n.enviar('probar 1')
        self.assertEqual(len(n.ser.escritos), 1)
        n.enviar('!')
        self.assertEqual(n.ser.escritos[-1], b'!')

    def test_altura_real_sin_offset_y_referencia_separada(self):
        n = self.crear(False)
        n.paso_simulado()
        dato = dict(n.ultimo); dato['z_cm'] = 30.
        n.publicar(dato)
        self.assertAlmostEqual(n.transformacion.transform.translation.z, .3)
        referencia = [m for m in n.pub_mk.mensajes[-1].markers if m.ns == 'objetivo_altura'][0]
        self.assertTrue(all(p.z == .3 for p in referencia.points))
        antes = n.n_tlm
        n.procesar_linea('0,nan,' + ','.join(['0']*19))
        self.assertEqual(n.n_tlm, antes)

    def test_todos_publica_ocho_pwm_y_rotulo_correcto(self):
        n = self.crear()
        n.procesar_linea(json.dumps(dict(version='dsm-motores-0.3', ms=5000, estado=3,
                                        motor=0, todos=True, pwm_us=1200, restante_ms=1500)))
        self.assertEqual(list(n.pub_mot.mensajes[-1].data), [1200]*8)
        info = [m.text for m in n.pub_mk.mensajes[-1].markers if m.ns == 'info'][0]
        self.assertIn('PRUEBA_TODOS|TODOS', info)

    def test_imu_real_mueve_dron_en_diagnostico_y_rechaza_datos_caducados(self):
        n = self.crear()
        dato = dict(version='dsm-motores-0.3',ms=6000,estado=0,motor=0,todos=False,pwm_us=0,
                    imu_ok=True,pitch=12.,roll=-7.,yaw=0.,gx=1.,gy=2.,gz=3.,
                    vbat_a=11.4,vbat_b=11.5,z_valida=True,z_cm=4.)
        n.procesar_linea(json.dumps(dato))
        self.assertEqual(len(n.pub_imu.mensajes),1)
        self.assertEqual(n.pub_act.mensajes[-1].vector.y,12.)
        self.assertNotEqual(n.transformacion.transform.rotation.y,0.)
        self.assertAlmostEqual(n.transformacion.transform.translation.z,.04)
        self.assertTrue(n.pub_z_valida.mensajes[-1].data)
        self.assertAlmostEqual(n.pub_z.mensajes[-1].data,4.)
        self.assertAlmostEqual(n.pub_bat.mensajes[-1].data[0],11.4,places=5)
        self.assertAlmostEqual(n.pub_bat.mensajes[-1].data[1],11.5,places=5)
        anterior = n.transformacion.transform.rotation
        dato.update(ms=6200,imu_ok=False,z_valida=False)
        n.procesar_linea(json.dumps(dato))
        self.assertEqual(len(n.pub_imu.mensajes),1)
        self.assertEqual(n.transformacion.transform.rotation,anterior)
        self.assertFalse(n.pub_z_valida.mensajes[-1].data)
        self.assertEqual(len(n.pub_z.mensajes),1)

    def test_vista_yaw_es_local_y_refleja_giro_real(self):
        n=self.crear();n.args.simular=False;n.ser=SerieFalsa()
        dato=dict(version='dsm-motores-0.3',ms=9000,estado=0,motor=0,pwm_us=0,
                  imu_ok=True,pitch=0.,roll=0.,yaw=90.,gx=0.,gy=0.,gz=0.)
        n.procesar_linea(json.dumps(dato))
        self.assertAlmostEqual(n.transformacion.transform.rotation.z,0.)
        n.enviar('vista_yaw 1');n.procesar_linea(json.dumps(dato))
        self.assertAlmostEqual(n.transformacion.transform.rotation.z,2**-.5)
        n.enviar('vista_yaw 0');n.procesar_linea(json.dumps(dato))
        self.assertAlmostEqual(n.transformacion.transform.rotation.z,0.)
        self.assertEqual(n.ser.escritos,[])

    def test_observador_isaac_no_envia_comandos_al_esp32(self):
        n = self.crear(); n.args.simular=False; n.ser=SerieFalsa()
        n.procesar_linea(json.dumps(dict(version='dsm-motores-0.3',ms=7000,estado=0,motor=0,todos=False,pwm_us=0)))
        with patch('puente_dron_ros2.subprocess.Popen') as proceso:
            proceso.return_value.poll.return_value=0
            n.enviar('isaac_sombra 15')
            proceso.assert_called_once()
            self.assertIn('ver_isaac_sin_pwm.py',proceso.call_args.args[0][1])
        self.assertEqual(n.ser.escritos,[])

    def test_montaje_imu8_laser1_no_cambia_la_actitud_medida(self):
        n = self.crear()
        dato = dict(version='dsm-motores-0.3',ms=7000,estado=0,motor=0,pwm_us=0,
                    imu_ok=True,pitch=12.,roll=-7.,yaw=0.,gx=1.,gy=2.,gz=3.)
        n.procesar_linea(json.dumps(dato))
        q = n.transformacion.transform.rotation
        n.procesar_linea(json.dumps(dict(version='dsm-montaje-0.1',imu_brazo=8,laser_brazo=1)))
        self.assertTrue(n.diag.vigente())
        n.procesar_linea(json.dumps(dato))
        self.assertEqual(n.transformacion.transform.rotation, q)
        sensores = {m.id: m for m in n.pub_mk.mensajes[-1].markers if m.ns=='sensores_montaje'}
        self.assertEqual(len(sensores),2)
        self.assertGreater(sensores[0].pose.position.x,0)
        self.assertGreater(sensores[0].pose.position.y,0)
        self.assertGreater(sensores[1].pose.position.x,0)
        self.assertAlmostEqual(sensores[1].pose.position.y,0)


if __name__ == '__main__':
    unittest.main()
