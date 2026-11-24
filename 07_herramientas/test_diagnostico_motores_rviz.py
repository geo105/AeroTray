import math
import time
import unittest

from diagnostico_motores_rviz import DiagnosticoMotores


def estado(**cambios):
    return dict(version='dsm-motores-0.2', ms=1000, estado=2, motor=0,
                pwm_us=1000, **cambios)


class DiagnosticoTest(unittest.TestCase):
    def test_identidad_y_caducidad_antes_de_ordenes(self):
        d = DiagnosticoMotores()
        with self.assertRaises(ValueError):
            d.preparar('armar')
        self.assertFalse(d.actualizar({'version': 'dsm-octo-0.4-banco'}))
        dato = estado(); dato['estado'] = 0
        self.assertTrue(d.actualizar(dato))
        self.assertEqual(d.preparar('armar'), ('armar', None))
        d.recibido = time.monotonic()-2
        with self.assertRaises(ValueError):
            d.preparar('armar')
        self.assertEqual(d.preparar('!'), ('!', None))

    def test_pwm_y_tiempo_seleccionables_y_limites(self):
        d = DiagnosticoMotores(); d.actualizar(estado())
        self.assertEqual(d.preparar('probar 1')[0], 'motor 1 1200 2000')
        d.preparar('pwm 1457'); d.preparar('tiempo 1.5')
        self.assertEqual(d.preparar('probar 8')[0], 'motor 8 1457 1500')
        self.assertEqual(d.preparar('probar 2 1950 .1')[0], 'motor 2 1950 100')
        for c in ('probar 9', 'probar 1 1951', 'probar 1 999', 'probar 1 1200 6',
                  'probar 1 1200 nan', 'tiempo nan', 'pwm 2000', 'iniciar 1300', 'motor 1 1200 2000'):
            with self.subTest(comando=c), self.assertRaises(ValueError):
                d.preparar(c)

    def test_firmware_antiguo_no_recibe_pwm_fuera_de_su_rango(self):
        d = DiagnosticoMotores(); r = estado(); r['version'] = 'dsm-motores-0.1'
        d.actualizar(r)
        with self.assertRaises(ValueError):
            d.preparar('probar 1 1400')

    def test_telemetria_no_inventa_altura_y_observacion_requiere_prueba(self):
        d = DiagnosticoMotores()
        with self.assertRaises(ValueError):
            d.preparar('observado 2 cw')
        r = estado(); r.update(estado=3, motor=2, pwm_us=1350)
        d.actualizar(r); t = d.telemetria()
        self.assertTrue(math.isnan(t['z_cm']))
        self.assertEqual([t[f'm{i}'] for i in range(1,9)], [1000,1350,1000,1000,1000,1000,1000,1000])
        with self.assertRaises(ValueError):
            d.preparar('probar 3')
        _, evento = d.preparar('observado 2 cw')
        self.assertFalse(evento['medido_por_sensor'])
        self.assertEqual(evento['prueba_confirmada']['pwm_us'], 1350)
        d.invalidar()
        with self.assertRaises(ValueError):
            d.preparar('probar 1')

    def test_datos_invalidos_no_identifican_firmware(self):
        for campo, valor in [('estado', 9), ('motor', 9), ('motor', 1.5), ('ms', float('nan')),
                             ('pwm_us', float('inf')), ('estado', True)]:
            d = DiagnosticoMotores(); r = estado(); r[campo] = valor
            self.assertFalse(d.actualizar(r))

    def test_todos_requiere_version_nueva_y_respeta_limites(self):
        d = DiagnosticoMotores(); r = estado(); d.actualizar(r)
        with self.assertRaises(ValueError):
            d.preparar('todos 1200 2')
        r['version'] = 'dsm-motores-0.3'; d.actualizar(r)
        self.assertEqual(d.preparar('todos')[0], 'todos 1200 2000')
        self.assertEqual(d.preparar('todos 1275 1.5')[0], 'todos 1275 1500')
        for c in ('todos 1951 2', 'todos 999 2', 'todos 1200 6', 'todos 1200 nan'):
            with self.subTest(comando=c), self.assertRaises(ValueError):
                d.preparar(c)
        r.update(estado=3, motor=0, todos=True, pwm_us=1200)
        self.assertTrue(d.actualizar(r))
        t = d.telemetria()
        self.assertEqual([t[f'm{i}'] for i in range(1,9)], [1200]*8)
        self.assertEqual(t['estado_texto'], 'PRUEBA TODOS')
        with self.assertRaises(ValueError):
            d.preparar('todos')
        with self.assertRaises(ValueError):
            d.preparar('observado 1 cw')
        for cambios in ({'version':'dsm-motores-0.2'}, {'motor':1}, {'todos':'true'}):
            dato=dict(r);dato.update(cambios)
            self.assertFalse(d.actualizar(dato))


if __name__ == '__main__':
    unittest.main()
