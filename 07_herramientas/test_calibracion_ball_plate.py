import tempfile
import unittest
from pathlib import Path
from calibracion_ball_plate import validar, guardar, cargar, crear_args
from vision_ball_plate import argumentos


class CalibracionTest(unittest.TestCase):
    def test_guardar_cargar_y_usar_en_visor(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'perfil.json'
            config=guardar({'ancho_cm':13.6,'alto_cm':9.2,'contraste':30},p)
            self.assertEqual(cargar(p),config)
            args=argumentos(['--calibracion',str(p)])
            self.assertEqual((args.ancho_cm,args.alto_cm,args.contraste),(13.6,9.2,30))
            self.assertTrue(args.auto_contraste)

    def test_no_usa_dimensiones_de_bordes_sin_confirmacion(self):
        args=crear_args({})
        self.assertIsNone(args.ancho_cm)
        self.assertIsNone(args.alto_cm)

    def test_rechaza_perfiles_incompletos_o_invalidos(self):
        for c in ({'ancho_cm':13.6},{'contraste':0},{'ventana_local':40},
                  {'area_min':float('nan')},{'alto_cm':float('inf'),'ancho_cm':10},
                  {'auto_contraste':'false'},{'clave_desconocida':1}):
            with self.subTest(c=c), self.assertRaises(ValueError):validar(c)
