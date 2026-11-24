"""Pruebas de medicion planar y rechazo de referencias ambiguas, sin hardware."""
import unittest
from unittest.mock import patch
from pathlib import Path
import cv2
import numpy as np

from vision_ball_plate import argumentos, crear_detector, centro_proyectivo, geometria, procesar, unificar_contornos, detectar_bola_adaptativa


def tablero(con_bola=True, doble_bola=False):
    img = np.full((480, 640, 3), 45, np.uint8)
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for i, (x, y) in enumerate([(65, 65), (575, 65), (575, 415), (65, 415)]):
        cv2.rectangle(img, (x-37, y-37), (x+37, y+37), (255, 255, 255), -1)
        marker = cv2.aruco.generateImageMarker(dic, i, 60) if hasattr(cv2.aruco, "generateImageMarker") else cv2.aruco.drawMarker(dic, i, 60)
        img[y-30:y+30, x-30:x+30] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    if con_bola:
        cv2.circle(img, (380, 210), 13, (245, 245, 245), -1)
    if doble_bola:
        cv2.circle(img, (250, 270), 13, (245, 245, 245), -1)
    return img


class VisionTest(unittest.TestCase):
    def setUp(self):
        self.args = argumentos(["--ancho-cm", "20", "--alto-cm", "14", "--sin-ventana"])
        self.detector = crear_detector()

    def test_pelota_y_signos_bajo_perspectiva(self):
        im = tablero()
        src = np.float32([[0, 0], [639, 0], [639, 479], [0, 479]])
        dst = np.float32([[75, 25], [600, 60], [630, 440], [15, 465]])
        warp = cv2.getPerspectiveTransform(src, dst)
        im = cv2.warpPerspective(im, warp, (640, 480), borderValue=(45, 45, 45))
        _, _, r = procesar(im, self.detector, self.args)
        self.assertEqual(sorted(r["ids"]), [0, 1, 2, 3])
        self.assertTrue(r["geometria_valida"])
        self.assertIsNotNone(r["bola"])
        # (380,210) respecto a centros (64.5,64.5)..(574.5,414.5).
        np.testing.assert_allclose(r["bola"]["xy"], [(380-319.5)*20/510, (239.5-210)*14/350], atol=0.12)

    def test_papel_blanco_no_es_pelota(self):
        _, _, r = procesar(tablero(False), self.detector, self.args)
        self.assertTrue(r["geometria_valida"])
        self.assertIsNone(r["bola"])

    def test_dos_objetos_es_ambiguo(self):
        _, _, r = procesar(tablero(True, True), self.detector, self.args)
        self.assertIsNone(r["bola"])
        self.assertIn("Ambigua", r["estado"])

    def test_repetidos_y_faltantes_no_dan_medida(self):
        c = [np.float32([[[10, 10], [30, 10], [30, 30], [10, 30]]])]*4
        for ids in ([0, 1, 1, 3], [0, 1, 2]):
            H, pts, _ = geometria(c[:len(ids)], ids, 20, 14)
            self.assertIsNone(H)
            self.assertIsNone(pts)

    def test_centro_no_es_promedio_vertices(self):
        src = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
        dst = np.float32([[10, 10], [200, 30], [130, 150], [50, 90]])
        H = cv2.getPerspectiveTransform(src, dst)
        esperado = cv2.perspectiveTransform(np.float32([[[0.5, 0.5]]]), H)[0, 0]
        np.testing.assert_allclose(centro_proyectivo(dst), esperado, atol=1e-4)

    def test_sin_regla_no_inventa_centimetros(self):
        args = argumentos(["--sin-ventana"])
        _, _, r = procesar(tablero(), self.detector, args)
        self.assertEqual(r["unidad"], "normalizada")
        self.assertIsNotNone(r["bola"])

    def test_metal_con_distinta_iluminacion(self):
        path = Path(__file__).resolve().parent / "pruebas_datos" / "bola_metalica.jpg"
        original = cv2.imread(str(path))
        self.assertIsNotNone(original)
        args = argumentos(["--modo", "metal", "--area-min", "100", "--area-max", "1200",
                           "--ancho-cm", "18.7", "--alto-cm", "14", "--sin-ventana"])
        for ganancia in (0.65, 0.8, 1.0, 1.2):
            with self.subTest(ganancia=ganancia):
                img = np.clip(original.astype(np.float32)*ganancia, 0, 255).astype(np.uint8)
                _, _, r = procesar(img, self.detector, args)
                self.assertTrue(r["geometria_valida"])
                self.assertIsNotNone(r["bola"])
                # Comparar con el cuerpo de la esfera, no con un brillo aislado.
                np.testing.assert_allclose(r["bola"]["uv"], [179, 122], atol=3)

    def test_metal_sin_pelota_ni_falsos_en_codigos(self):
        args = argumentos(["--modo", "metal", "--sin-ventana"])
        _, _, r = procesar(tablero(False), self.detector, args)
        self.assertTrue(r["geometria_valida"])
        self.assertIsNone(r["bola"])

    def test_un_papel_con_dos_contornos_no_es_duplicado(self):
        grande = np.float32([[[10, 10], [30, 10], [30, 30], [10, 30]]])
        interior = np.float32([[[12, 12], [28, 12], [28, 28], [12, 28]]])
        c, ids = unificar_contornos([interior, grande], [1, 1])
        self.assertEqual(ids, [1])
        np.testing.assert_array_equal(c[0], grande)

    def test_dos_papeles_repetidos_siguen_siendo_ambiguos(self):
        a = np.float32([[[10, 10], [30, 10], [30, 30], [10, 30]]])
        b = a + np.float32([40, 0])
        _, ids = unificar_contornos([a, b], [1, 1])
        self.assertEqual(ids, [1, 1])

    def test_escala_confirmada_entre_centros(self):
        a=argumentos(['--ancho-cm','13.6','--alto-cm','9.2'])
        _,_,r=procesar(tablero(),self.detector,a)
        np.testing.assert_allclose(r['bola']['xy'],[(380-319.5)*13.6/510,(239.5-210)*9.2/350],atol=.1)
        self.assertTrue(r['control_valido'])

    def test_tres_esquinas_no_inventan_una_cuarta(self):
        self.args.memoria_aruco=True
        im=tablero();im[20:110,20:110]=45
        _,_,r=procesar(im,self.detector,self.args)
        self.assertEqual(sorted(r['ids']),[1,2,3])
        self.assertFalse(r['geometria_valida'])
        self.assertFalse(r['control_valido'])

    def test_memoria_tres_arucos_inmoviles_mantiene_control(self):
        # Un 4.º ArUco tapado por un reflejo no debe cortar el control si los
        # otros 3 siguen quietos (<=2 px): la bandeja no se ha movido.
        self.args.memoria_aruco=True
        with patch('vision_ball_plate.time.monotonic',return_value=100):
            procesar(tablero(),self.detector,self.args)
        im=tablero();im[20:110,20:110]=45
        with patch('vision_ball_plate.time.monotonic',return_value=100.1):
            _,_,r=procesar(im,self.detector,self.args)
        self.assertEqual(r['origen_geometria'],'memoria3')
        self.assertFalse(r['geometria_observada'])
        self.assertTrue(r['control_valido'])
        self.assertEqual(sorted(r['ids']),[1,2,3])
        # Sin ninguna observación completa durante más de 0.25 s entre fotos: caduca.
        with patch('vision_ball_plate.time.monotonic',return_value=100.5):
            _,_,r=procesar(im,self.detector,self.args)
        self.assertFalse(r['geometria_valida'])
        self.assertFalse(r['control_valido'])

    def test_memoria_con_dos_arucos_no_habilita_control(self):
        self.args.memoria_aruco=True
        with patch('vision_ball_plate.time.monotonic',return_value=100):
            procesar(tablero(),self.detector,self.args)
        im=tablero();im[20:110,20:110]=45;im[20:110,-110:-20]=45
        with patch('vision_ball_plate.time.monotonic',return_value=100.1):
            _,_,r=procesar(im,self.detector,self.args)
        self.assertFalse(r['control_valido'])

    def test_movimiento_no_reutiliza_referencia_vieja(self):
        self.args.memoria_aruco=True
        procesar(tablero(),self.detector,self.args)
        im=tablero();im[20:110,20:110]=45
        im=cv2.warpAffine(im,np.float32([[1,0,8],[0,1,0]]),(640,480))
        _,_,r=procesar(im,self.detector,self.args)
        self.assertFalse(r['geometria_valida'])

    def test_recuperacion_exige_dos_umbrales_consistentes(self):
        a=argumentos(['--modo','metal','--auto-contraste'])
        m=np.zeros((10,10),np.uint8)
        def falso(*_):
            a._diagnostico_bola={}
            if a.contraste==25:return None,m,'Sin pelota'
            return {'uv':[50+a.contraste/100,50],'area':500.,'xy':[0.,0.]},m,'Pelota detectada'
        with patch('vision_ball_plate.detectar_bola',side_effect=falso) as f:
            b,_,s=detectar_bola_adaptativa(None,None,None,None,a)
        self.assertIsNotNone(b)
        self.assertEqual(f.call_count,3)
        self.assertIn('35',s)
        self.assertEqual(a.contraste,25)

    def test_recuperacion_no_elige_entre_dos_objetos(self):
        a=argumentos(['--modo','metal','--auto-contraste'])
        _,_,r=procesar(tablero(True,True),self.detector,a)
        self.assertIsNone(r['bola'])
        self.assertIn('Ambigua',r['estado'])

    def test_metal_auto_sin_pelota_no_inventa_medida(self):
        a=argumentos(['--modo','metal','--auto-contraste'])
        _,_,r=procesar(tablero(False),self.detector,a)
        self.assertIsNone(r['bola'])

    def test_normalizado_no_habilita_control(self):
        _,_,r=procesar(tablero(),self.detector,argumentos([]))
        self.assertIsNotNone(r['bola'])
        self.assertFalse(r['control_valido'])

    def test_recupera_aruco_borroso_en_fotos_reales_nuevas(self):
        paths=sorted((Path(__file__).parent/'pruebas_datos').glob('aruco_borroso_*.png'))
        self.assertEqual(len(paths),3)
        a=argumentos(['--modo','metal','--auto-contraste','--area-min','100','--area-max','1200',
                      '--ancho-cm','13.6','--alto-cm','9.2'])
        for p in paths:
            with self.subTest(p=p):
                _,_,r=procesar(cv2.imread(str(p)),self.detector,a)
                self.assertEqual(sorted(r['ids']),[0,1,2,3])
                self.assertTrue(r['control_valido'])
                np.testing.assert_allclose(r['bola']['uv'],[134,117],atol=4)

    def test_sin_dibujar_conserva_la_medicion_para_control(self):
        foto,mask,r=procesar(tablero(),self.detector,self.args,dibujar=False)
        self.assertIsNone(foto)
        self.assertEqual(mask.shape,(480,640))
        self.assertTrue(r['control_valido'])
        _,_,visible=procesar(tablero(),self.detector,self.args)
        np.testing.assert_allclose(r['bola']['xy'],visible['bola']['xy'])

    def test_calibracion_de_tamano_rechaza_mancha_pequena(self):
        im=tablero()
        cv2.circle(im,(250,270),7,(245,245,245),-1)
        a=argumentos(['--modo','metal','--auto-contraste','--area-min','250','--area-max','1200'])
        _,_,r=procesar(im,self.detector,a)
        self.assertIsNotNone(r['bola'])
        np.testing.assert_allclose(r['bola']['uv'],[380,210],atol=1)
        _,_,doble=procesar(tablero(True,True),self.detector,a)
        self.assertIsNone(doble['bola'])

    def test_posicion_del_borde_no_parpadea_en_muestras_registradas(self):
        paths=sorted((Path(__file__).parent/'pruebas_datos').glob('borde_metal_*.png'))
        self.assertGreaterEqual(len(paths),2)
        a=argumentos(['--modo','metal','--auto-contraste','--area-min','250','--area-max','1200',
                      '--ancho-cm','13.6','--alto-cm','9.2'])
        for p in paths:
            with self.subTest(p=p):
                _,_,r=procesar(cv2.imread(str(p)),self.detector,a,dibujar=False)
                self.assertTrue(r['control_valido'])
                np.testing.assert_allclose(r['bola']['uv'],[124,186],atol=4)


if __name__ == "__main__":
    unittest.main()
