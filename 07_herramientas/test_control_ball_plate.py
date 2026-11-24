import unittest
from control_ball_plate import ControlPD


class ControlTest(unittest.TestCase):
    def test_signos_y_ejes_intercambiados(self):
        self.assertEqual(ControlPD([(0,1),(1,1)]).paso([2,-2],1),[89.3,90.7])
        self.assertEqual(ControlPD([(1,-1),(0,1)]).paso([2,-2],1),[89.3,89.3])
        with self.assertRaises(ValueError):ControlPD([(0,1),(0,-1)])

    def test_limites_datos_y_tiempo(self):
        c=ControlPD([(0,1),(1,1)])
        self.assertEqual(c.paso([8,-6],1),[88,92])
        with self.assertRaises(ValueError):c.paso([8,-6],1.4)
        for pos in ([9,0],[0,6.3],[float('nan'),0]):
            with self.assertRaises(ValueError):ControlPD([(0,1),(1,1)]).paso(pos,1)

    def test_convergencia_modelo_ideal_con_retardo(self):
        # Prueba del algoritmo, no certifica las ganancias ni la estabilidad físicas.
        for mapa in ([(0,1),(1,1)],[(1,-1),(0,1)]):
            c=ControlPD(mapa);p=[2.,-1.5];v=[0.,0.];cola=[[90.,90.]]*2;dt=1/12
            for n in range(240):
                u=c.paso(p,1+n*dt);cola.append(u);retardo=cola.pop(0)
                acc=[0.,0.]
                for j,(eje,signo) in enumerate(mapa):acc[eje]=signo*8*(retardo[j]-90)
                for k in range(2):v[k]+=acc[k]*dt;p[k]+=v[k]*dt
            self.assertLess(max(abs(x) for x in p),.15)


if __name__=='__main__':unittest.main()
