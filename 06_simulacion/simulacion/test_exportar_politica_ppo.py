"""Comparación numérica de inferencia RSL-RL/PyTorch contra C++ compilado en CPU."""
import ctypes
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
import numpy as np
import torch
from tensordict import TensorDict
from rsl_rl.models import MLPModel
from rsl_rl.modules import EmpiricalNormalization
from exportar_politica_ppo_cpp import ROOT, exportar, fmt_arr

CKPT=ROOT/'herramientas/IsaacLab/logs/rsl_rl/aerotray_cad/2026-10-31_10-09-28_curriculo_etapa_4/model_24700.pt'


class ExportarTest(unittest.TestCase):
    def test_cpp_coincide_con_modelo_original(self):
        sd=torch.load(CKPT,map_location='cpu',weights_only=False)['actor_state_dict']
        model=MLPModel(TensorDict({'policy':torch.zeros(1,17)},batch_size=[1]),
                       {'actor':['policy']},'actor',8,hidden_dims=[64,64],
                       activation='elu',obs_normalization=True)
        model.load_state_dict({k:v for k,v in sd.items() if not k.startswith('distribution.')})
        model.eval()
        torch.manual_seed(417)
        mean=sd['obs_normalizer._mean'];std=sd['obs_normalizer._std']
        obs=mean+(std+EmpiricalNormalization(17).eps)*torch.randn(256,17)*2
        with torch.no_grad():expected=model(TensorDict({'policy':obs},batch_size=[256])).clamp(-1,1).numpy()
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);header=d/'policy.h';exportar(CKPT,header)
            (d/'bridge.cpp').write_text('#include "policy.h"\nextern "C" void infer(const float *x, float *y) { dsm_ppo::evaluar_actor(x,y); }\n')
            subprocess.run(['g++','-std=c++17','-shared','-fPIC','-O2',str(d/'bridge.cpp'),'-o',str(d/'bridge.so')],check=True)
            lib=ctypes.CDLL(str(d/'bridge.so'))
            ptr=ctypes.POINTER(ctypes.c_float);lib.infer.argtypes=[ptr,ptr]
            actual=np.empty_like(expected);inputs=obs.numpy().astype(np.float32)
            for x,y in zip(inputs,actual):lib.infer(x.ctypes.data_as(ptr),y.ctypes.data_as(ptr))
            error=float(np.abs(actual-expected).max())
            print(f'Error máximo RSL-RL frente a C++ en 256 observaciones: {error:.9g}')
            np.testing.assert_allclose(actual,expected,atol=2e-5,rtol=1e-5)

    def test_rechaza_evaluacion_de_otro_checkpoint(self):
        with tempfile.TemporaryDirectory() as d:
            d=Path(d);p=d/'eval.json'
            p.write_text(json.dumps({'modo':'policy','checkpoint':'otro.pt'}))
            with self.assertRaises(ValueError):exportar(CKPT,d/'policy.h',p)
            self.assertFalse((d/'policy.h').exists())

    def test_constantes_enteras_son_literales_cpp_validos(self):
        self.assertEqual(fmt_arr(torch.tensor([0.,1.,-2.])), '0.0f, 1.0f, -2.0f')


if __name__=='__main__':unittest.main()
