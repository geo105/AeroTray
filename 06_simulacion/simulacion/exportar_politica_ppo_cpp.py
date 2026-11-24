#!/usr/bin/env python3
"""Exporta el Actor PPO (normalizador + MLP 17->64->64->8 ELU) a un header C++ para ESP32."""
from pathlib import Path
import argparse
import hashlib
import json
import math
import torch
from rsl_rl.modules import EmpiricalNormalization

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "firmware/dron_octo_esp32/politica_ppo_sim.h"


def fmt_arr(t):
    vals = t.detach().cpu().numpy().ravel()
    def literal(v):
        s = f"{float(v):.9g}"
        return s + ('f' if '.' in s or 'e' in s else '.0f')
    return ", ".join(literal(v) for v in vals)


def exportar(CKPT, OUT, evaluacion=None):
    CKPT, OUT = Path(CKPT).resolve(), Path(OUT).resolve()
    reporte = None
    if evaluacion:
        reporte = json.loads(Path(evaluacion).read_text())
        if Path(reporte.get('checkpoint', '')).resolve() != CKPT or reporte.get('modo') != 'policy':
            raise ValueError('La evaluación no corresponde al checkpoint solicitado')
        claves = ('supervivencia_episodio', 'rms_altura_ultimos_2s_mm', 'rms_inclinacion_ultimos_2s_deg')
        if any(not isinstance(reporte.get(k), (int, float)) or not math.isfinite(reporte[k]) for k in claves):
            raise ValueError('La evaluación no contiene métricas finitas')
    data = torch.load(CKPT, map_location="cpu", weights_only=False)
    sd = data["actor_state_dict"]
    for key in ('obs_normalizer._mean', 'obs_normalizer._std'):
        if tuple(sd[key].shape) == (1, 17):
            sd[key] = sd[key].reshape(17)
    shapes = {'obs_normalizer._mean': (17,), 'obs_normalizer._std': (17,),
              'mlp.0.weight': (64,17), 'mlp.0.bias': (64,),
              'mlp.2.weight': (64,64), 'mlp.2.bias': (64,),
              'mlp.4.weight': (8,64), 'mlp.4.bias': (8,)}
    for key, shape in shapes.items():
        if tuple(sd[key].shape) != shape or not torch.isfinite(sd[key]).all():
            raise ValueError(f'Parámetro no compatible: {key}; se esperaba {shape}')
    if not (sd['obs_normalizer._std'] > 0).all():
        raise ValueError('Desviación de normalización no positiva')
    sha = hashlib.sha256(CKPT.read_bytes()).hexdigest()
    normalizer_eps = EmpiricalNormalization(17).eps
    mean = sd["obs_normalizer._mean"]
    std = sd["obs_normalizer._std"]
    w0, b0 = sd["mlp.0.weight"], sd["mlp.0.bias"]
    w2, b2 = sd["mlp.2.weight"], sd["mlp.2.bias"]
    w4, b4 = sd["mlp.4.weight"], sd["mlp.4.bias"]

    code = f"""#pragma once
// Generado automaticamente desde {CKPT.parent.name}/{CKPT.name}
// SHA256 checkpoint: {sha}
// Red Actor PPO: 17 -> 64 (ELU) -> 64 (ELU) -> 8 acciones en [-1, 1]
#include <math.h>

namespace dsm_ppo {{

static constexpr float OBS_MEAN[17] = {{{fmt_arr(mean)}}};
static constexpr float OBS_STD[17] = {{{fmt_arr(std)}}};
static constexpr float NORMALIZER_EPS = {normalizer_eps:.9g}f;

static constexpr float W0[64 * 17] = {{{fmt_arr(w0)}}};
static constexpr float B0[64] = {{{fmt_arr(b0)}}};

static constexpr float W2[64 * 64] = {{{fmt_arr(w2)}}};
static constexpr float B2[64] = {{{fmt_arr(b2)}}};

static constexpr float W4[8 * 64] = {{{fmt_arr(w4)}}};
static constexpr float B4[8] = {{{fmt_arr(b4)}}};
static constexpr float TRIM_N[8] = {{4.4128929f, 4.4130923f, 4.4159834f, 4.4100014f, 4.4151779f, 4.4150369f, 4.4108069f, 4.4109480f}};
static constexpr float ACTION_SCALE_N = 1.5f;

inline float elu(float x) {{
  return x >= 0.0f ? x : (expf(x) - 1.0f);
}}

inline void evaluar_actor(const float obs[17], float acciones_out[8]) {{
  float x[17], h1[64], h2[64];
  for (int i = 0; i < 17; ++i) {{
    x[i] = (obs[i] - OBS_MEAN[i]) / (OBS_STD[i] + NORMALIZER_EPS);
  }}
  for (int i = 0; i < 64; ++i) {{
    float s = B0[i];
    const float* w = &W0[i * 17];
    for (int j = 0; j < 17; ++j) s += w[j] * x[j];
    h1[i] = elu(s);
  }}
  for (int i = 0; i < 64; ++i) {{
    float s = B2[i];
    const float* w = &W2[i * 64];
    for (int j = 0; j < 64; ++j) s += w[j] * h1[j];
    h2[i] = elu(s);
  }}
  for (int i = 0; i < 8; ++i) {{
    float s = B4[i];
    const float* w = &W4[i * 64];
    for (int j = 0; j < 64; ++j) s += w[j] * h2[j];
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    acciones_out[i] = s;
  }}
}}

inline void paso_red(float zM, float vMS, const float rotRad[3], const float omegaRadS[3],
                     float refM, float accionesPrev[8], float fuerzasOut[8]) {{
  float normRot = sqrtf(rotRad[0]*rotRad[0] + rotRad[1]*rotRad[1] + rotRad[2]*rotRad[2]);
  float obs[17] = {{
    (zM - refM) / 0.04f,
    vMS * 0.2f,
    rotRad[0], rotRad[1], rotRad[2], normRot,
    omegaRadS[0] * 0.25f, omegaRadS[1] * 0.25f, omegaRadS[2] * 0.25f,
    accionesPrev[0], accionesPrev[1], accionesPrev[2], accionesPrev[3],
    accionesPrev[4], accionesPrev[5], accionesPrev[6], accionesPrev[7]
  }};
  float act[8];
  evaluar_actor(obs, act);
  for (int i = 0; i < 8; ++i) {{
    accionesPrev[i] = act[i];
    float f = TRIM_N[i] + ACTION_SCALE_N * act[i];
    if (f < 0.0f) f = 0.0f;
    if (f > 6.0f) f = 6.0f;
    fuerzasOut[i] = f;
  }}
}}

}}  // namespace dsm_ppo
"""
    OUT.parent.mkdir(parents=True, exist_ok=True)
    temporal = OUT.with_suffix(OUT.suffix + '.tmp')
    temporal.write_text(code)
    temporal.replace(OUT)
    manifiesto = {'checkpoint': str(CKPT), 'checkpoint_sha256': sha,
                  'normalizer_eps': normalizer_eps,
                  'header_sha256': hashlib.sha256(code.encode()).hexdigest(),
                  'evaluacion': reporte, 'estado': 'exportado; no implica carga ni validación física'}
    OUT.with_suffix('.json').write_text(json.dumps(manifiesto, indent=2, ensure_ascii=False) + '\n')
    print(f"Exportado {OUT} desde {CKPT} ({OUT.stat().st_size} bytes)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--salida', type=Path, default=OUT)
    parser.add_argument('--evaluacion', type=Path)
    args = parser.parse_args()
    exportar(args.checkpoint, args.salida, args.evaluacion)


if __name__ == "__main__":
    main()
