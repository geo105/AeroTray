"""Reserva reproducible: los mismos controles con o sin política residual."""
import argparse
import json
import csv
from pathlib import Path
import numpy as np
import torch
import gymnasium as gym
from isaaclab.app import add_launcher_args, launch_simulation
import isaac_task  # noqa: F401
from isaac_task.config import OctoCfg, OctoPPOCfg
from control import ROOT


def main():
    p=argparse.ArgumentParser(); p.add_argument('--checkpoint')
    p.add_argument('--num_envs',type=int,default=8); p.add_argument('--segundos',type=float,default=6.)
    p.add_argument('--seed',type=int,default=81243); p.add_argument('--salida',type=Path)
    add_launcher_args(p); a=p.parse_args()
    if a.segundos<=0 or a.num_envs<1: p.error('Duración y entornos deben ser positivos')
    cfg=OctoCfg(); cfg.scene.num_envs=a.num_envs; cfg.seed=a.seed; cfg.episode_length_s=a.segundos+2
    cfg.sim.physics=cfg.sim.physics.newton_mjwarp
    with launch_simulation(cfg,a):
        raw=gym.make('DSM-Octo-Actual-Residual-v0',cfg=cfg); env=raw.unwrapped
        wrapper=None; policy=None
        if a.checkpoint:
            from isaaclab_rl.rsl_rl import (RslRlVecEnvWrapper,create_rsl_rl_runner,
                check_rsl_rl_version,handle_deprecated_rsl_rl_cfg)
            wrapper=RslRlVecEnvWrapper(raw,clip_actions=1.)
            agent=handle_deprecated_rsl_rl_cfg(OctoPPOCfg(),check_rsl_rl_version())
            runner=create_rsl_rl_runner(wrapper,agent); runner.load(a.checkpoint)
            policy=runner.get_inference_policy(device=env.device)
        torch.manual_seed(a.seed); raw.reset(seed=a.seed)
        obs=wrapper.get_observations() if wrapper else None
        alive=np.ones(env.num_envs,dtype=bool); tilt_log=[]; ball_log=[]; masks=[]; action_log=[]
        trajectory=[]
        with torch.inference_mode():
            for _ in range(round(a.segundos/env.step_dt)):
                actions=policy(obs) if policy else torch.zeros((env.num_envs,4),device=env.device)
                if wrapper:
                    obs,_,dones,_=wrapper.step(actions); failed=dones.cpu().numpy().astype(bool); policy.reset(dones)
                else:
                    _,_,done,timeout,_=raw.step(actions); failed=done.cpu().numpy().astype(bool)
                alive&=~failed
                masks.append(alive.copy()); tilt_log.append((env.angles[:,:2]**2).sum(1))
                ball_log.append(np.array([(b.position[0]**2).sum() for b in env.balls]))
                action_log.append(actions.cpu().numpy())
                if alive[0]:
                    s=env.sensors[0]; c=env.controllers[0]; b=env.balls[0]
                    trajectory.append([len(tilt_log)*env.step_dt,0.,*np.rad2deg(env.angles[0]),
                        *np.rad2deg(s.angles[:2]),*np.rad2deg(s.gyro),s.tof,int(s.valid),s.height,c.scale,c.cut,
                        *c.pwm,*env.motor[0],s.norm,*b.position[0],*b.velocity[0],*b.pulse[0],
                        *b.angle[0],int(b.lost[0])])
        mask=np.array(masks); tilt=np.array(tilt_log); ball=np.array(ball_log); actions=np.array(action_log)
        tail=min(len(mask),round(2/env.step_dt)); tail_mask=mask[-tail:]
        result={'motor_fisico':'Isaac Lab / Newton MJWarp CUDA','modo':'residual' if policy else 'controles_originales',
            'checkpoint':str(Path(a.checkpoint).resolve()) if a.checkpoint else None,
            'seed_reserva':a.seed,'entornos':env.num_envs,'duracion_s':a.segundos,
            'episodios_completos':int(alive.sum()),'supervivencia':float(alive.mean()),
            'rms_tilt_deg':float(np.rad2deg(np.sqrt(tilt[mask].mean()))) if mask.any() else None,
            'rms_bola_mm':float(1000*np.sqrt(ball[mask].mean())) if mask.any() else None,
            'rms_tilt_ultimos_2s_deg':float(np.rad2deg(np.sqrt(tilt[-tail:][tail_mask].mean()))) if tail_mask.any() else None,
            'rms_bola_ultimos_2s_mm':float(1000*np.sqrt(ball[-tail:][tail_mask].mean())) if tail_mask.any() else None,
            'rms_residuo_normalizado':float(np.sqrt((actions**2).mean())),
            'nota':'Junta fija vertical. Controles originales + 4 residuos acotados. Pelota reducida, sensores/actuadores provisionales. RMS incluye solo muestras de episodios vivos; supervivencia se informa por separado.'}
        output=a.salida or ROOT/'resultados'/('isaac_residual.json' if policy else 'isaac_originales.json')
        output.parent.mkdir(parents=True,exist_ok=True); output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        with output.with_suffix('.csv').open('w') as f:
            writer=csv.writer(f)
            writer.writerow(['t_s','z_m','roll_deg','pitch_deg','yaw_deg','est_roll_deg','est_pitch_deg',
                'p_dps','q_dps','r_dps','tof_m','tof_valido','z_est_m','escala_RP','corte',
                *[f'M{i}_us' for i in range(1,9)],*[f'F{i}_N' for i in range(1,9)],'accel_g',
                'bola_x_m','bola_y_m','bola_vx_ms','bola_vy_ms','servo_x_us','servo_y_us',
                'placa_x_rad','placa_y_rad','bola_perdida'])
            writer.writerows(trajectory)
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        raw.close()


if __name__=='__main__': main()
