"""Evaluar la tarea real de Isaac Lab con trim, referencia o política PPO."""
from pathlib import Path
import argparse
import json
import os
import re
import torch
import gymnasium as gym
from isaaclab.app import add_launcher_args,launch_simulation
import isaaclab_aerotray  # noqa: F401
from isaaclab_aerotray.config import AeroTrayCfg
from isaaclab_aerotray.agent import AeroTrayPPOCfg
from isaaclab.utils.math import quat_apply_inverse

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--modo',choices=['trim','referencia','policy'],default='trim')
    parser.add_argument('--checkpoint')
    parser.add_argument('--num_envs',type=int,default=32)
    parser.add_argument('--salida',type=Path,default=ROOT/'isaaclab_aerotray/evaluacion.json')
    parser.add_argument('--nominal',action='store_true',help='Evaluar en altura fija nominal sin perturbaciones')
    parser.add_argument('--etapa',type=int,choices=(1,2,3,4),help='Currículo de evaluación; por defecto se infiere del checkpoint')
    add_launcher_args(parser);args=parser.parse_args()
    coincidencia=re.search(r'curriculo_etapa_([1-4])',args.checkpoint or '')
    etapa=args.etapa if args.etapa is not None else (int(coincidencia[1]) if coincidencia else int(os.environ.get('DSM_ETAPA_CURRICULO','2')))
    os.environ['DSM_ETAPA_CURRICULO']=str(etapa)
    cfg=AeroTrayCfg();cfg.sim.physics=cfg.sim.physics.newton_mjwarp
    cfg.scene.num_envs=args.num_envs;cfg.episode_length_s=10.;cfg.seed=73021
    if args.modo=='trim' or args.nominal:
        cfg.enable_multi_height=False
        cfg.enable_disturbances=False
    if args.modo=='trim':
        cfg.initial_angle_range=cfg.initial_height_range=cfg.initial_velocity_range=cfg.initial_omega_range=0.
    with launch_simulation(cfg,args):
        raw=gym.make('DSM-AeroTray-Direct-v0',cfg=cfg)
        env=raw.unwrapped
        policy=None;wrapper=None
        if args.modo=='policy':
            from isaaclab_rl.rsl_rl import (
                RslRlVecEnvWrapper,create_rsl_rl_runner,
                check_rsl_rl_version,handle_deprecated_rsl_rl_cfg,
            )
            wrapper=RslRlVecEnvWrapper(raw,clip_actions=1.)
            agent=handle_deprecated_rsl_rl_cfg(AeroTrayPPOCfg(),check_rsl_rl_version())
            runner=create_rsl_rl_runner(wrapper,agent)
            runner.load(args.checkpoint);policy=runner.get_inference_policy(device=env.device)
        torch.manual_seed(cfg.seed)
        raw.reset()
        obs=wrapper.get_observations() if wrapper else None
        n=env.num_envs;device=env.device
        alive=torch.ones(n,device=device,dtype=torch.bool)
        z_errors=[];angles_log=[];active_log=[]
        integral_z=torch.zeros(n,device=device)
        integral_t=torch.zeros((n,3),device=device)
        inverse=torch.linalg.pinv(env.allocation)
        dt=env.step_dt
        duration=2. if args.modo=='trim' else 8.
        with torch.inference_mode():
            for _ in range(round(duration/dt)):
                z,v,angles,omega=env._state()
                if args.modo=='trim':actions=torch.zeros((n,8),device=device)
                elif args.modo=='policy':actions=policy(obs)
                else:
                    q=env.robot.data.body_link_quat_w.torch[:,env.upper]
                    w=quat_apply_inverse(q,env.robot.data.body_link_ang_vel_w.torch[:,env.upper])
                    error=env.target_z-z
                    cosine=(1-2*(q[:,0]**2+q[:,1]**2)).clamp(min=.5)
                    vertical=3.6*9.80665+320*error-60*v+integral_z
                    torque=-angles*torch.tensor([12.,12.,.4],device=device)-w*torch.tensor([2.4,2.4,.45],device=device)+integral_t
                    desired=torch.cat(((vertical/cosine)[:,None],torque),dim=1)
                    forces=desired@inverse.T
                    actions=((forces-env.trim)/cfg.action_scale).clamp(-1,1)
                    achieved=(env.trim+cfg.action_scale*actions)@env.allocation.T
                    integral_z=(integral_z+dt*(160*error+5*(achieved[:,0]*cosine-vertical))).clamp(-12,12)
                    integral_t=(integral_t+dt*(-angles*torch.tensor([3.,3.,.05],device=device)+5*(achieved[:,1:]-torque))).clamp(
                        min=torch.tensor([-1.,-1.,-.08],device=device),max=torch.tensor([1.,1.,.08],device=device))
                if wrapper:
                    obs,reward,dones,info=wrapper.step(actions)
                    # Los episodios duran 10 s; durante la evaluación de 8 s
                    # cualquier done implica fallo, no reinicio por tiempo.
                    failed=dones.bool()
                    policy.reset(dones)
                else:
                    _,reward,failed,timeout,info=raw.step(actions)
                alive&=~failed
                z2,_,angles2,_=env._state()
                z_errors.append((z2-env.target_z).clone())
                angles_log.append(angles2.clone());active_log.append(alive.clone())
        z=torch.stack(z_errors);ang=torch.stack(angles_log);active=torch.stack(active_log)
        tail=int(2/dt);mask=active[-tail:]
        result={'modo':args.modo,'checkpoint':args.checkpoint,'entornos':n,'duracion_s':duration,
            'etapa_curriculo':etapa,'escala_rafaga':env.escala_rafaga,'seed':cfg.seed,
            'multi_altura':cfg.enable_multi_height,'perturbaciones':cfg.enable_disturbances,
            'supervivencia_episodio':float(alive.float().mean().cpu()),
            'masa_movil_kg':env.robot.data.body_mass.torch[0].cpu().tolist(),
            'joints':env.robot.joint_names,'rango_elevacion_m':[0,cfg.max_height],
            'rms_altura_ultimos_2s_mm':float(torch.sqrt((z[-tail:][mask]**2).mean()).cpu()*1000) if mask.any() else None,
            'rms_inclinacion_ultimos_2s_deg':float(torch.sqrt((ang[-tail:,:,:2][mask]**2).sum(1).mean()).cpu()*180/torch.pi) if mask.any() else None,
            'max_error_altura_m':float(z.abs().max().cpu()),
            'nota':'Los RMS solo incluyen muestras aún vivas; se informa supervivencia para evitar ocultar fallos.'}
        args.salida.parent.mkdir(exist_ok=True,parents=True);args.salida.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        raw.close()
        if args.modo=='trim':
            assert result['supervivencia_episodio']==1.
            assert result['max_error_altura_m']<.001


if __name__=='__main__':main()
