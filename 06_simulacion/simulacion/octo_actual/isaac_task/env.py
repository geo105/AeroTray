"""Isaac Lab/Newton para ensayar residuos sobre los DOS controles originales.

La física de la junta corre en Newton GPU. Rodadura y sensores se calculan
con los mismos módulos CPU que Gazebo; no se representan contactos/imágenes.
Los dos primeros residuos corrigen par RP (±.025 Nm); los otros dos corrigen
servos de la placa (±10 us). Ninguna acción sale de la simulación.
"""
import numpy as np
import torch
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import sample_uniform, euler_xyz_from_quat, quat_apply_inverse
from control import CFG, Sensors, FirmwareControl, rotation
from ball_plate import BallPlate


class OctoEnv(DirectRLEnv):
    def __init__(self,cfg,render_mode=None,**kwargs):
        super().__init__(cfg,render_mode,**kwargs)
        self.robot=self.scene['robot']
        bodies,_=self.robot.find_bodies('dron'); assert len(bodies)==1
        self.upper=bodies[0]; self.body_ids=torch.tensor(bodies,device=self.device,dtype=torch.int32)
        self.rotations,_=self.robot.find_joints('rotula.*'); assert len(self.rotations)==3
        self.sensors=[Sensors(seed=5000+i) for i in range(self.num_envs)]
        self.controllers=[FirmwareControl() for _ in range(self.num_envs)]
        self.balls=[BallPlate(seed=6000+i) for i in range(self.num_envs)]
        self.old_w=np.zeros((self.num_envs,3)); self.held=np.zeros((self.num_envs,8))
        self.motor=np.zeros_like(self.held); self.residual=np.zeros((self.num_envs,4))
        self.actions=torch.zeros((self.num_envs,4),device=self.device)
        self.forces=torch.zeros((self.num_envs,1,3),device=self.device)
        self.torques=torch.zeros_like(self.forces)
        self.tick=0; self.local_time=np.zeros(self.num_envs); self.angles=np.zeros((self.num_envs,3))
        self.w=np.zeros_like(self.angles); self.xy=np.array(CFG['motores_flu_m'])[:,:2]
        self.reset_counts=np.zeros(self.num_envs,dtype=int)
        print('[Octo actual] Newton + PID angular ZIP + PID Pi/PS3 Eye; junta fija, no altura.',flush=True)

    def _pre_physics_step(self,actions):
        self.actions=actions.clamp(-1,1)
        self.residual=self.actions.detach().cpu().numpy()
        self.local_time=self.episode_length_buf.cpu().numpy()*self.step_dt
        self.tick=0

    def _apply_action(self):
        dt=self.physics_dt
        if self.tick%5==0:
            quat=self.robot.data.body_link_quat_w.torch[:,self.upper]
            self.angles=torch.stack(euler_xyz_from_quat(quat),dim=1).cpu().numpy()
            w=quat_apply_inverse(quat,self.robot.data.body_link_ang_vel_w.torch[:,self.upper]).cpu().numpy()
            alpha=(w-self.old_w)/.005; self.old_w=w.copy(); self.w=w
            for i,(sensor,ctrl,ball) in enumerate(zip(self.sensors,self.controllers,self.balls)):
                R=rotation(self.angles[i]); t=self.local_time[i]+self.tick*dt
                sensor.update_imu(R,w[i],alpha[i],np.zeros(3),.005)
                sensor.update_tof(R,CFG['pivot_altura_m'],t)
                ctrl.update(sensor,.005,self.residual[i,:2]*.025)
                ball.extra_pulse[0]=self.residual[i,2:]*10
                ball.step(t,.005,self.angles[i],w[i],alpha[i])
        if self.tick%20==0:
            self.held=np.stack([c.request for c in self.controllers])
        self.motor+=(1-np.exp(-dt/CFG['actuadores']['tau_s']))*(self.held-self.motor)
        torque=np.stack([self.motor@self.xy[:,1],-self.motor@self.xy[:,0],
            self.motor@(CFG['actuadores']['par_empuje_m']*np.r_[np.ones(4),-np.ones(4)])],axis=1)
        torque-=.035*self.w
        torque+=np.stack([b.reaction_delta()[0] for b in self.balls])
        times=self.local_time+self.tick*dt
        first=(times>=1)&(times<1.12); second=(times>=4)&(times<4.12)
        torque[first,:2]+=[.055,-.040]; torque[second,:2]+=[-.040,.055]
        self.forces[:,0,2]=torch.as_tensor(self.motor.sum(1),device=self.device,dtype=torch.float32)
        self.torques[:,0,:]=torch.as_tensor(torque,device=self.device,dtype=torch.float32)
        self.robot.permanent_wrench_composer.set_forces_and_torques_index(
            forces=self.forces,torques=self.torques,body_ids=self.body_ids,is_global=False)
        self.tick+=1

    def _get_observations(self):
        rows=[]
        for i,(s,c,b) in enumerate(zip(self.sensors,self.controllers,self.balls)):
            rows.append(np.r_[s.angles[:2]*5,s.gyro[:2],c.integral,
                b.pid.previous[0]/30,b.pid.velocity[0]/100,b.pid.integral[0]/100,self.residual[i]])
        return {'policy':torch.as_tensor(np.array(rows),device=self.device,dtype=torch.float32)}

    def _get_rewards(self):
        tilt=(self.angles[:,:2]**2).sum(1)
        ball=np.array([(b.position[0]**2).sum() for b in self.balls])
        value=np.exp(-400*tilt)+np.exp(-ball/.0004)-.02*(self.residual**2).sum(1)
        value-=2*np.array([b.lost[0] or bool(c.cut) for b,c in zip(self.balls,self.controllers)])
        return torch.as_tensor(value,device=self.device,dtype=torch.float32)*self.step_dt

    def _get_dones(self):
        bad=np.array([b.lost[0] or bool(c.cut) for b,c in zip(self.balls,self.controllers)])
        bad|=np.linalg.norm(self.angles[:,:2],axis=1)>np.deg2rad(12)
        return torch.as_tensor(bad,device=self.device),self.episode_length_buf>=self.max_episode_length

    def _reset_idx(self,env_ids):
        if env_ids is None: env_ids=self.robot._ALL_INDICES
        super()._reset_idx(env_ids)
        n=len(env_ids); ids=env_ids.cpu().numpy()
        pos=self.robot.data.default_joint_pos.torch[env_ids].clone()
        vel=self.robot.data.default_joint_vel.torch[env_ids].clone()
        pos[:,self.rotations]=sample_uniform(-.01745,.01745,(n,3),self.device)
        pos[:,self.rotations[2]]=0
        vel[:,self.rotations]=sample_uniform(-.01745,.01745,(n,3),self.device)
        self.robot.write_joint_state_to_sim_index(position=pos,velocity=vel,env_ids=env_ids)
        angles=pos[:,self.rotations].cpu().numpy()
        rates=vel[:,self.rotations].cpu().numpy()
        for j,i in enumerate(ids):
            self.reset_counts[i]+=1; seed=int(self.cfg.seed)+int(i)*1000+int(self.reset_counts[i])
            self.sensors[i]=Sensors(seed=seed); self.sensors[i].angles=angles[j].copy()
            self.controllers[i]=FirmwareControl(); self.balls[i]=BallPlate(seed=seed+500)
            self.balls[i].position[0]=np.random.default_rng(seed).uniform(-.03,.03,2)
            self.angles[i]=angles[j]; self.old_w[i]=rates[j]; self.w[i]=rates[j]
            self.residual[i]=0.; self.motor[i]=0.; self.held[i]=0.
        self.actions[env_ids]=0
