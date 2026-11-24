"""Entorno Isaac Lab real con Newton/MJWarp y ocho fuerzas aprendidas."""
import json
import os
from pathlib import Path
import torch
from isaaclab.envs import DirectRLEnv
from isaaclab.utils.math import sample_uniform


class AeroTrayEnv(DirectRLEnv):
    def __init__(self,cfg,render_mode=None,**kwargs):
        super().__init__(cfg,render_mode,**kwargs)
        self.etapa_curriculo=int(os.environ.get('DSM_ETAPA_CURRICULO','2'))
        self.escala_rafaga={1:0.7,2:1.0,3:1.35,4:1.15}.get(self.etapa_curriculo,1.0)
        self.peso_fino={1:1.4,2:1.8,3:2.2,4:3.0}.get(self.etapa_curriculo,1.4)
        self.robot=self.scene['robot']
        sliders,_=self.robot.find_joints('elevacion')
        assert len(sliders)==1
        self.slider=sliders[0]
        self.rotations=[i for i in range(self.robot.num_joints) if i!=self.slider]
        bodies,_=self.robot.find_bodies('conjunto_rotatorio')
        assert len(bodies)==1
        self.upper=bodies[0]
        self.body_ids=torch.tensor(bodies,device=self.device,dtype=torch.int32)
        meta=json.loads((Path(__file__).resolve().parents[1]/'supuestos_conjunto_cad.json').read_text())
        pivot=torch.tensor(meta['centro_rotula_modelo_m'],device=self.device)
        positions=torch.tensor([meta['centros_empuje_modelo_m'][str(i)] for i in range(1,9)],device=self.device)-pivot
        yaw=torch.tensor([.015]*4+[-.015]*4,device=self.device)
        self.allocation=torch.stack((torch.ones(8,device=self.device),positions[:,1],-positions[:,0],yaw))
        self.trim=torch.tensor(meta['trim_empuje_ideal_N'],device=self.device)
        self.actions=torch.zeros((self.num_envs,8),device=self.device)
        self.previous_actions=torch.zeros_like(self.actions)
        self.forces=torch.zeros((self.num_envs,1,3),device=self.device)
        self.torques=torch.zeros_like(self.forces)
        self.target_z=torch.full((self.num_envs,),self.cfg.target_height,device=self.device)
        self.motor_gain=torch.ones((self.num_envs,8),device=self.device)
        self.mass_bias_f=torch.zeros(self.num_envs,device=self.device)
        self.dist_force_z=torch.zeros(self.num_envs,device=self.device)
        self.dist_torque=torch.zeros((self.num_envs,3),device=self.device)
        self.dist_steps_left=torch.zeros(self.num_envs,device=self.device,dtype=torch.long)
        self.env_phase=torch.arange(self.num_envs,device=self.device)
        print('[AeroTray] joints:',self.robot.joint_names,'bodies:',self.robot.body_names)

    def _pre_physics_step(self,actions):
        self.previous_actions.copy_(self.actions)
        self.actions=actions.clamp(-1,1)
        step=self.episode_length_buf
        if getattr(self.cfg,'enable_multi_height',False):
            # Mitad de los drones siguen rampas continuas de subida/bajada y mitad escalones desfasados
            es_rampa=(self.env_phase%2==0)
            if es_rampa.any():
                fase_t=((step[es_rampa]+self.env_phase[es_rampa]*23)%200).float()/200.0
                tri=1.0-torch.abs(2.0*fase_t-1.0)
                self.target_z[es_rampa]=self.cfg.min_target_height+tri*(self.cfg.max_target_height-self.cfg.min_target_height)
            cambio=(~es_rampa)&(((step+self.env_phase*13)%90==0)&(step>0))
            if cambio.any():
                ids=cambio.nonzero(as_tuple=False).squeeze(-1)
                self.target_z[ids]=sample_uniform(
                    self.cfg.min_target_height,self.cfg.max_target_height,(len(ids),),self.device)
        if getattr(self.cfg,'enable_disturbances',False):
            activo=self.dist_steps_left>0
            self.dist_steps_left[activo]-=1
            fin=activo&(self.dist_steps_left==0)
            if fin.any():
                self.dist_force_z[fin]=0.
                self.dist_torque[fin]=0.
            disparo=((step+self.env_phase*17)%75==25)
            if disparo.any():
                ids=disparo.nonzero(as_tuple=False).squeeze(-1)
                n=len(ids)
                s=self.escala_rafaga
                self.dist_force_z[ids]=sample_uniform(-self.cfg.disturb_force_range*s,self.cfg.disturb_force_range*s,(n,),self.device)
                txy=sample_uniform(-self.cfg.disturb_torque_xy_range*s,self.cfg.disturb_torque_xy_range*s,(n,2),self.device)
                tz=sample_uniform(-self.cfg.disturb_torque_z_range*s,self.cfg.disturb_torque_z_range*s,(n,1),self.device)
                self.dist_torque[ids]=torch.cat((txy,tz),dim=1)
                self.dist_steps_left[ids]=18
                # Aplicar tambien escalon de inclinacion de rampa / golpe angular visible (+-12 deg)
                pos=self.robot.data.joint_pos.torch[ids].clone()
                vel=self.robot.data.joint_vel.torch[ids].clone()
                salto_ang=sample_uniform(-.21*s,.21*s,(n,len(self.rotations)),self.device)
                salto_ang[:,2]*=.35
                pos[:,self.rotations]=(pos[:,self.rotations]+salto_ang).clamp(-.42,.42)
                vel[:,self.rotations]+=sample_uniform(-.45*s,.45*s,(n,len(self.rotations)),self.device)
                self.robot.write_joint_state_to_sim_index(position=pos,velocity=vel,env_ids=ids)

    def _apply_action(self):
        motor_forces=((self.trim+self.cfg.action_scale*self.actions)*self.motor_gain).clamp(0,6)
        wrench=motor_forces@self.allocation.T
        self.forces[:,:,2]=(wrench[:,0]+self.mass_bias_f+self.dist_force_z)[:,None]
        self.torques[:,0,:]=wrench[:,1:]+self.dist_torque
        self.robot.permanent_wrench_composer.set_forces_and_torques_index(
            forces=self.forces,torques=self.torques,body_ids=self.body_ids,is_global=False)

    def _state(self):
        pos=self.robot.data.joint_pos.torch
        vel=self.robot.data.joint_vel.torch
        return pos[:,self.slider],vel[:,self.slider],pos[:,self.rotations],vel[:,self.rotations]

    def _get_observations(self):
        z,v,angles,omega=self._state()
        # Rotación vectorial de la rótula: 3 componentes, más su norma.
        orientation=torch.cat((angles,torch.linalg.vector_norm(angles,dim=1,keepdim=True)),dim=1)
        obs=torch.cat(((z-self.target_z)[:,None]/.04,v[:,None]*.2,
                       orientation,omega*.25,self.actions),dim=1)
        return {'policy':obs}

    def _get_rewards(self):
        z,v,angles,omega=self._state()
        err_z=z-self.target_z
        tilt_sq=angles[:,:2].square().sum(1)
        yaw_sq=angles[:,2].square()
        coarse=torch.exp(-900*err_z.square()-35*tilt_sq-4*yaw_sq)
        fine=torch.exp(-4200*err_z.square()-420*tilt_sq-25*yaw_sq)
        reward=1.6*coarse+1.4*fine+.5-.025*omega.square().sum(1)-.12*v.square()
        reward-=.004*self.actions.square().sum(1)+.012*(self.actions-self.previous_actions).square().sum(1)
        reward-=2*self.reset_terminated.float()
        return reward*self.step_dt

    def _get_dones(self):
        z,v,angles,omega=self._state()
        failed=(z<-.002)|(z>self.cfg.max_height+.002)|(torch.linalg.vector_norm(angles[:,:2],dim=1)>.523599)
        failed|=torch.linalg.vector_norm(omega,dim=1)>6
        failed|=~torch.isfinite(z)|~torch.isfinite(angles).all(dim=1)
        return failed,self.episode_length_buf>=self.max_episode_length

    def _reset_idx(self,env_ids):
        if env_ids is None:env_ids=self.robot._ALL_INDICES
        self.extras.setdefault('log',{})['Metrics/success_rate']=self.reset_time_outs[env_ids].float().mean()
        super()._reset_idx(env_ids)
        n=len(env_ids)
        if getattr(self.cfg,'enable_multi_height',False):
            self.target_z[env_ids]=sample_uniform(self.cfg.min_target_height,self.cfg.max_target_height,(n,),self.device)
        else:
            self.target_z[env_ids]=self.cfg.target_height
        if getattr(self.cfg,'enable_disturbances',False):
            self.motor_gain[env_ids]=sample_uniform(.94,1.06,(n,8),self.device)
            self.mass_bias_f[env_ids]=sample_uniform(-1.8,1.8,(n,),self.device)
        else:
            self.motor_gain[env_ids]=1.
            self.mass_bias_f[env_ids]=0.
        self.dist_force_z[env_ids]=0.
        self.dist_torque[env_ids]=0.
        self.dist_steps_left[env_ids]=0
        pos=self.robot.data.default_joint_pos.torch[env_ids].clone()
        vel=self.robot.data.default_joint_vel.torch[env_ids].clone()
        z0=(self.target_z[env_ids]+sample_uniform(-self.cfg.initial_height_range,self.cfg.initial_height_range,(n,),self.device)).clamp(.005,self.cfg.max_height-.005)
        pos[:,self.slider]=z0 if getattr(self.cfg,'enable_multi_height',False) else (
            self.cfg.target_height+sample_uniform(-self.cfg.initial_height_range,self.cfg.initial_height_range,(n,),self.device))
        pos[:,self.rotations]=sample_uniform(-self.cfg.initial_angle_range,self.cfg.initial_angle_range,
                                             (n,len(self.rotations)),self.device)
        vel[:,self.slider]=sample_uniform(-self.cfg.initial_velocity_range,self.cfg.initial_velocity_range,(n,),self.device)
        vel[:,self.rotations]=sample_uniform(-self.cfg.initial_omega_range,self.cfg.initial_omega_range,(n,len(self.rotations)),self.device)
        self.robot.write_joint_state_to_sim_index(position=pos,velocity=vel,env_ids=env_ids)
        self.actions[env_ids]=0;self.previous_actions[env_ids]=0
