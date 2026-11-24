"""Plugin Python ejecutado dentro de Gazebo Harmonic (física DART)."""
import csv
import json
import os
import numpy as np
from gz.sim8 import Model, Link, World, world_entity
from gz.math7 import Vector3d, Pose3d, Quaterniond
from gz.transport13 import Node
from gz.msgs10.stringmsg_pb2 import StringMsg
from control import CFG, ROOT, G, Sensors, FirmwareControl
from ball_plate import BallPlate


class OctoSystem:
    def configure(self, entity, sdf, ecm, event_mgr):
        self.model = Model(entity); self.body = Link(self.model.link_by_name(ecm,'dron'))
        self.body.enable_velocity_checks(ecm)
        self.mode = os.environ.get('DSM_SIM_MODO', 'banco')
        self.seed = int(os.environ.get('DSM_SIM_SEED','73021'))
        self.sensors = Sensors(seed=self.seed,noise=os.environ.get('DSM_SIM_RUIDO','1')=='1')
        self.controller = FirmwareControl(self.mode,float(os.environ.get('DSM_SIM_GAIN','1')))
        self.disabled = os.environ.get('DSM_SIM_SIN_CONTROL','0')=='1'
        self.forces = np.zeros(8); self.held = np.zeros(8)
        if self.mode=='altura_ideal':
            self.controller.reference=CFG['altura']['altura_inicial_ideal_m']
            self.forces[:]=(CFG['masa_rotatoria_kg']+CFG['masa_carro_vertical_kg'])*G/8
            self.held=self.forces.copy()
        self.last_control = -.005; self.last_output = -.020; self.last_report = -.020
        self.old_w = np.zeros(3); self.old_v = np.zeros(3)
        self.alpha=np.zeros(3); self.acceleration=np.zeros(3)
        gains=json.loads(os.environ['DSM_SIM_BP_GAINS']) if 'DSM_SIM_BP_GAINS' in os.environ else None
        self.ball=BallPlate(gains=gains,seed=self.seed+100)
        self.ball.feedback=os.environ.get('DSM_SIM_BP_ABIERTO','0')!='1'
        self.world=World(world_entity(ecm))
        self.plate_visual=None; self.ball_visual=None
        self.node = Node(); self.publisher = self.node.advertise('/dsm_sim/estado',StringMsg)
        self.path = ROOT/'resultados'/os.environ.get('DSM_SIM_CSV',self.mode+'.csv')
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.file = self.path.open('w',buffering=1); self.writer = csv.writer(self.file)
        self.writer.writerow(['t_s','z_m','roll_deg','pitch_deg','yaw_deg','est_roll_deg','est_pitch_deg',
            'p_dps','q_dps','r_dps','tof_m','tof_valido','z_est_m','escala_RP','corte',
            *[f'M{i}_us' for i in range(1,9)],*[f'F{i}_N' for i in range(1,9)],'accel_g',
            'bola_x_m','bola_y_m','bola_vx_ms','bola_vy_ms','servo_x_us','servo_y_us',
            'placa_x_rad','placa_y_rad','bola_perdida'])
        self.last_state = None
        print(f'[DSM SIM] Gazebo: {self.mode}; registro {self.path}',flush=True)

    def pre_update(self, info, ecm):
        if info.paused: return
        if self.plate_visual is None:
            pe=self.world.model_by_name(ecm,'placa_visual'); be=self.world.model_by_name(ecm,'bola_visual')
            if pe and be:
                self.plate_visual=Model(pe); self.ball_visual=Model(be)
        t = info.sim_time.total_seconds(); dt = info.dt.total_seconds()
        pose = self.body.world_pose(ecm); wg = self.body.world_angular_velocity(ecm)
        vg = self.body.world_linear_velocity(ecm)
        if pose is None or wg is None or vg is None or dt<=0: return
        q = pose.rot(); rpy = np.array([q.roll(),q.pitch(),q.yaw()])
        # Rotación de vectores mediante API de Gazebo; filas por ejes transformados.
        axes = [q.rotate_vector(Vector3d(*e)) for e in np.eye(3)]
        R = np.array([[v.x(),v.y(),v.z()] for v in axes]).T
        w = R.T@np.array([wg.x(),wg.y(),wg.z()]); v = np.array([vg.x(),vg.y(),vg.z()])
        z = pose.pos().z()
        if t-self.last_control >= .005-1e-7:
            delta = t-self.last_control
            alpha = (w-self.old_w)/delta; accel = (v-self.old_v)/delta
            self.alpha=alpha; self.acceleration=accel
            self.old_w=w.copy(); self.old_v=v.copy(); self.last_control=t
            self.sensors.update_imu(R,w,alpha,accel,delta)
            dropout = os.environ.get('DSM_SIM_TOF_FALLO','0')=='1' and 3<t<3.3
            self.sensors.update_tof(R,z,t,dropout)
            self.controller.update(self.sensors,delta)
        self.ball.step(t,dt,rpy,w,self.alpha,R.T@self.acceleration)
        if t-self.last_output >= .020-1e-7:
            self.last_output=t; self.held=self.controller.request.copy()
            if self.disabled: self.held[:]=0
        self.forces += (1-np.exp(-dt/CFG['actuadores']['tau_s']))*(self.held-self.forces)
        xy = self.controller.xy; fz=float(self.forces.sum())
        torque = np.array([xy[:,1]@self.forces, -xy[:,0]@self.forces,
                          CFG['actuadores']['par_empuje_m']*np.r_[np.ones(4),-np.ones(4)]@self.forces])
        torque -= .035*w
        torque += self.ball.reaction_delta()[0]
        # Perturbación física aplicada, sin teletransportar el estado.
        if os.environ.get('DSM_SIM_PERTURBAR','1')=='1':
            if 1<=t<1.12: torque[:2] += [.055,-.040]
            if 4<=t<4.12: torque[:2] += [-.040,.055]
        global_force=R@np.array([0,0,fz]); global_torque=R@torque
        self.body.add_world_wrench(ecm,Vector3d(*global_force),Vector3d(*global_torque))
        if t-self.last_report >= .020-1e-7:
            self.last_report=t
            s=self.sensors; c=self.controller
            bp=self.ball
            plate_q=q*Quaterniond(-float(bp.angle[0,1]),float(bp.angle[0,0]),0.)
            plate_pos=pose.pos()+q.rotate_vector(Vector3d(0,0,CFG['ball_plate']['altura_placa_m']))
            ball_pos=plate_pos+plate_q.rotate_vector(Vector3d(*map(float,[*bp.position[0],CFG['ball_plate']['diametro_bola_m']/2])))
            if self.plate_visual is not None:
                self.plate_visual.set_world_pose_cmd(ecm,Pose3d(plate_pos,plate_q))
                self.ball_visual.set_world_pose_cmd(ecm,Pose3d(ball_pos,plate_q))
            state={'t':t,'modo':self.mode,'z':z-CFG['pivot_altura_m'],'rpy':rpy.tolist(),
                'angles':s.angles.tolist(),'omega':s.gyro.tolist(),'tof':float(s.tof) if s.valid else None,
                'tof_valid':s.valid,'height':float(s.height) if np.isfinite(s.height) else None,
                'scale':c.scale,'cut':c.cut,'pwm':c.pwm.tolist(),'force':self.forces.tolist(),'accel_g':s.norm,'accel':s.accel.tolist(),
                'ball':bp.position[0].tolist(),'ball_v':bp.velocity[0].tolist(),'plate':bp.angle[0].tolist(),
                'servos':bp.pulse[0].tolist(),'ball_lost':bool(bp.lost[0])}
            msg=StringMsg(); msg.data=json.dumps(state,allow_nan=False); self.publisher.publish(msg)
            self.writer.writerow([t,state['z'],*np.rad2deg(rpy),*np.rad2deg(s.angles[:2]),
                *np.rad2deg(s.gyro),s.tof,int(s.valid),s.height,c.scale,c.cut,*c.pwm,*self.forces,s.norm,
                *bp.position[0],*bp.velocity[0],*bp.pulse[0],*bp.angle[0],int(bp.lost[0])])
            self.last_state=state


def get_system(): return OctoSystem()
