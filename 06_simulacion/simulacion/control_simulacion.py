"""Control de referencia para el modelo restringido de AeroTray.

Sólo simulación: estado perfecto o ruido sintético, fuerzas en N y parámetros
provisionales. No genera PWM ni accede a una placa o a motores físicos.
"""
import numpy as np
import mujoco
from scipy.optimize import lsq_linear


def rotation_vector(quaternion):
    q=np.asarray(quaternion,dtype=float).copy()
    q/=np.linalg.norm(q)
    if q[0]<0:q=-q
    norm=np.linalg.norm(q[1:])
    if norm<1e-10:return 2*q[1:]
    return q[1:]*(2*np.arctan2(norm,q[0])/norm)


class ControlReferencia:
    """PI-D de altura y actitud, con asignación acotada y anti-windup."""
    def __init__(self,model):
        self.model=model
        self.body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'conjunto_rotatorio')
        joint=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'elevacion')
        self.z_address=int(model.jnt_qposadr[joint])
        self.v_address=int(model.jnt_dofadr[joint])
        self.range=model.jnt_range[joint].copy()
        self.target=float(self.range.mean())
        self.mass=float(model.body_mass.sum())
        self.gravity=abs(float(model.opt.gravity[2]))
        self.bounds=model.actuator_ctrlrange.copy()
        self.matrix=np.array([[1,model.site_pos[site,1],-model.site_pos[site,0],model.actuator_gear[i,5]]
                              for i,site in enumerate(model.actuator_trnid[:,0])]).T
        assert self.matrix.shape==(4,8) and np.linalg.matrix_rank(self.matrix)==4
        self.inverse=np.linalg.pinv(self.matrix)
        # Prioridad de inclinación sobre altura y de altura sobre guiñada.
        self.weights=np.array([1.,6/.35,6/.35,.2/.015])
        self.weighted=self.weights[:,None]*self.matrix
        self.kp_z=320.;self.kd_z=60.;self.ki_z=160.
        self.kp=np.array([12.,12.,.4])
        self.kd=np.array([2.4,2.4,.45])
        self.ki=np.array([3.,3.,.05])
        self.reset()

    def reset(self):
        self.integral_force=0.
        self.integral_torque=np.zeros(3)
        self.velocity=np.zeros(6)

    def set_target(self,height):
        # Mantener la referencia 5 mm alejada de los extremos modelados.
        self.target=float(np.clip(height,self.range[0]+.005,self.range[1]-.005))

    def allocate(self,desired):
        forces=self.inverse@desired
        limited=bool(np.any(forces<self.bounds[:,0]) or np.any(forces>self.bounds[:,1]))
        if limited:
            result=lsq_linear(self.weighted,self.weights*desired,
                bounds=(self.bounds[:,0],self.bounds[:,1]),method='bvls',tol=1e-8,max_iter=50)
            if not result.success:raise RuntimeError('Asignación de motores no convergió')
            forces=result.x
        return np.clip(forces,self.bounds[:,0],self.bounds[:,1]),limited

    def update(self,data,dt,noise=None):
        mujoco.mj_forward(self.model,data)
        height=float(data.qpos[self.z_address])
        speed=float(data.qvel[self.v_address])
        error=rotation_vector(data.xquat[self.body])
        mujoco.mj_objectVelocity(self.model,data,mujoco.mjtObj.mjOBJ_BODY,self.body,self.velocity,1)
        omega=self.velocity[:3].copy()
        cosine=float(data.xmat[self.body].reshape(3,3)[2,2])
        if noise is not None:
            height+=noise['height'];speed+=noise['speed']
            error+=noise['attitude'];omega+=noise['omega']
        z_error=self.target-height
        vertical=self.mass*self.gravity+self.kp_z*z_error-self.kd_z*speed+self.integral_force
        # El régimen probado es de inclinaciones pequeñas; no se afirma
        # recuperación desde un vehículo invertido.
        cosine_for_control=max(.5,cosine)
        torque=-self.kp*error-self.kd*omega+self.integral_torque
        desired=np.r_[vertical/cosine_for_control,torque]
        forces,limited=self.allocate(desired)
        achieved=self.matrix@forces
        # Anti-windup por retrocálculo del esfuerzo alcanzable.
        self.integral_force=float(np.clip(self.integral_force+dt*(self.ki_z*z_error+
            5*(achieved[0]*cosine_for_control-vertical)),-12,12))
        self.integral_torque=np.clip(self.integral_torque+dt*(-self.ki*error+
            5*(achieved[1:]-torque)),[-1,-1,-.08],[1,1,.08])
        return forces,{'limited':limited,'error_z_m':z_error,'error_attitude_rad':error,
                       'desired_wrench':desired,'achieved_wrench':achieved}

    def configuration(self):
        return {'tipo':'PI-D de altura y actitud con asignación de ocho fuerzas acotadas',
            'masa_nominal_kg':self.mass,'altura_referencia_m':self.target,
            'kp_altura':self.kp_z,'kd_altura':self.kd_z,'ki_altura':self.ki_z,
            'kp_actitud':self.kp.tolist(),'kd_actitud':self.kd.tolist(),'ki_actitud':self.ki.tolist(),
            'limites_fuerza_N':self.bounds.tolist(),
            'limite':'Sólo modelo provisional. Requiere sensores y actuadores identificados antes de aplicarlo al prototipo.'}
