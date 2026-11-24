"""Tarea de altura/actitud sobre guía y rótula; parámetros aún provisionales."""
from pathlib import Path
import json
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg,AssetBaseCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab_visualizers.viser import ViserVisualizerCfg
from isaaclab_newton.physics import NewtonCfg,MJWarpSolverCfg
from isaaclab_tasks.utils import PresetCfg

ASSET=Path(__file__).resolve().parent/'assets'


@configclass
class AeroPhysicsCfg(PresetCfg):
    newton_mjwarp=NewtonCfg(
        solver_cfg=MJWarpSolverCfg(njmax=32,nconmax=8,integrator='implicitfast',
                                   disable_contacts=True,disable_sensors=True),
        num_substeps=1,use_cuda_graph=True,debug_mode=False,
    )
    default=newton_mjwarp


@configclass
class AeroSceneCfg(InteractiveSceneCfg):
    robot=ArticulationCfg(
        prim_path='{ENV_REGEX_NS}/Robot',
        spawn=sim_utils.UsdFileCfg(usd_path=json.loads((ASSET/'asset.json').read_text())['usd_path']),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={'rotula.*':0.,'elevacion':.03831472969055},joint_vel={'.*':0.},
        ),
        actuators={},
    )
    light=AssetBaseCfg(prim_path='/World/Light',spawn=sim_utils.DistantLightCfg(intensity=2500.))


@configclass
class AeroTrayCfg(DirectRLEnvCfg):
    decimation=10
    episode_length_s=8.
    action_space=8
    observation_space=17
    state_space=0
    sim=SimulationCfg(dt=.002,render_interval=decimation,gravity=(0.,0.,-9.80665),physics=AeroPhysicsCfg())
    scene=AeroSceneCfg(num_envs=64,env_spacing=1.6,replicate_physics=True)
    target_height=.03831472969055
    max_height=.0766294593811
    min_target_height=.010
    max_target_height=.067
    enable_multi_height=True
    enable_disturbances=True
    disturb_force_range=4.2
    disturb_torque_xy_range=.85
    disturb_torque_z_range=.12
    action_scale=1.5
    initial_angle_range=.30
    initial_height_range=.032
    initial_velocity_range=.06
    initial_omega_range=.35

    def __post_init__(self):
        self.sim.default_visualizer_cfg=ViserVisualizerCfg(
            eye=(2.6,-2.6,1.7),lookat=(0.,0.,.45),bind_address='127.0.0.1',
            display_address='127.0.0.1',port=8080,share=False,open_browser=False,
        )
