"""Newton/Isaac Lab instalado; junta fija vertical y pelota de modelo reducido."""
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
from isaaclab_visualizers.viser import ViserVisualizerCfg
from isaaclab_newton.physics import NewtonCfg, MJWarpSolverCfg
from isaaclab_tasks.utils import PresetCfg
from isaaclab_rl.rsl_rl import RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
from control import ROOT


@configclass
class PhysicsCfg(PresetCfg):
    newton_mjwarp=NewtonCfg(solver_cfg=MJWarpSolverCfg(njmax=32,nconmax=8,
        integrator='implicitfast',disable_contacts=True,disable_sensors=True),
        num_substeps=1,use_cuda_graph=True,debug_mode=False)
    default=newton_mjwarp


@configclass
class SceneCfg(InteractiveSceneCfg):
    robot=ArticulationCfg(prim_path='{ENV_REGEX_NS}/Robot',
        spawn=sim_utils.UsdFileCfg(usd_path=(ROOT/'usd_path.txt').read_text().strip()),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos={'rotula.*':0.},joint_vel={'.*':0.}),actuators={})
    light=AssetBaseCfg(prim_path='/World/Light',spawn=sim_utils.DistantLightCfg(intensity=2500.))


@configclass
class OctoCfg(DirectRLEnvCfg):
    decimation=20
    episode_length_s=8.
    action_space=4
    observation_space=16
    state_space=0
    sim=SimulationCfg(dt=.001,render_interval=decimation,gravity=(0.,0.,-9.81),physics=PhysicsCfg())
    scene=SceneCfg(num_envs=8,env_spacing=1.6,replicate_physics=True)

    def __post_init__(self):
        self.sim.default_visualizer_cfg=ViserVisualizerCfg(eye=(2.6,-2.6,1.7),lookat=(0.,0.,.55),
            bind_address='127.0.0.1',display_address='127.0.0.1',port=8081,share=False,open_browser=False)


@configclass
class OctoPPOCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env=32
    max_iterations=100
    save_interval=25
    experiment_name='octo_actual_residual'
    clip_actions=1.
    obs_groups={'actor':['policy'],'critic':['policy']}
    actor=RslRlMLPModelCfg(hidden_dims=[32,32],activation='elu',obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=.2))
    critic=RslRlMLPModelCfg(hidden_dims=[32,32],activation='elu',obs_normalization=True)
    algorithm=RslRlPpoAlgorithmCfg(value_loss_coef=1.,use_clipped_value_loss=True,clip_param=.2,
        entropy_coef=.001,num_learning_epochs=4,num_mini_batches=4,learning_rate=.0003,
        schedule='adaptive',gamma=.99,lam=.95,desired_kl=.01,max_grad_norm=1.)
