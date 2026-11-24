from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlMLPModelCfg,RslRlOnPolicyRunnerCfg,RslRlPpoAlgorithmCfg


@configclass
class AeroTrayPPOCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env=32
    max_iterations=1000
    save_interval=100
    experiment_name='aerotray_cad'
    clip_actions=1.
    obs_groups={'actor':['policy'],'critic':['policy']}
    actor=RslRlMLPModelCfg(hidden_dims=[64,64],activation='elu',obs_normalization=True,
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=.35))
    critic=RslRlMLPModelCfg(hidden_dims=[64,64],activation='elu',obs_normalization=True)
    algorithm=RslRlPpoAlgorithmCfg(value_loss_coef=1.,use_clipped_value_loss=True,clip_param=.2,
        entropy_coef=.0002,num_learning_epochs=5,num_mini_batches=4,learning_rate=.0005,
        schedule='adaptive',gamma=.99,lam=.95,desired_kl=.01,max_grad_norm=1.)
