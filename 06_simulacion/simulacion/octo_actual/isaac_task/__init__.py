"""Tarea nueva: referencia probada + correcciones residuales acotadas."""
import gymnasium as gym

gym.register(id='DSM-Octo-Actual-Residual-v0',entry_point='isaac_task.env:OctoEnv',
    disable_env_checker=True,kwargs={
        'env_cfg_entry_point':'isaac_task.config:OctoCfg',
        'rsl_rl_cfg_entry_point':'isaac_task.config:OctoPPOCfg','default_agent':'rsl_rl'})
