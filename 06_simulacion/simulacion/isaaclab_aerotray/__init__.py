"""Registro externo de AeroTray; no modifica el código de Isaac Lab."""
import gymnasium as gym

gym.register(
    id='DSM-AeroTray-Direct-v0',
    entry_point='isaaclab_aerotray.env:AeroTrayEnv',
    disable_env_checker=True,
    kwargs={
        'env_cfg_entry_point':'isaaclab_aerotray.config:AeroTrayCfg',
        'rsl_rl_cfg_entry_point':'isaaclab_aerotray.agent:AeroTrayPPOCfg',
        'default_agent':'rsl_rl',
    },
)
