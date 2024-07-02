"""
Bi-manual inspire hand environment
"""

import gymnasium as gym
from . import agents 
from .direct_pick import InspirePickEnv
from .direct_pick_cfg import InspirePickEnvCfg

gym.register(
  id="Isaac-Inspire-Hand-Direct-Pick",
  entry_point="omni.isaac.lab_tasks.direct.inspire_hand:InspirePickEnv",
  disable_env_checker=True,
  kwargs={
      "env_cfg_entry_point": InspirePickEnv,
      "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
  #      "rsl_rl_cfg_entry_point": agents.rsl_rl_ppo_cfg.ShadowHandPPORunnerCfg,
    },
)
