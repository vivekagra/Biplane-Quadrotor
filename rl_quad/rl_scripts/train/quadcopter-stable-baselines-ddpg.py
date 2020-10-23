import gym
import time
import numpy as np, pandas as pd
import matplotlib.pyplot as plt

from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from stable_baselines import DDPG
from rl_quad.environment.continous import QuadEnvCont

env = QuadEnvCont()

n_actions = env.action_space.shape[-1]
param_noise = None
action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

model = DDPG(MlpPolicy, env, verbose=1, param_noise=param_noise, action_noise=action_noise, tensorboard_log="/home/ayush/Projects/rl_quad/training/logs/")
model.learn(total_timesteps=100000)
model.save("quad-ddpg-v1-100k")

model.learn(total_timesteps=100000)
model.save("quad-ddpg-v1-200k")

model.learn(total_timesteps=200000)
model.save("quad-ddpg-v1-400k")
