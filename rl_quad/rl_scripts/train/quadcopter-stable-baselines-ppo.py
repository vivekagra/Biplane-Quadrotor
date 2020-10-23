import gym
from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO1, PPO2
from rl_quad.environment.simplerEnv import Environment

env = Environment()

model = PPO1(MlpPolicy, env, verbose=1, tensorboard_log="/home/ayush/Projects/rl_quad/training/logs/")
model.learn(total_timesteps=500000)
model.save("quad-ppo-v1")
