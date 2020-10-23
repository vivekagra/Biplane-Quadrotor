import gym
import time
import numpy as np, pandas as pd
import matplotlib.pyplot as plt
import torch

from rl_quad.utils.utils import *
from rl_quad.utils.quadPlot import plot_quad_3d
from rl_quad.environment.continous import QuadEnvCont as Environment
from rl_quad.rl_algorithms.ddpg.ddpg_main import DDPGagent
from rl_quad.rl_algorithms.ddpg.ddpg_utils import OUNoise, NormalizedEnv
from rl_quad.rl_algorithms.ddpg.actor_critic import Actor, Critic


if __name__ == "__main__":
    if torch.cuda.is_available():
        print("GPU is available")
        device = torch.device("cuda")
    else:
        print("..no GPU is available. Using CPU..")
        torch.device("cpu")

    env = Environment()
    agent = DDPGagent(env)
    noise = OUNoise(env.action_space)    

    batch_size = 32
    total_episodes = 500    
    max_number_of_steps = 5000
    start_time = time.time()
    print(env.action_space)
    highest_reward = 0

    episode_wise_rewards = []

    # Episodic training, do not need to use discounted future rewards over here    
    for x in range(total_episodes):
        done = False
        env.reset()
        noise.reset()
        episode_reward = 0
        
        state = env.get_state()
        print(state)
        agent.print_info_training()
        # accumulating actions and noise over the current episode
        for i in range(max_number_of_steps):
            
            # Pick an action based on the current state
            action = agent.get_action(state)
            action = noise.get_action(action, i)
            print(action*60)
            new_state, reward, done, distance = env.step(action)
            agent.memory.push(state, action, reward, new_state, done)
            state = new_state

            if len(agent.memory) > batch_size:
                agent.update(batch_size)
            
            episode_reward += reward
            if(i%25==0):
                print("Episode: {}, Timestep: {}, Distance: {}, Reward: {}, Cumulated Reward: {}".format(x, i, distance, reward, episode_reward))
                print("---------------------------------------------------------------------------------------------")
        
            if done:
                print("Episode ended".format(x, i))
                break         
            
        episode_wise_rewards.append([episode_reward])
    

    print(episode_wise_rewards)
    np.save('episode_wise_rewards.npy', episode_wise_rewards)
    agent.save_models("./")
