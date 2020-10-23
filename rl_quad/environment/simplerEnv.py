import numpy as np
import csv
import gym
from gym import utils as gym_utils, spaces
from rl_quad.environment.model.physicsSim import PhysicsSim

class Environment(gym.Env):
    def __init__(self, init_pose=None, init_velocities=None, 
        init_angle_velocities=None, runtime=5., target_pos=None):
        self.sim = PhysicsSim(init_pose, init_velocities, init_angle_velocities, runtime) 
        bounds = 50
        self.observation_space=spaces.Box(low=np.array([-bounds, -bounds, -bounds, -np.pi, -np.pi, -np.pi, -bounds, -bounds, -bounds, -bounds, -bounds, -bounds]), 
                                            high=np.array([bounds,  bounds,  bounds,  np.pi,  np.pi,  np.pi, bounds,  bounds,  bounds,  bounds,  bounds,  bounds]))
        self.reward_range = (-np.inf, np.inf)

        self.action_space = spaces.Box(low=-1*np.ones(4), high=np.ones(4))
        self.target_pos = target_pos if target_pos is not None else np.array([0., 0., 10.]) 

    def get_reward(self):
        """Uses current pose and velocity of sim to return reward."""
        reward = 1.-.3*(abs(self.sim.pose[:3] - self.target_pos)).sum()
        reward = max(1, min(-1, reward)) 
        return reward

    def step(self, action):
        """Uses action to obtain next state, reward, done."""
        reward = 0
        print("Action: " + str(action))
        rotor_speeds = np.array(list(map(lambda x: x*45+45, action)))
        print("RS: " + str(rotor_speeds))
        print(self.sim.get_state())
        done = self.sim.next_timestep(rotor_speeds)
        reward = self.get_reward()
        observation = self.sim.get_state()
        
        return observation, reward, done, {}

    def reset(self):
        """Reset the sim to start a new episode."""
        self.sim.reset()
        self.sim.set_state(self.observation_space.sample())
        return self.sim.get_state()