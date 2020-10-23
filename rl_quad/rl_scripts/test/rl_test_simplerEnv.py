%autoreload 2

import sys
import csv
import numpy as np
import gym
from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO1, PPO2
from rl_quad.environment.simplerEnv import Environment


file_output = 'ddpg_agent_takeoff_sim.txt'                         

# Setup
runtime = 5.  
init_pose = np.array([0., 0., 0., 0., 0., 0.])   # initial pose
init_velocities = np.array([0., 0., 0.])         # initial velocities
init_angle_velocities = np.array([0., 0., 0.])   # initial angle velocities

target_pos = np.array([0., 0., 100.])
task = Task(init_pose, init_velocities, init_angle_velocities, runtime, target_pos)
done = False
labels = ['time', 'x', 'y', 'z', 'phi', 'theta', 'psi', 'x_velocity',
          'y_velocity', 'z_velocity', 'phi_velocity', 'theta_velocity',
          'psi_velocity', 'rotor_speed1', 'rotor_speed2', 'rotor_speed3', 'rotor_speed4']
results = {x: [] for x in labels}

# Run the simulation, and save the results.
with open(file_output, 'w') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(labels)
    while True:
        state = agent.reset_episode()
        rotor_speeds = agent.act(state)
        _, _, done = task.step(rotor_speeds)
        to_write = [task.sim.time] + list(task.sim.pose) + list(task.sim.v) + list(task.sim.angular_v) + list(rotor_speeds)
        for ii in range(len(labels)):
            results[labels[ii]].append(to_write[ii])
        writer.writerow(to_write)
        if done:
            break

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
    
mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d', xlabel='x', ylabel='y', zlabel='z')
x = np.append(init_pose[0], results['x'])
y = np.append(init_pose[1], results['y'])
z = np.append(init_pose[2], results['z'])
final_idx = len(x)-1
ax.plot(x, y, z, label='Quadcopter flight trajectory')
ax.plot([init_pose[0]], [init_pose[1]], [init_pose[2]], label='Initial position', color='blue', marker='x')
ax.plot([x[final_idx]], [y[final_idx]], [z[final_idx]], label='Final position', color='green', marker='o')
ax.legend()
plt.show()