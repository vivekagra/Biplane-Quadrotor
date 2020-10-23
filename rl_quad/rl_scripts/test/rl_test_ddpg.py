"""
author: Peter Huang, Antonio Cuni
email: hbd730@gmail.com, anto.cuni@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

from rl_quad.utils.quadPlot import plot_quad_3d

from rl_quad.conventional_control import lqr_controller as lqr
from rl_quad.conventional_control import pid_controller as pid
from rl_quad.conventional_control import df_controller as df1
from rl_quad.conventional_control import df_controller_rotor_drag as df2
from rl_quad.conventional_control import df_controller_rotor_drag_VT as df3
import torch
import rl_quad.utils.trajGen as trajGen
import rl_quad.utils.trajGen3D as trajGen3D
import rl_quad.utils.utils as utils
import rl_quad.environment.model.params as params
from rl_quad.environment.model.quadcopter import Quadcopter
from rl_quad.environment.continous import QuadEnvCont
import numpy as np
from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from stable_baselines import DDPG

import matplotlib.pyplot as plt

animation_frequency = 50
control_frequency = 200 # Hz for attitude control loop
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
time = [0.0]
sim_time = 2*np.pi

# variables to plot
F_t = list()  # Thrust
M_t = list()  # Torque
t_s = list()  # simulation time
d_s = list()  # desired states
q_s = list()  # quadrotor states
w_i = list()  # rotor speeds

# Test: sim_time = 2pi, v = 1.5, helix

def record(name):
    fig0 = plt.figure(figsize=(20,10))
    fig0.tight_layout()
    fig0ax0 = fig0.add_subplot(3,2,1)
    fig0ax1 = fig0.add_subplot(3,2,2)
    fig0ax2 = fig0.add_subplot(3,2,3)
    fig0ax3 = fig0.add_subplot(3,2,4)
    fig0ax4 = fig0.add_subplot(3,2,5)
    fig0ax5 = fig0.add_subplot(3,2,6)

    fig1 = plt.figure(figsize=(20,10))
    fig1.tight_layout()
    fig1ax0 = fig1.add_subplot(3,2,1)
    fig1ax1 = fig1.add_subplot(3,2,2)
    fig1ax2 = fig1.add_subplot(3,2,3)
    fig1ax3 = fig1.add_subplot(3,2,4)
    fig1ax4 = fig1.add_subplot(3,2,5)
    fig1ax5 = fig1.add_subplot(3,2,6)

    weight = params.mass*params.g*np.ones_like(t_s)
    fig0ax0 = utils.add_plots(fig0ax0,t_s,[F_t,weight],["-","--"],["r","k"],["F","m*g"],"Rotor Thrust -F- over time",'t {s}','F {N}')
    fig0ax0.legend(loc='lower right', shadow=True, fontsize='small') 

    # Torques
    u2 = map(lambda a: a[0],M_t) # extract ux for all points in time
    u3 = map(lambda a: a[1],M_t)
    u4 = map(lambda a: a[2],M_t)

    fig0ax1 = utils.add_plots(fig0ax1,t_s,[u2,u3,u4],["-","-","-"],["r","g","b"],["u2","u3","u4"],"Components of torque vector M over time","t {s}","{N*m}")
    fig0ax1.legend(loc='lower right', shadow=True, fontsize='small')

    # X position
    q_x = map(lambda a: a[0][0], q_s)   # get quad x position
    d_x = map(lambda a: a.pos[0], d_s)  # get desired x position
    x_e = map(lambda a,b: 10*(a-b),d_x,q_x)  # compute error

    fig0ax2 = utils.add_plots(fig0ax2,t_s,[q_x,d_x,x_e],["-","--","-"],["g","r","b"],["quad -x","des x","x error (x10)"],"X - axis position of quadrotor","t {s}","x {m}")
    fig0ax2.legend(loc='lower right', shadow=True, fontsize='small')

    # Y position
    q_y = map(lambda a: a[0][1], q_s)
    d_y = map(lambda a: a.pos[1], d_s)
    y_e = map(lambda a,b: 10*(a-b),d_y,q_y)

    fig0ax3 = utils.add_plots(fig0ax3,t_s,[q_y,d_y,y_e],["-","--","-"],["g","r","b"],["quad -y","des y","y error (x10)"],"Y - axis position of quadrotor","t {s}","y {m}")
    fig0ax3.legend(loc='lower right', shadow=True, fontsize='small')

    # Z position
    q_z = map(lambda a: a[0][2], q_s)
    d_z = map(lambda a: a.pos[2], d_s)
    z_e = map(lambda a,b: 10*(a-b),d_z,q_z)

    fig0ax4 = utils.add_plots(fig0ax4,t_s,[q_z,d_z,z_e],["-","--","-"],["g","r","b"],["quad z","des z","z error (x10)"],"Z - axis position of quadrotor","t {s}","z {m}")
    fig0ax4.legend(loc='lower right', shadow=True, fontsize='small')

    # Euler angles
    q_phi = map(lambda a: a[2][0]*180.0/np.pi, q_s)
    q_theta = map(lambda a: a[2][1]*180.0/np.pi, q_s)
    q_psi = map(lambda a: a[2][2]*180.0/np.pi, q_s)

    fig0ax5 = utils.add_plots(fig0ax5,t_s,[q_phi,q_theta,q_psi],["-","-","-"],["r","g","b"],["phi","theta","psi"],"Angular position of quadrotor",'t {s}','phi, theta, psi {degree}')
    fig0ax5.legend(loc='lower right', shadow=True, fontsize='small')

    #  X Linear velocity
    q_vx = map(lambda a: a[1][0], q_s)
    d_vx = map(lambda a: a.vel[0], d_s)   
    vx_e = map(lambda a,b: 10*(a-b),d_vx,q_vx)

    fig1ax0 = utils.add_plots(fig1ax0,t_s,[q_vx,d_vx,vx_e],["-","--","-"],["g","r","b"],["quad Vx","des Vx","Vx error (x10)"],"X axis linear Velocities of quadrotor",'t {s}','Vx {m/s}')
    fig1ax0.legend(loc='lower right', shadow=True, fontsize='small')   

    #  Y Linear velocity
    q_vy = map(lambda a: a[1][1], q_s)
    d_vy = map(lambda a: a.vel[1], d_s)   
    vy_e = map(lambda a,b: 10*(a-b),d_vy,q_vy)

    fig1ax1 = utils.add_plots(fig1ax1,t_s,[q_vy,d_vy,vy_e],["-","--","-"],["g","r","b"],["quad Vy","des Vy","Vy error (x10)"],"Y axis linear Velocities of quadrotor",'t {s}','Vy {m/s}')
    fig1ax1.legend(loc='lower right', shadow=True, fontsize='small')  

    #  Z Linear velocity
    q_vz = map(lambda a: a[1][2], q_s)
    d_vz = map(lambda a: a.vel[2], d_s)   
    vz_e = map(lambda a,b: 10*(a-b),d_vz,q_vz)

    fig1ax2 = utils.add_plots(fig1ax2,t_s,[q_vz,d_vz,vz_e],["-","--","-"],["g","r","b"],["quad Vz","des Vz","Vz error (x10)"],"Z axis linear Velocities of quadrotor",'t {s}','Vz {m/s}')
    fig1ax2.legend(loc='lower right', shadow=True, fontsize='small')  

    # Angular velocities
    q_wx = map(lambda a: a[3][0]*180.0/np.pi, q_s)
    q_wy = map(lambda a: a[3][1]*180.0/np.pi, q_s)
    q_wz = map(lambda a: a[3][2]*180.0/np.pi, q_s)

    fig1ax3 = utils.add_plots(fig1ax3,t_s,[q_wx,q_wy,q_wz],["-","-","-"],["r","g","b"],["wx","wy","wz"],"Angular velocities of quadrotor",'t {s}','wx, wy, wz {degree/s}')
    fig1ax3.legend(loc='lower right', shadow=True, fontsize='small')

    # rotor speeds
    w_0 = map(lambda a: np.sqrt(a[0][0]) if a[0][0] > 0 else -np.sqrt(-a[0][0]),w_i)
    w_1 = map(lambda a: np.sqrt(a[1][0]) if a[1][0] > 0 else -np.sqrt(-a[1][0]),w_i)
    w_2 = map(lambda a: np.sqrt(a[2][0]) if a[2][0] > 0 else -np.sqrt(-a[2][0]),w_i)
    w_3 = map(lambda a: np.sqrt(a[3][0]) if a[3][0] > 0 else -np.sqrt(-a[3][0]),w_i)

    fig1ax4 = utils.add_plots(fig1ax4, t_s, [w_0,w_1,w_2,w_3],["-","-","-","-"],["r","g","b","c"],["w0","w1","w2","w3"],"Rotor Speeds",'t {s}','{rpm}')
    fig1ax4.legend(loc='lower right', shadow=True, fontsize='small')
    # save
    fig0.savefig("t_"+name, dpi = 300) #translation variables
    fig1.savefig("r_"+name, dpi = 300) #rotation variables
    print("Saved t_{} and r_{}.".format(name,name))

if __name__ == "__main__":
    env = QuadEnvCont()
    model = DDPG.load("quad-ddpg-v1")
    obs = env.reset()

    env.quadcopter.state=np.array(np.array([0,0,0,0,0,0,0,0,0,1,0,0,0]))
    waypoints = trajGen3D.get_helix_waypoints(sim_time, 8)
    (coeff_x, coeff_y, coeff_z) = trajGen3D.get_MST_coefficients(waypoints)
    env.set_goal([-coeff_x[0], -coeff_y[0], -coeff_z[0]])
    print("ASDDDDSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSs")
    print(env.get_state())

    def control_loop(i):
    
        for _ in range(int(control_iterations)):
            global obs
            action, _states = model.predict(obs)
            print(action)
            obs, rewards, dones, info = env.step(action)
            
        return env.quadcopter.world_frame()

    plot_quad_3d(waypoints, control_loop)
    print("Closing.")
