RED = '\033[91m'
BOLD = '\033[1m'
ENDC = '\033[0m'
LINE = "%s%s##############################################################################%s" % (RED, BOLD, ENDC)

import gym
from rl_quad.utils.quadPlot import plot_quad_3d
from rl_quad.conventional_control import lqr_controller as lqr
from rl_quad.conventional_control import pid_controller as pid
from rl_quad.conventional_control import df_controller as df1
from rl_quad.conventional_control import df_controller_rotor_drag as df2
from rl_quad.conventional_control import df_controller_rotor_drag_VT as df3
from sklearn.preprocessing import normalize

import rl_quad.utils.trajGen as trajGen
import rl_quad.utils.trajGen3D as trajGen3D
import rl_quad.utils.utils as utils
import rl_quad.environment.model.params as params
from rl_quad.environment.model.quadcopter import Quadcopter

from gym import utils as gym_utils, spaces
import numpy as np
import matplotlib.pyplot as plt
from gym.utils import seeding
from sklearn.preprocessing import StandardScaler, scale, normalize
import time as cpu_clock
from rl_quad.utils.quaternion import Quaternion
from rl_quad.utils.utils import RPYToRot_ZYX as RPYToRot
from rl_quad.utils.utils import RotToRPY_ZYX as RotToRPY
from rl_quad.utils.utils import RotToQuat

import math

animation_frequency = 50
control_frequency = 200 # Hz for attitude control loop
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
time = [0.0]
sim_time = 2*np.pi
initial_pos = (0,0,0) 
initial_attitude = (0,0,0)
inf = 10000

F_t = list()  # Thrust
M_t = list()  # Torque
t_s = list()  # simulation time
d_s = list()  # desired states
q_s = list()  # quadrotor states
w_i = list()  # rotor speeds

# Test: sim_time = 2pi, v = 1.5, helix
class QuadEnvCont(gym.Env):
    
    def __init__(self):
        super(QuadEnvCont, self).__init__()
        print("..initialising program..")
        self.quadcopter = Quadcopter(initial_pos, initial_attitude)  
        self.msg = [0, 0, 0, 0]
        self.pwm_max = 60
        
        print("Weight of quadcopter: {} N".format(params.mass * params.g))
        print("Maximum thrust from one rotor: {}, total max thrust: {}".format(self.pwm_max * self.pwm_max * 100 * 100 * params.kf, self.pwm_max * self.pwm_max * 100 * 100 * params.kf * params.number_of_rotors))

        F_t = list()  # Thrust
        M_t = list()  # Torque
        t_s = list()  # simulation time
        d_s = list()  # desired states
        q_s = list()  # quadrotor states
        w_i = list()  # rotor speeds

        self.euler_state=np.zeros(12)   
        self.max_distance = np.inf
        self.goal_distance = 0.3
        self.action_space = spaces.Box(low=-1*np.ones(4), high=np.ones(4))
        self.observation_space = spaces.Box(low=np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.pi, -np.pi, -np.pi, -np.inf, -np.inf, -np.inf]), 
                                            high=np.array([np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.pi,  np.pi,  np.pi,  np.inf,  np.inf,  np.inf]))
        self.max_distance = 5000
        self.out_of_bounds_done = False
        self.reward_range = (-np.inf, np.inf)
        self._seed()
        print("..created the environment successfully")
        # exit()
        cpu_clock.sleep(0.5)

    def state_to_euler_state(self, state):
        quaternion = Quaternion(self.quadcopter.state[6:10])
        rot=quaternion.as_rotation_matrix()
        attitude=RotToRPY(rot)
        euler_state = np.array([state[0],state[1],state[2],state[3],state[4],state[5],attitude[0], attitude[1], attitude[2], state[10], state[11], state[12]])
        return euler_state

    def euler_state_to_quat_state(self, euler_state):
        attitude=euler_state[6:9]
        quat = RotToQuat(RPYToRot(attitude[0], attitude[1], attitude[2]))
        state = np.array([euler_state[0], euler_state[1],euler_state[2],euler_state[3], euler_state[4], euler_state[5], quat[0], quat[1], quat[2], quat[3], euler_state[9], euler_state[10], euler_state[11]])
        return state

    def set_goal(self, pos):
        self.quadcopter.state = [pos[0], pos[1], pos[2], 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _state(self, action):
        return discretized_ranges, done

    def get_state(self):
        return self.state_to_euler_state(np.asarray(self.quadcopter.state))

    def test_step(self, action):
        self.quadcopter.rotor_speeds = np.array(list(map(lambda x: x*60+60, action)))
        F, M = self.quadcopter.pid_thrust_mapping()
        self.quadcopter.update(dt=dt, F=F, M=M)
        self.record_data(F, M, time, self.quadcopter.state[:3], self.quadcopter)
        time[0] += dt
        distance = self.calculate_distance(self.desired_state[:3], self.quadcopter.position())
        observation = self.quadcopter.state
        done = distance < self.goal_distance
        return observation, done

    def step(self, action):        
        self.quadcopter.rotor_speeds = np.array(list(map(lambda x: x*60+60, action)))
        print(self.quadcopter.rotor_speeds)
        F, M = self.quadcopter.pid_thrust_mapping()
        self.quadcopter.update(dt, F, M)
        distance = self.calculate_distance(self.desired_state[:3], self.quadcopter.position())
        self.reach_goal = distance < self.goal_distance
        self.out_of_bounds_done = distance > self.max_distance
        euler_state = self.state_to_euler_state(self.quadcopter.state)
        pos_cost = np.linalg.norm(self.quadcopter.position() - self.desired_state[:3], ord=2)
        att_cost = np.linalg.norm(euler_state[3:6] - self.euler_state[3:6], ord=2)
        vel_cost = np.linalg.norm(self.quadcopter.velocity() - self.desired_state[7:10], ord=2)
        omega_cost = np.linalg.norm(self.quadcopter.omega() - self.desired_state[10:13], ord=2)
        self.reward = -(0.004*pos_cost +  0.0002*att_cost + 0.003*omega_cost + 0.0005*vel_cost)*100
        completed=False
        if (self.out_of_bounds_done):
            completed=True
        if (np.linalg.norm(self.quadcopter.omega(), ord=2)>300):
            completed=True
        if (np.linalg.norm(self.quadcopter.velocity(), ord=2)>300):
            completed=True
        self.record_data(F, M, time, self.desired_state[:3], self.quadcopter)
        print("Pos: {}, Att: {}, Vel: {}, Omega: {}, Total: {}".format(pos_cost, att_cost, omega_cost, vel_cost, self.reward))
        time[0] += dt
        observation = self.state_to_euler_state(self.quadcopter.state)
        print(distance, self.quadcopter.attitude(), self.quadcopter.velocity(), self.quadcopter.omega())

        return observation, self.reward, completed, {}  

    def reset(self):
        self.quadcopter.state=self.euler_state_to_quat_state(self.observation_space.sample())
        self.quadcopter.state[0], self.quadcopter.state[1], self.quadcopter.state[2] = np.random.rand(3)*10
        self.reward = 0
        time[0]=0
        self.desired_state = np.array([0, 0, 10, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
        self.euler_state=self.state_to_euler_state(self.quadcopter.state)
        return self.euler_state
        
    def record_data(self, F, M, time, desired_pos, quad):
        F_t.append(F)
        M_t.append(M)
        t_s.append(time[0])
        d_s.append(desired_pos)
        q_s.append([quad.state[0:3],quad.state[3:6],quad.attitude(),quad.state[10:13]])
        w_i.append(np.dot(params.invB,np.concatenate((np.array([F]),M),axis = 0)))   # rotor speeds

    def _killall(self, process_name):
        pids = subprocess.check_output(["pidof",process_name]).split()
        for pid in pids:
            os.system("kill -9 "+str(pid))

    def calculate_distance(self,desired, current):
        return math.sqrt((desired[0] - current[0])*(desired[0] - current[0]) + (desired[1] - current[1])*(desired[1] - current[1]) + (desired[2] - current[2])*(desired[2] - current[2]))

    def plot(self, name):
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
        u2 = list(map(lambda a: a[0],M_t)) # extract ux for all points in time
        u3 = list(map(lambda a: a[1],M_t))
        u4 = list(map(lambda a: a[2],M_t))

        fig0ax1 = utils.add_plots(fig0ax1,t_s,[u2,u3,u4],["-","-","-"],["r","g","b"],["u2","u3","u4"],"Components of torque vector M over time","t {s}","{N*m}")
        fig0ax1.legend(loc='lower right', shadow=True, fontsize='small')

        # X position
        q_x = list(map(lambda a: a[0][0], q_s))   # get quad x position
        d_x = list(map(lambda a: a[0], d_s))  # get desired x position
        x_e = list(map(lambda a,b: 10*(a-b),d_x,q_x))  # compute error

        fig0ax2 = utils.add_plots(fig0ax2,t_s,[q_x,d_x,x_e],["-","--","-"],["g","r","b"],["quad -x","des x","x error (x10)"],"X - axis position of quadrotor","t {s}","x {m}")
        fig0ax2.legend(loc='lower right', shadow=True, fontsize='small')

        # Y position
        q_y = list(map(lambda a: a[0][1], q_s))
        d_y = list(map(lambda a: a[1], d_s))
        y_e = list(map(lambda a,b: 10*(a-b),d_y,q_y))

        fig0ax3 = utils.add_plots(fig0ax3,t_s,[q_y,d_y,y_e],["-","--","-"],["g","r","b"],["quad -y","des y","y error (x10)"],"Y - axis position of quadrotor","t {s}","y {m}")
        fig0ax3.legend(loc='lower right', shadow=True, fontsize='small')

        # Z position
        q_z = list(map(lambda a: a[0][2], q_s))
        d_z = list(map(lambda a: a[2], d_s))
        z_e = list(map(lambda a,b: 10*(a-b),d_z,q_z))

        fig0ax4 = utils.add_plots(fig0ax4,t_s,[q_z,d_z,z_e],["-","--","-"],["g","r","b"],["quad z","des z","z error (x10)"],"Z - axis position of quadrotor","t {s}","z {m}")
        fig0ax4.legend(loc='lower right', shadow=True, fontsize='small')

        # Euler angles
        q_phi = list(map(lambda a: a[2][0]*180.0/np.pi, q_s))
        q_theta = list(map(lambda a: a[2][1]*180.0/np.pi, q_s))
        q_psi = list(map(lambda a: a[2][2]*180.0/np.pi, q_s))

        fig0ax5 = utils.add_plots(fig0ax5,t_s,[q_phi,q_theta,q_psi],["-","-","-"],["r","g","b"],["phi","theta","psi"],"Angular position of quadrotor",'t {s}','phi, theta, psi {degree}')
        fig0ax5.legend(loc='lower right', shadow=True, fontsize='small')

        # #  X Linear velocity
        # q_vx = list(map(lambda a: a[1][0], q_s))
        # d_vx = list(map(lambda a: a.vel[0], d_s))   
        # vx_e = list(map(lambda a,b: 10*(a-b),d_vx,q_vx))

        # fig1ax0 = utils.add_plots(fig1ax0,t_s,[q_vx,d_vx,vx_e],["-","--","-"],["g","r","b"],["quad Vx","des Vx","Vx error (x10)"],"X axis linear Velocities of quadrotor",'t {s}','Vx {m/s}')
        # fig1ax0.legend(loc='lower right', shadow=True, fontsize='small')   

        # #  Y Linear velocity
        # q_vy = list(map(lambda a: a[1][1], q_s))
        # d_vy = list(map(lambda a: a.vel[1], d_s))
        # vy_e = list(map(lambda a,b: 10*(a-b),d_vy,q_vy))

        # fig1ax1 = utils.add_plots(fig1ax1,t_s,[q_vy,d_vy,vy_e],["-","--","-"],["g","r","b"],["quad Vy","des Vy","Vy error (x10)"],"Y axis linear Velocities of quadrotor",'t {s}','Vy {m/s}')
        # fig1ax1.legend(loc='lower right', shadow=True, fontsize='small')  

        # #  Z Linear velocity
        # q_vz = list(map(lambda a: a[1][2], q_s))
        # d_vz = list(map(lambda a: a.vel[2], d_s))
        # vz_e = list(map(lambda a,b: 10*(a-b),d_vz,q_vz))

        # fig1ax2 = utils.add_plots(fig1ax2,t_s,[q_vz,d_vz,vz_e],["-","--","-"],["g","r","b"],["quad Vz","des Vz","Vz error (x10)"],"Z axis linear Velocities of quadrotor",'t {s}','Vz {m/s}')
        # fig1ax2.legend(loc='lower right', shadow=True, fontsize='small')  

        # Angular velocities
        q_wx = list(map(lambda a: a[3][0]*180.0/np.pi, q_s))
        q_wy = list(map(lambda a: a[3][1]*180.0/np.pi, q_s))
        q_wz = list(map(lambda a: a[3][2]*180.0/np.pi, q_s))

        fig1ax3 = utils.add_plots(fig1ax3,t_s,[q_wx,q_wy,q_wz],["-","-","-"],["r","g","b"],["wx","wy","wz"],"Angular velocities of quadrotor",'t {s}','wx, wy, wz {degree/s}')
        fig1ax3.legend(loc='lower right', shadow=True, fontsize='small')

        # # rotor speeds
        # w_0 = list(map(lambda a: np.sqrt(a[0][0]) if a[0][0] > 0 else -np.sqrt(-a[0][0]),w_i))
        # w_1 = list(map(lambda a: np.sqrt(a[1][0]) if a[1][0] > 0 else -np.sqrt(-a[1][0]),w_i))
        # w_2 = list(map(lambda a: np.sqrt(a[2][0]) if a[2][0] > 0 else -np.sqrt(-a[2][0]),w_i))
        # w_3 = list(map(lambda a: np.sqrt(a[3][0]) if a[3][0] > 0 else -np.sqrt(-a[3][0]),w_i))

        # fig1ax4 = utils.add_plots(fig1ax4, t_s, [w_0,w_1,w_2,w_3],["-","-","-","-"],["r","g","b","c"],["w0","w1","w2","w3"],"Rotor Speeds",'t {s}','{rpm}')
        # fig1ax4.legend(loc='lower right', shadow=True, fontsize='small')
        # save
        fig0.savefig("t_"+name, dpi = 300) #translation variables
        fig1.savefig("r_"+name, dpi = 300) #rotation variables
        print("Saved t_{} and r_{}.".format(name,name))