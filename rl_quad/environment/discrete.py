RED = '\033[91m'
BOLD = '\033[1m'
ENDC = '\033[0m'
LINE = "%s%s##############################################################################%s" % (RED, BOLD, ENDC)

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
import time as cpu_clock
import math

animation_frequency = 50
control_frequency = 200 # Hz for attitude control loop
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
time = [0.0]
sim_time = 2*np.pi
initial_pos = (0.5,0,0) 
initial_attitude = (0,0,0)

F_t = list()  # Thrust
M_t = list()  # Torque
t_s = list()  # simulation time
d_s = list()  # desired states
q_s = list()  # quadrotor states
w_i = list()  # rotor speeds

# Test: sim_time = 2pi, v = 1.5, helix

class QuadEnvDisc(gym.Env):
    
    def __init__(self):
        super(QuadEnvDisc, self).__init__()

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

        self.waypoints = trajGen3D.get_helix_waypoints(sim_time, 2)
        self.coeff_x, self.coeff_y, self.coeff_z = trajGen3D.get_MST_coefficients(self.waypoints)
        self.desired_pos = [0,0,10]
        self.max_distance = 30
        self.goal_distance = 0.3
        # self.action_space = spaces.Box(low=np.array([-100, -100, -100, -100]), high=np.array([100, 100, 100, 100]))
        self.action_space = spaces.Discrete(8)
        self.hover_start = 0
        self.hover_duration = 0
        self.hover_done = 10
        self.reach_goal_done = False
        self.reach_goal_once = False
        self.reach_goal = False
        self.out_of_bounds_done = False
        
        self.reward_range = (-np.inf, np.inf)
        self._seed()
        
        print("..created the environment successfully")

        cpu_clock.sleep(3)

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _state(self, action):
        return discretized_ranges, done

    def get_state():
        return self.quadcopter.state.astype(int)

    def step(self, action):
        
        print("Action: " + str(action))
        if action==0: self.quadcopter.rotor_speeds[0]+=1
        if action==1: self.quadcopter.rotor_speeds[1]+=1
        if action==2: self.quadcopter.rotor_speeds[2]+=1
        if action==3: self.quadcopter.rotor_speeds[3]+=1
        if action==4: self.quadcopter.rotor_speeds[0]-=1
        if action==5: self.quadcopter.rotor_speeds[1]-=1
        if action==6: self.quadcopter.rotor_speeds[2]-=1
        if action==7: self.quadcopter.rotor_speeds[3]-=1

        self.quadcopter.rotor_speeds = np.clip(self.quadcopter.rotor_speeds, -self.pwm_max, self.pwm_max)
        # self.quadcopter.rotor_speeds = [30, 30, 30, 30]
        # after each timestep, we will update the entire quadcopter-world
        F, M = self.quadcopter.pid_thrust_mapping()
        self.quadcopter.update(dt, F, M)
    
        observation = self.quadcopter.state.astype(int)
        distance = self.calculate_distance(self.desired_pos, self.quadcopter.position())
        self.reach_goal = distance < self.goal_distance
        self.out_of_bounds_done = distance > self.max_distance

        quad_attitude = self.quadcopter.attitude()
        attitude_cost = quad_attitude[0]*quad_attitude[0] + quad_attitude[1]*quad_attitude[1] + quad_attitude[2]*quad_attitude[2]
        x_cost = self.quadcopter.position()[0]
        y_cost = self.quadcopter.position()[1]

        print("Current position: {}, desired position: {}".format(self.quadcopter.position(), self.desired_pos))
        print("Attitude cost: {}, Distance cost: {}".format(attitude_cost, distance))

        w1=1
        w2=0.1
        w3=1
        w4=1
        
        if self.reach_goal_once and self.reach_goal:
            print("Near the goal, and trying to stabilise")
            self.hover_duration+=dt

        if self.reach_goal and not self.reach_goal_once:
            print("First time reached the goal")
            self.reach_goal_once = True
            self.hover_start = time[0]
            self.hover_duration = 0
        
        if not self.reach_goal and self.hover_duration>0:
            print("Hovered outside")
            self.hover_duration-=2*dt
            
        if self.hover_duration > self.hover_done:
            print("Stabilisation done")
            self.reach_goal_done = True
        
        if self.hover_duration <= 0:
            self.reach_goal_once = False
        
        if(self.reach_goal_done):
            self.reward = 1000
            done = self.reach_goal_done
        elif(self.out_of_bounds_done):
            self.reward = -1000
            done = self.out_of_bounds_done
        else:
            self.reward = 10 - distance*w1 - attitude_cost*w2 - x_cost*w3 - y_cost*w4
            done = False
        
        self.record_data(F, M, time, self.desired_pos, self.quadcopter)
        time[0] += dt
        
        print("Hovering for: {}".format(self.hover_duration))
        return observation, self.reward, done, {}

    def reset(self):
        self.quadcopter.state = [0,0,0,0,0,0,0,0,0,1,0,0,0]
        self.reward = 0
        time[0]=0
        self.waypoints = trajGen3D.get_helix_waypoints(sim_time, 2)
        self.coeff_x, self.coeff_y, self.coeff_z = trajGen3D.get_MST_coefficients(self.waypoints)
        self.desired_pos = [0,0,5]
        self.reach_goal_once=False
        self.reach_goal=False
        self.reach_goal_done=False
    
        

    def record_data(self, F, M, time, desired_pos, quad):
        F_t.append(F)
        M_t.append(M)
        t_s.append(time[0])
        d_s.append(desired_pos)
        # q_s.append([quad.state[0:3],quad.state[3:6],quad.attitude(),quad.state[10:13]])
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
        d_x = list(map(lambda a: a.pos[0], d_s))  # get desired x position
        x_e = list(map(lambda a,b: 10*(a-b),d_x,q_x))  # compute error

        fig0ax2 = utils.add_plots(fig0ax2,t_s,[q_x,d_x,x_e],["-","--","-"],["g","r","b"],["quad -x","des x","x error (x10)"],"X - axis position of quadrotor","t {s}","x {m}")
        fig0ax2.legend(loc='lower right', shadow=True, fontsize='small')

        # Y position
        q_y = list(map(lambda a: a[0][1], q_s))
        d_y = list(map(lambda a: a.pos[1], d_s))
        y_e = list(map(lambda a,b: 10*(a-b),d_y,q_y))

        fig0ax3 = utils.add_plots(fig0ax3,t_s,[q_y,d_y,y_e],["-","--","-"],["g","r","b"],["quad -y","des y","y error (x10)"],"Y - axis position of quadrotor","t {s}","y {m}")
        fig0ax3.legend(loc='lower right', shadow=True, fontsize='small')

        # Z position
        q_z = list(map(lambda a: a[0][2], q_s))
        d_z = list(map(lambda a: a.pos[2], d_s))
        z_e = list(map(lambda a,b: 10*(a-b),d_z,q_z))

        fig0ax4 = utils.add_plots(fig0ax4,t_s,[q_z,d_z,z_e],["-","--","-"],["g","r","b"],["quad z","des z","z error (x10)"],"Z - axis position of quadrotor","t {s}","z {m}")
        fig0ax4.legend(loc='lower right', shadow=True, fontsize='small')

        # Euler angles
        q_phi = list(map(lambda a: a[2][0]*180.0/np.pi, q_s))
        q_theta = list(map(lambda a: a[2][1]*180.0/np.pi, q_s))
        q_psi = list(map(lambda a: a[2][2]*180.0/np.pi, q_s))

        fig0ax5 = utils.add_plots(fig0ax5,t_s,[q_phi,q_theta,q_psi],["-","-","-"],["r","g","b"],["phi","theta","psi"],"Angular position of quadrotor",'t {s}','phi, theta, psi {degree}')
        fig0ax5.legend(loc='lower right', shadow=True, fontsize='small')

        #  X Linear velocity
        q_vx = list(map(lambda a: a[1][0], q_s))
        d_vx = list(map(lambda a: a.vel[0], d_s))   
        vx_e = list(map(lambda a,b: 10*(a-b),d_vx,q_vx))

        fig1ax0 = utils.add_plots(fig1ax0,t_s,[q_vx,d_vx,vx_e],["-","--","-"],["g","r","b"],["quad Vx","des Vx","Vx error (x10)"],"X axis linear Velocities of quadrotor",'t {s}','Vx {m/s}')
        fig1ax0.legend(loc='lower right', shadow=True, fontsize='small')   

        #  Y Linear velocity
        q_vy = list(map(lambda a: a[1][1], q_s))
        d_vy = list(map(lambda a: a.vel[1], d_s))
        vy_e = list(map(lambda a,b: 10*(a-b),d_vy,q_vy))

        fig1ax1 = utils.add_plots(fig1ax1,t_s,[q_vy,d_vy,vy_e],["-","--","-"],["g","r","b"],["quad Vy","des Vy","Vy error (x10)"],"Y axis linear Velocities of quadrotor",'t {s}','Vy {m/s}')
        fig1ax1.legend(loc='lower right', shadow=True, fontsize='small')  

        #  Z Linear velocity
        q_vz = list(map(lambda a: a[1][2], q_s))
        d_vz = list(map(lambda a: a.vel[2], d_s))
        vz_e = list(map(lambda a,b: 10*(a-b),d_vz,q_vz))

        fig1ax2 = utils.add_plots(fig1ax2,t_s,[q_vz,d_vz,vz_e],["-","--","-"],["g","r","b"],["quad Vz","des Vz","Vz error (x10)"],"Z axis linear Velocities of quadrotor",'t {s}','Vz {m/s}')
        fig1ax2.legend(loc='lower right', shadow=True, fontsize='small')  

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