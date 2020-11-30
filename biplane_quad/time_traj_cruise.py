import numpy as np
from globals import flag, hover_t, hover_z, hover_x, hover_y
from utils import desired_state

def time_traj_cruise(t,s):
    t1=t-fort_t
    x=20*t1
    z=0
    y=0
    thetad=1.54
    phid=0
    psid=0
    thetadot_des = 0
    phidot_des=0
    psidot_des=0
    xdot=20
    ydot=0
    zdot=0
    xddot=0
    yddot=0
    zddot=0
    Tfwd=0
    Mfwd=0
    
    des_state = desired_state()
    des_state.pos = np.array([x, y, z])
    des_state.vel = np.array([xdot, ydot, zdot])
    des_state.acc = np.array([xddot, yddot, zddot])
    des_state.rot = np.array([phid, thetad, psid])
    des_state.omega = np.array([phidot_des, thetadot_des, psidot_des])
    des_state.control = np.array([Tfwd, Mfwd])
    if (t1>0.5 and len(s)>0):
        if (s.vel(1)>19.5):
            flag=4
            cruise_t=t
            cruise_z=s.pos[2]
            cruise_x=s.pos[0]
            cruise_y=s.pos[1]
        else:
            return
    return        