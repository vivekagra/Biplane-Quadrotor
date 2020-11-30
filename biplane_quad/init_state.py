""" Take Inital position, Initial Velocity, Initial Rotation, and Inital omega and merge them in a vector to return Initial State """

import numpy as np

def init_state(start_pos,start_vel,start_rot,start_omega):
    s = np.zeros(12)
    print('Starting Position',start_pos)
    s[0]  = start_pos[0]               #x
    s[1]  = start_pos[1]               #y
    s[2]  = start_pos[2]               #z
    s[3]  = start_vel[0]               #xdot
    s[4]  = start_vel[1]               #ydot
    s[5]  = start_vel[2]               #zdot
    s[6] = start_rot[0]                #phi
    s[7] = start_rot[1]                #theta
    s[8] = start_rot[2]                #psi
    s[9] = start_omega[0]        #phidot
    s[10] = start_omega[1]        #thetadot
    s[11] = start_omega[2]        #psidot
    
    return s
    