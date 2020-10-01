from globals import flag, hover_t, hover_z, hover_x, hover_y
from utils import desired_state

def time_traj_hover(t,s):
    if (t<1):
        z=t
    else:
        z=1
        
    
    #z=t;
    x=0
    y=0
    thetad=0
    phid=0
    psid=0
    thetadot_des = 0
    phidot_des=0
    psidot_des=0
    xdot=0
    ydot=0
    zdot=0
    xddot=0
    yddot=0
    zddot=0
    Tfwd=0
    Mfwd=0
    
    des_state = desired_state()
    des_state.pos = [[x], [y], [z]]
    des_state.vel = [[xdot], [ydot], [zdot]]
    des_state.acc = [[xddot], [yddot], [zddot]]
    des_state.rot = [[phid], [thetad], [psid]]
    des_state.omega = [[phidot_des], [thetadot_des], [psidot_des]]
    des_state.control = [[Tfwd], [Mfwd]]
    
    
    if (t>1 and len(s)>0):
        if (s.vel[0]<0.01 and s.vel[1]<0.001 and s.vel[2]<0.001 and 
            s.rot[0]<0.001 and s.rot[1]<0.01 and s.rot[2]<0.001 and 
            s.omega[0]<0.001 and s.omega[1]<0.001 and s.omega[2]<0.001):
        
            flag=2
            hover_t=t
            hover_z=s.pos[2]
            hover_x=s.pos[0]
            hover_y=s.pos[1]
        else:
            return