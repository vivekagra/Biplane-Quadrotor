from biplane_quad.globals import backt_t, backt_z, max_time
from biplane_quad.utils import desired_state

def time_traj_land(t):
    # t1=t-backt_t
    x = 0
    z = (backt_z*(max_time-t))/(max_time-backt_t)
    y = 0
    thetad = 0
    phid = 0
    psid = 0
    thetadot_des = 0
    phidot_des = 0
    psidot_des = 0
    xdot = 0
    ydot = 0
    zdot = 0
    xddot = 0
    yddot = 0
    zddot = 0
    
    #Tfwd=-(117.72)*(t-backt_t)/(max_time-backt_t)
    #Tfwd=((2*117.72)*(t-backt_t)/(-max_time+backt_t))+117.72
    
    Tfwd=0
    Mfwd=0
    
    if (z<0):
        z=0
    
    des_state = desired_state()
    des_state.pos = [[x], [y], [z]]
    des_state.vel = [[xdot], [ydot], [zdot]]
    des_state.acc = [[xddot], [yddot], [zddot]]
    des_state.rot=[[phid], [thetad], [psid]]
    des_state.omega=[[phidot_des], [thetadot_des], [psidot_des]]
    des_state.control=[[Tfwd], [Mfwd]]
    
    return des_state