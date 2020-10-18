import numpy as np
from numpy import sin
from numpy import cos

def debug(variable):
    print variable, '=', repr(eval(variable))

""" traj_=[
            0.00000000000000, 2.38979695502618, 4.77382734355769, 7.14528763663408,
            9.48490559304162, 11.7624254933948, 13.9512841836096, 16.0345450467868,
            18.0067099114244, 19.8700348284321, 21.6287988676316, 23.2857311047559,
            24.8402723340366, 26.2877458941240, 27.6193335556213, 28.8228663097017, 
            29.8847503912196, 30.7933647838816, 31.5441847624783, 32.1418301282528,
            32.5984934225259, 32.9319963157787, 33.1636946944331, 33.3161073635660,
            33.4096158103627, 33.4613806883770, 33.4856560949199, 33.4943969621540,
            33.4965072850043, 33.4968046882397,33.4968470135881
            ]
"""

""" traj_dot_=[
                20.0000000000000, 19.9724889788625, 19.9054338738792, 19.7416622069201,
                19.3548827835231, 18.7119196613496, 17.8832905898648, 16.9620656312847,
                16.0357984279122, 15.1437658811142, 14.2843353051331, 13.4356129152420,
                12.5655528323586, 11.6384523427997, 10.6218661694046, 9.49406078577192,
                8.25382692926901, 6.94033271619121, 5.62765043956652, 4.38811966456048,
                3.27694328802304, 2.33256649079139, 1.57575681807551, 1.00240998170344,
                0.585901796834056, 0.299863510269067, 0.123040818227709,0.0357850765749011,
                0.00610735459558673, 0.000856677804326638, 0]
"""

def eul2rotm(eulerAngles):
    phi = eulerAngles[0]
    theta = eulerAngles[1]
    psi = eulerAngles[2]
    
    rotMatrix=[[cos(theta)*cos(psi),-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)],
                     [cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi)],
                     [-sin(theta),             sin(phi)*cos(theta),                                     cos(phi)*cos(theta)]]
        
    return rotMatrix


def traj_gen(traj, traj_dot, flag):    
    n = 4
    N = 31
    Tf = 0
    
    if (flag==4 or flag == 5):
        Tf=3.58
    
    elif (flag ==2 or flag == 3):
        Tf = 3.1
            
    f = np.zeros((30*n,1))

    for i in range(N-1):
        t0 = (i)*Tf/(N-1)
        tf = (i+1)*Tf/(N-1);
        M  = [[1, t0, t0**2, t0**3],
                 [1, tf,  tf**2,  tf**3],
                 [0, 1,  2*t0,   3*(t0**2)],
                 [0, 1,  2*tf,    3*tf**2]]
        M = np.array(M)
        
        b = [[traj[i]],
                [traj[i+1]],
                [traj_dot[i]],
                [traj_dot[i+1]]]
        b  = np.array(b)
        
        A = np.linalg.lstsq(M,b,rcond = None)[0] 
        
        f[n*i : n*(i+1)] = A
    return f


class Qd:
    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.rot = np.zeros(3)
        self.omega = np.zeros(3)

def stateToQd(x):
	qd = Qd()
	qd.pos = x[0:3]
	qd.vel = x[3:6]
	qd.rot = x[6:9]
	qd.omega = x[9:12]
	return qd

class desired_state:
    def __init__(self):
        self.pos = 0
        self.vel = 0
        self.acc = []
        self.rot = []
        self.omega = []
        self.control = []


