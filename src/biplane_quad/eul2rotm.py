from numpy import sin
from numpy import cos

def eul2rotm(Euler):
    phi = Euler[0]
    theta = Euler[1]
    psi = Euler[2]
    
    R = [[cos(theta)*cos(psi), -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi),  sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi)],
            [cos(theta)*sin(psi),   cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi)],
            [-sin(theta),               sin(phi)*cos(theta),                                           cos(phi)*cos(theta)                             ]]
    return R