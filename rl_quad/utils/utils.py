"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from math import sin, cos, asin, atan2, sqrt

def RotToRPY_ZXY(R):
    phi = asin(R[1,2])
    theta = atan2(-R[0,2]/cos(phi),R[2,2]/cos(phi))
    psi = atan2(-R[1,0]/cos(phi),R[1,1]/cos(phi))
    return np.array([phi, theta, psi])

def RotToRPY_ZYX(R):
    """
    Get euler angles from rotation matrix using ZYX convention
    """
    theta = -asin(R[0,2])
    phi = atan2(R[1,2]/cos(theta), R[2,2]/cos(theta))
    psi = atan2(R[0,1]/cos(theta), R[0,0]/cos(theta))
    return np.array([phi, theta, psi])

def RPYToRot_ZXY(phi, theta, psi):
    """
    phi, theta, psi = roll, pitch , yaw
    The euler angle convention used is ZXY. This means: first a rotation of psi-degrees
    around Z axis, then rotation of phi-degress around X axis, and finally rotation of
    theta-degrees around Y axis
    """
    return np.array([[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta)],
                     [-cos(phi)*sin(psi), cos(phi)*cos(psi), sin(phi)],
                     [cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(theta)]])


def RPYToRot_ZYX(phi, theta, psi):
    """
    phi, theta, psi = roll, pitch , yaw
    The euler angle convention used is ZYX. This means: first a rotation of psi-degrees
    around Z axis, then rotation of theta-degrees around Y axis, and finally rotation of 
    phi-degress around X axis 
    """
    return np.array([[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)],
                     [-cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)],
                     [sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)]])


def RotToQuat(R):
    """
    ROTTOQUAT Converts a Rotation matrix into a Quaternion
    from the following website, deals with the case when tr<0
    http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
    takes in W_R_B rotation matrix
    """

    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        S = sqrt(tr+1.0) * 2 # S=4*qw
        qw = 0.25 * S
        qx = (R[2,1] - R[1,2]) / S
        qy = (R[0,2] - R[2,0]) / S
        qz = (R[1,0] - R[0,1]) / S
    elif (R[0,1] > R[1,1]) and (R[0,0] > R[2,2]):
        S = sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2 # S=4*qx
        qw = (R[2,1] - R[1,2]) / S
        qx = 0.25 * S
        qy = (R[0,1] + R[1,0]) / S
        qz = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2 # S=4*qy
        qw = (R[0,2] - R[2,0]) / S
        qx = (R[0,1] + R[1,0]) / S
        qy = 0.25 * S
        qz = (R[1,2] + R[2,1]) / S
    else:
        S = sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2 # S=4*qz
        qw = (R[1,0] - R[0,1]) / S
        qx = (R[0,2] + R[2,0]) / S
        qy = (R[1,2] + R[2,1]) / S
        qz = 0.25 * S

    q = np.sign(qw) * np.array([qw, qx, qy, qz])
    return q

def writeNpArrayToFile(data):
    with open('state.csv','a') as f:
        np.savetxt(f, data, newline=",", fmt='%.2f')
        f.write('\n')

def outputTraj(x,y,z):
    output = []
    output.append((x,y,z))
    with open('traj.out', 'w') as fp:
        fp.write('\n'.join('%s %s %s' % item for item in output))

def add_plots(ax,x,datas,lines,cols,labs,tit,xlab,ylab):

    for (data, line, colr, labl) in zip(datas, lines, cols, labs):
        ax.plot(x,data, linestyle = line, color = colr, label = labl)
    ax.set_title(tit)
    ax.set_xlabel(xlab)
    ax.set_ylabel(ylab)
    return ax

def saturate_scalar_minmax(value, max_value, min_value):
    """
    @ description saturation function for a scalar with definded maximum and minimum value
    See Q. Quan. Introduction to Multicopter Design (2017), Ch11.3, page 265 for reference
    """
    mean = (max_value + min_value)/2.0
    half_range = (max_value - min_value)/2.0
    return saturate_vector_dg(value-mean, half_range) + mean


# saturation function for vectors
def saturate_vector_dg(v, max_value):
    """
    @description saturation function for the magnitude of a vector with maximum magnitude 
    and guaranteed direction.
    See Q. Quan. Introduction to Multicopter Design (2017), Ch. 10.2 for reference
    """
    mag = np.linalg.norm(v)
    if( mag < max_value):
        return v
    else:
        return np.dot(v/mag,max_value)  # return vector in same direction but maximum possible magnitude