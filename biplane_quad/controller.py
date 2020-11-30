from numpy import identity as eye
import numpy as np
from numpy.linalg import norm as norm
from numpy import sin as sin
from numpy import cos as cos
from utils import eul2rotm
from aeroFunctions import momentEstimate, forceEstimate
from globals import flag,Rd

def controller(t, state, des_state, BQ):
    kr = eye(3)
    kw = eye(3)
    kp = eye(3)
    kv = eye(3)
         
    if(flag == 1):
        kp[0,0]=25     #hover
        kp[1,1]=1
        kp[2,2]=40
        kv[0,0]=15
        kv[1,1]=6
        kv[2,2]=0
        kr[0,0]=3
        kr[1,1]=45
        kr[2,2]=5
        kw[0,0]=5
        kw[1,1]=70
        kw[2,2]=1
            
    elif (flag==2):
        kp[0,0]=0     #10      forward tran, kpx nt impacting 
        kp[1,1]=0
        kp[2,2]=300     #50
        kv[0,0]=180     #15
        kv[1,1]=100
        kv[2,2]=120         #40
        kr[0,0]=0
        kr[1,1]=200
        kr[2,2]=0
        kw[0,0]=1
        kw[1,1]=200     #if after some seconds the code is not working then change kw and kr. Make inner looprun faster
        kw[2,2]=1

    elif (flag==3):
        kp[0,0]=0
        kp[1,1]=1      #cruise
        kp[2,2]=20
        kv[0,0]=70 #30
        kv[1,1]=1
        kv[2,2]=30
        kr[0,0]=1
        kr[1,1]=250 #50
        kr[2,2]=1
        kw[0,0]=1
        kw[1,1]=500 #70
        kw[2,2]=1   

    elif (flag==4):
        kp[0,0]=5
        kp[1,1]=1    #backtrans
        kp[2,2]=1
        kv[0,0]=5
        kv[1,1]=1
        kv[2,2]=30
        kr[0,0]=1
        kr[1,1]=70
        kr[2,2]=1
        kw[0,0]=1
        kw[1,1]=200
        kw[2,2]=1

    elif (flag==5):
        kp[0,0]=5
        kp[1,1]=1     #land
        kp[2,2]=1
        kv[0,0]=5
        kv[1,1]=1
        kv[2,2]=30
        kr[0,0]=1
        kr[1,1]=70 
        kr[2,2]=1
        kw[0,0]=1
        kw[1,1]=1030
        kw[2,2]=1
    
    ud = des_state.control
    
    omega_curr=np.array([[1, 0, -sin(state.rot[1])], 
                [0, cos(state.rot[2]), cos(state.rot[1])*sin(state.rot[2])], 
                [0, -sin(state.rot[2]), cos(state.rot[1])*cos(state.rot[2])]]) * np.array([[state.omega[0]], [state.omega[1]], [state.omega[2]]]) # check validity
                         
    omega_des= np.array([[1, 0, -sin(des_state.rot[1])], 
                        [0, cos(des_state.rot[2]), cos(des_state.rot[1])*sin(des_state.rot[2])], 
                        [0, -sin(des_state.rot[2]), cos(des_state.rot[1])*cos(des_state.rot[2])]]) * np.array([[des_state.omega[0]], [des_state.omega[1]], [des_state.omega[2]]]) # check validity

    Rb=np.array(eul2rotm(state.rot))
    R=np.array(eul2rotm(des_state.rot))
    
    #Fa=forceEstimate([0 state.rot(2) 0],[state.vel(1);0;state.vel(3)],omega_curr)
    # [ Fa,Fa_w,alpha,beta ] 
    Fa,Fa_w,alpha,beta = forceEstimate(np.array([state.rot[0], state.rot[1], state.rot[2]]),
                          np.array([state.vel[0], state.vel[1], state.vel[2]]), omega_curr);

    #print("Fa", Fa)
    #print("Rb", Rb)
    #print((BQ.m))
    #print(-(Rb*Fa)/BQ.m)
    

    if ud[0]!=0:
        BQ.g =0

    # acc_net=(((R*[ [0], [0], ud[0]]) / BQ.m) +   kp*[[0], [0], 
    #     [(des_state.pos[2] - state.pos[2])]] +  kv*[[(des_state.vel[0] - state.vel[0])], 
    #                      [(des_state.vel[1]-state.vel[1])], [(des_state.vel[2]-state.vel[2])]] + np.array([0, 0, BQ.g]) - ((Rb*Fa)/(BQ.m)))

    acc_net = [0,0,1]

    # calculation of current euler angles
    b3 =acc_net/norm(acc_net)
    c2 = np.transpose([-sin(des_state.rot[2]), cos(des_state.rot[2]), 0])
    b1 = np.cross(c2,b3)/norm(np.cross(c2,b3))
    b2 = np.cross(b3,b1)
    Rd = np.array([b1, b2, b3])
    
    # thrust control input
              
    F = BQ.m*acc_net[2]           
    
    R = eul2rotm(state.rot)
    erm = 0.5*(np.transpose(Rd).dot(R) - np.transpose(R).dot(Rd))
    er = np.array([erm[2,1], erm[0,2], erm[1,0]])
    ew = omega_curr-(np.transpose(R).dot(Rd)).dot(omega_des)
    
    tau_a=momentEstimate(np.array([state.rot[0], state.rot[1], state.rot[2]]), 
                                         np.array([state.vel[0], state.vel[1], state.vel[2]]), omega_curr, Fa)
    
    # tau_a=momentEstimate([0; state.rot(2); 0],[state.vel(1);0;state.vel(3)],omega_curr, Fa);
    tau_a=np.array([0,0,0])
    M=np.array([0, ud[1], 0]) #- kr*er #- kw*ew + np.cross(omega_curr, BQ.J * omega_curr) - BQ.J*np.cross(ew,(np.transpose(R)*Rd)*omega_des)-tau_a    # moment input

    return [F, Fa, M, tau_a] 