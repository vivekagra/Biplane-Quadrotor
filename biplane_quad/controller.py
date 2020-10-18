from numpy import identity as eye
import numpy as np
from numpy.linalg import norm as norm
from numpy import sin as sin
from numpy import cos as cos
from utils import eul2rotm
from aeroFunctions import momentEstimate, forceEstimate
import globals

class Controller:
    def __init__(self, t, state, des_state, BQ):
        self.flag = globals.flag
        self.t = t
        self.state = state
        self.des_state = des_state
        self.BQ = BQ
        self.kr = eye(3)
        self.kw = eye(3)
        self.kp = eye(3)
        self.kv = eye(3)
    
    def run(self):
        if(self.flag == 1):
            self.kp[0,0]=25     #hover
            self.kp[1,1]=1
            self.kp[2,2]=40
            self.kv[0,0]=15
            self.kv[1,1]=6
            self.kv[2,2]=0
            self.kr[0,0]=3
            self.kr[1,1]=45
            self.kr[2,2]=5
            self.kw[0,0]=5
            self.kw[1,1]=70
            self.kw[2,2]=1
            
        elif (self.flag==2):
            self.kp[0,0]=0     #10      forward tran, kpx nt impacting 
            self.kp[1,1]=0
            self.kp[2,2]=300     #50
            self.kv[0,0]=180     #15
            self.kv[1,1]=100
            self.kv[2,2]=120         #40
            self.kr[0,0]=0
            self.kr[1,1]=200
            self.kr[2,2]=0
            self.kw[0,0]=1
            self.kw[1,1]=200     #if after some seconds the code is not working then change kw and kr. Make inner looprun faster
            self.kw[2,2]=1
    
        elif (self.flag==3):
            self.kp[0,0]=0
            self.kp[1,1]=1      #cruise
            self.kp[2,2]=20
            self.kv[0,0]=70 #30
            self.kv[1,1]=1
            self.kv[2,2]=30
            self.kr[0,0]=1
            self.kr[1,1]=250 #50
            self.kr[2,2]=1
            self.kw[0,0]=1
            self.kw[1,1]=500 #70
            self.kw[2,2]=1   
    
        elif (self.flag==4):
            self.kp[0,0]=5
            self.kp[1,1]=1    #backtrans
            self.kp[2,2]=1
            self.kv[0,0]=5
            self.kv[1,1]=1
            self.kv[2,2]=30
            self.kr[0,0]=1
            self.kr[1,1]=70
            self.kr[2,2]=1
            self.kw[0,0]=1
            self.kw[1,1]=200
            self.kw[2,2]=1
    
        elif (self.flag==5):
            self.kp[0,0]=5
            self.kp[1,1]=1     #land
            self.kp[2,2]=1
            self.kv[0,0]=5
            self.kv[1,1]=1
            self.kv[2,2]=30
            self.kr[0,0]=1
            self.kr[1,1]=70 
            self.kr[2,2]=1
            self.kw[0,0]=1
            self.kw[1,1]=1030
            self.kw[2,2]=1
        
        ud = self.des_state.control
        
        omega_curr=np.array([[1, 0, -sin(self.state.rot[1])], 
                    [0, cos(self.state.rot[2]), cos(self.state.rot[1])*sin(self.state.rot[2])], 
                    [0, -sin(self.state.rot[2]), cos(self.state.rot[1])*cos(self.state.rot[2])]]) * np.array([[self.state.omega[0]], [self.state.omega[1]], [self.state.omega[2]]]) # check validity
                             
        omega_des= np.array([[1, 0, -sin(self.des_state.rot[1])], 
                            [0, cos(self.des_state.rot[2]), cos(self.des_state.rot[1])*sin(self.des_state.rot[2])], 
                            [0, -sin(self.des_state.rot[2]), cos(self.des_state.rot[1])*cos(self.des_state.rot[2])]]) * np.array([[self.des_state.omega[0]], [self.des_state.omega[1]], [self.des_state.omega[2]]]) # check validity

        Rb=np.array(eul2rotm(self.state.rot))
        R=np.array(eul2rotm(self.des_state.rot))
        
        #Fa=forceEstimate([0 state.rot(2) 0],[state.vel(1);0;state.vel(3)],omega_curr)
        
        [ Fa,Fa_w,alpha,beta ] = forceEstimate(np.array([self.state.rot[0], self.state.rot[1], self.state.rot[2]]),
                              np.array([self.state.vel[0], self.state.vel[1], self.state.vel[2]]), omega_curr);

        print("Fa", Fa)
        print("Rb", Rb)
        print(-((Rb*Fa)/(self.BQ.m)))
        print((self.BQ.m))
        

        if ud[0]!=0:
            BQ.g =0

        # acc_net=(((R*[ [0], [0], ud[0]]) / self.BQ.m) +   kp*[[0], [0], 
        #     [(self.des_state.pos[2] - self.state.pos[2])]] +  kv*[[(self.des_state.vel[0] - self.state.vel[0])], 
        #                      [(self.des_state.vel[1]-self.state.vel[1])], [(self.des_state.vel[2]-self.state.vel[2])]] + np.array([0, 0, self.BQ.g]) - ((Rb*Fa)/(self.BQ.m)))

        acc_net = [0,0,1]

        # calculation of current euler angles
        b3 =acc_net/norm(acc_net)
        c2 = np.transpose([-sin(self.des_state.rot[2]), cos(self.des_state.rot[2]), 0])
        b1 = np.cross(c2,b3)/norm(np.cross(c2,b3))
        b2 = np.cross(b3,b1)
        Rd = [b1, b2, b3]
        
        # thrust control input
                  
        F = self.BQ.m*acc_net[2]           
        
        R = eul2rotm(self.state.rot)
        erm = 0.5*((np.transpose(Rd)*R) - (np.transpose(R)*Rd))
        er = [[erm[2,1], [erm[0,2]], [erm[1,0]]]]
        ew = (omega_curr-((np.transpose(R)*Rd)*omega_des))
        
        tau_a=momentEstimate([self.state.rot[0], self.state.rot[1], self.state.rot[2]], 
                                             [[self.state.vel[0], self.state.vel[1], self.state.vel[2]]], omega_curr, Fa)
        
        # tau_a=momentEstimate([0; state.rot(2); 0],[state.vel(1);0;state.vel(3)],omega_curr, Fa);
        tau_a=np.array([0,0,0])
        M=[[0], [ud[1]], [0]] #- self.kr*er #- self.kw*ew + np.cross(omega_curr, self.BQ.J * omega_curr) - self.BQ.J*np.cross(ew,(np.transpose(R)*Rd)*omega_des)-tau_a    # moment input

        return [F, Fa, M, tau_a] 