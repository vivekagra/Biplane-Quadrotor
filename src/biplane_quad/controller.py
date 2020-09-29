from numpy import identity as eye
from numpy import sin as sin
from numpy import cos as cos

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
            self.kr(2,2]=5
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
        
        omega_curr=[[1, 0, -sin(self.state.rot(2))], 
                             [0, cos(self.state.rot(3)), cos(state.rot(2))*sin(state.rot(3))], 
                             [0, -sin(state.rot(3)), cos(state.rot(2))*cos(state.rot(3))]] * 
                             [[state.omega(1)], [state.omega(2)], [state.omega(3)]] # check validity
                             
        omega_des=[[1, 0, -sin(self.des_state.rot(2))], 
                            [0, cos(self.des_state.rot(3)), cos(self.des_state.rot(2))*sin(self.des_state.rot(3))], 
                            [0, -sin(des_state.rot(3)), cos(des_state.rot(2))*cos(des_state.rot(3))]] * 
                            [[des_state.omega(1)], [des_state.omega(2)], [des_state.omega(3)]] # check validity

        