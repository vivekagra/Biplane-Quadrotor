import numpy as np
import globals
class controller:
    def __init__(self, t, state, des_state, BQ):
        self.flag = globals.flag
        self.t = t
        self.state = state
        self.des_state = des_state
        self.BQ = BQ
        self.kr = np.identity(3)
        self.kw = np.identity(3)
        self.kp = np.identity(3)
        self.kv = np.identity(3)
    def control(self):
        if(self.flag == 1):
            self.kp(1,1)=25     #hover
            self.kp(2,2)=1
            self.kp(3,3)=40
            self.kv(1,1)=15
            self.kv(2,2)=6
            self.kv(3,3)=0
            self.kr(1,1)=3
            self.kr(2,2)=45
            self.kr(3,3)=5
            self.kw(1,1)=5
            self.kw(2,2)=70
            self.kw(3,3)=1
            
        elif (self.flag==2):
            self.kp(1,1)=0     #10      forward tran, kpx nt impacting 
            self.kp(2,2)=0
            self.kp(3,3)=300     #50
            self.kv(1,1)=180     #15
            self.kv(2,2)=100
            self.kv(3,3)=120         #40
            self.kr(1,1)=0
            self.kr(2,2)=200
            self.kr(3,3)=0
            self.kw(1,1)=1
            self.kw(2,2)=200     #if after some seconds the code is not working then change kw and kr. Make inner looprun faster
            self.kw(3,3)=1
    
        elif (self.flag==3):
            self.kp(1,1)=0
            self.kp(2,2)=1      #cruise
            self.kp(3,3)=20
            self.kv(1,1)=70 #30
            self.kv(2,2)=1
            self.kv(3,3)=30
            self.kr(1,1)=1
            self.kr(2,2)=250 #50
            self.kr(3,3)=1
            self.kw(1,1)=1
            self.kw(2,2)=500 #70
            self.kw(3,3)=1   
    
        elif (self.flag==4):
            self.kp(1,1)=5
            self.kp(2,2)=1    #backtrans
            self.kp(3,3)=1
            self.kv(1,1)=5
            self.kv(2,2)=1
            self.kv(3,3)=30
            self.kr(1,1)=1
            self.kr(2,2)=70
            self.kr(3,3)=1
            self.kw(1,1)=1
            self.kw(2,2)=200
            self.kw(3,3)=1
    
        elif (self.flag==5):
            self.kp(1,1)=5
            self.kp(2,2)=1     #land
            self.kp(3,3)=1
            self.kv(1,1)=5
            self.kv(2,2)=1
            self.kv(3,3)=30
            self.kr(1,1)=1
            self.kr(2,2)=70
            self.kr(3,3)=1
            self.kw(1,1)=1
            self.kw(2,2)=1030
            self.kw(3,3)=1
        