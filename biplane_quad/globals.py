""" This file stores all the global variables"""

from numpy import pi

global flag
flag = 0

global backt_t
global backt_z
backt_t = 0
backt_z = 0

global cruise_t
global cruise_x
global cruise_y
global cruise_z
cruise_t = 0
cruise_x = 0
cruise_y = 0
cruise_z = 0

global fort_t
global fort_x
global fort_y
global fort_z
fort_t = 0
fort_x = 0
fort_y = 0
fort_z = 0

global hover_t
global hover_x
global hover_y
global hover_z
hover_t = 0
hover_x = 0
hover_y = 0
hover_z = 0

global max_time
max_time = 0


class systemParameters:
	def __init__(self):
		self.m = 12#%kg
		self.g  = 9.81
		self.rho = 1.225
		self.J =[[1.86,0,0],
		    	    [0,2.031,0],
		    	    [0,0,3.617]]
		self.L = 0.5
		self.x_ac = [0.083800]
		self.x_cg = [0.15700]
		self.c_root = 0.39
		self.c_tip = 0.176
		self.t = self.c_tip/self.c_root
		self.c = self.c_root*(2/3)*((1+self.t+self.t**2)/(1+self.t))
		self.b = 2.29/2
		self.S = 0.754/2
		self.wing_n=2
		
		#%%% motor parameters
		self.kc = 3*10**(-6)
		self.ka = 1.14*10**(-7)
		self.kc = (self.m*self.g/4)*((60/(pi))**2)
		self.ka = self.kc/10
		self.CD_0 = 0.009
		self.CL0 = 0.4918
		self.CL_alpha = 4.695
		self.CLq = 0
		self.Cm_0 = -0.0156
		self.Cm_alpha = 0.995
		self.Cm_q = -0.51
		self.CY_beta = -0.951
		self.CY_p = 0
		self.CY_r = 0.008
		self.Cl_beta = 0
		self.Cl_p = -0.43
		self.Cl_r = 0.29
		self.Cn_beta = 0.0812
		self.Cn_p = -0.4044
		self.Cn_r = -0.05

		self.k = 1/(pi*6.9*0.8)
		self.M = 50
		self.alpha0 = 20*(pi/180)

