import globals 
from sim_3d import sim_3d
from controller import controller
from time_traj_hover import time_traj_hover

""" File to run the simultor """
controlhandle = controller
trajhandle = time_traj_hover

#globals.init()

flag = 1
hover_t = 0
fort_t = 0
backt_t = 0
cruise_t = 0

t, state = sim_3d(trajhandle, controlhandle)