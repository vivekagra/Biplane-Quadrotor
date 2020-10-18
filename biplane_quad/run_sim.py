from globals import flag, hover_t, fort_t, backt_t, cruise_t
from sim_3d import sim_3d
from controller import Controller
from time_traj_hover import time_traj_hover
""" File to run the simultor """
controlhandle = Controller
trajhandle = time_traj_hover


flag = 1
hover_t = 0
fort_t = 0
backt_t = 0
cruise_t = 0

t, state = sim_3d(trajhandle, controlhandle);