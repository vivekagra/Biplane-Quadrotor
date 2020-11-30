import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from init_state import init_state
from quadEOM import quadEOM


from globals import flag, hover_t, hover_z, hover_x, hover_y
from globals import t, s
from globals import systemParameters
from globals import fort_t, fort_x, fort_y, fort_z, backt_t, cruise_z, cruise_x, cruise_y, cruise_t, backt_x, backt_y, backt_z, max_time

from time_traj_land import time_traj_land
from time_traj_backtrans import time_traj_backtrans
from time_traj_cruise import time_traj_cruise
from time_traj_fortrans import time_traj_fortrans
from time_traj_hover import time_traj_hover

from utils import stateToQd

end=-1

def sim_3d(trajhandle, controlhandle):
    max_time = 1; 

    #controlhandle =Controller()
    
    # parameters for simulation
    BQ = systemParameters()
    # *********************** INITIAL CONDITIONS ***********************
    print('Setting initial conditions...')

    tstep    = 0.01 # this determines the time step at which the solution is given %0.02
    cstep    = 0.01 # image capture time interval 0.06
    max_iter = max_time/cstep # max iteration
    #max_iter = 1
    nstep    = int(cstep/tstep)
    time     = 0 # current time
    # Get start position
    # des_start = trajhandle(0, []);
    # x0    = init_state(des_start.pos,des_start.vel,des_start.rot,des_start.omega)

    store_shape = int(max_iter*nstep)
    xtraj = np.zeros((store_shape, 12))
    ttraj = np.zeros(store_shape)
    des_x = np.zeros(store_shape)
    des_z = np.zeros(store_shape)
    des_y = np.zeros(store_shape)
    des_vx = np.zeros(store_shape)
    des_vy = np.zeros(store_shape)
    des_vz = np.zeros(store_shape)
    des_phi = np.zeros(store_shape)
    des_psi = np.zeros(store_shape)
    F = np.zeros(store_shape)
    Mx = np.zeros(store_shape)
    My = np.zeros(store_shape)
    Mz = np.zeros(store_shape)
    Fa1x = np.zeros(store_shape)
    Fa1y = np.zeros(store_shape)
    Fa1z = np.zeros(store_shape)
    taux = np.zeros(store_shape)
    tauy = np.zeros(store_shape)
    tauz = np.zeros(store_shape)
    des_theta = np.zeros(store_shape)
    des_thrust = np.zeros(store_shape)
    des_thetadot = np.zeros(store_shape)
    des_phidot = np.zeros(store_shape)
    des_psidot = np.zeros(store_shape)
    
    des_start = trajhandle(0, [])
    x0    = init_state(des_start.pos,des_start.vel,des_start.rot,des_start.omega)   
    x0[4]=1.6

    t = 0

     # ************************* RUN SIMULATION *************************
    print('Simulation Running....')

    # Main loop
    iteration = 0
    for iteration in range(int(max_iter)):

        # timeint = time:tstep:time +cstep;
        timeint = np.arange(time, time+cstep+tstep/10, tstep)
        
        #lambda t,y: rhs_2nd_order_ode(t,y,a,b)
        #Since we cannot pass the variable scorrecty into it,
        results = solve_ivp(lambda t,y: quadEOM(t, y, controlhandle, trajhandle, BQ), (time, time+cstep), x0, t_eval=timeint)
        
        xsave = np.transpose(results.y)
        tsave = results.t
        x     = np.transpose(xsave[-1, :])
        
        #print('xsave', xsave.shape)
        #print('x', x.shape)
        
        # Save to traj
        xtraj[iteration*nstep:(iteration+1)*nstep,:] = xsave[0:-1,:];
        ttraj[iteration*nstep:(iteration+1)*nstep] = tsave[0:-1];
        
        #print('Test',nstep,fort_t)
        for i in range(nstep):
            if (backt_t>0 and ttraj[iteration*nstep+i]>=backt_t):
                trajhandle = time_traj_land;
                des_state = trajhandle(ttraj[iteration*nstep+i], []);
                #des_state.pos(2);
                des_state.pos[1]=des_state.pos[1]+backt_y;
                des_state.pos[0]=des_state.pos[0]+backt_x;
            
            elif (cruise_t>0 and ttraj[iteration*nstep+i]>cruise_t):
                trajhandle = time_traj_backtrans;
                des_state = trajhandle(ttraj[iteration*nstep+i], []);
                des_state.pos[2]=des_state.pos[2]+cruise_z;
                des_state.pos[1]=des_state.pos[1]+cruise_y;
                des_state.pos[0]=des_state.pos[0]+cruise_x;
            
            elif (fort_t>0 and ttraj(iteration*nstep+i)>fort_t):
                trajhandle = time_traj_cruise;
                des_state = trajhandle(ttraj[iteration*nstep+i], []);
                des_state.pos[2]=des_state.pos[2]+fort_z;
                des_state.pos[1]=des_state.pos[1]+fort_y;
                des_state.pos[0]=des_state.pos[0]+fort_x;
            
            elif (hover_t>0 and ttraj(iteration*nstep+i)>hover_t):
                trajhandle = time_traj_fortrans;
                des_state = trajhandle(ttraj[iteration*nstep+i], []);
                des_state.pos[2]=des_state.pos[2]+hover_z
                des_state.pos[1]=des_state.pos[1]+hover_y
                des_state.pos[0]=des_state.pos[0]+hover_x
                
            else:
                trajhandle = time_traj_hover
                des_state = trajhandle(ttraj[iteration*nstep+i], [])
                #print(des_state.pos[2])


            des_z[iteration*nstep+i]=des_state.pos[2]
            des_x[iteration*nstep+i]=des_state.pos[0]
            des_y[iteration*nstep+i]=des_state.pos[1]
            des_vx[iteration*nstep+i]=des_state.vel[0]
            des_vy[iteration*nstep+i]=des_state.vel[1]
            des_vz[iteration*nstep+i]=des_state.vel[2]
            des_phi[iteration*nstep+i]=des_state.rot[0]
            des_theta[iteration*nstep+i]=des_state.rot[1]
            des_thetadot[iteration*nstep+i]=des_state.omega[1]
            des_phidot[iteration*nstep+i]=des_state.omega[0]
            des_psidot[iteration*nstep+i]=des_state.omega[2]

            des_psi[iteration*nstep+i]=des_state.rot[2]
            des_thrust[iteration*nstep+i]=des_state.control[0]
            s1 = xsave[i,:]
            current_state = stateToQd(s1)
            [F1, Fa1, M1, tau_a1] = controlhandle(tsave[i], current_state, des_state, BQ)
     
            F[iteration*nstep+i]=F1
            Mx[iteration*nstep+i]=M1[0]
            My[iteration*nstep+i]=M1[1]
            Mz[iteration*nstep+i]=M1[2]
            Fa1x[iteration*nstep+i]=Fa1[0]
            Fa1y[iteration*nstep+i]=Fa1[1]
            Fa1z[iteration*nstep+i]=Fa1[2]
            taux[iteration*nstep+i]=tau_a1[0]
            tauy[iteration*nstep+i]=tau_a1[1]
            tauz[iteration*nstep+i]=tau_a1[2]
        
        time = time + cstep; #% Update simulation time
        
      #  %t = toc

    # ************************* POST PROCESSING *************************
    # Truncate xtraj and ttraj
    iteration = iteration + 1
    xtraj = xtraj[0:iteration*nstep,:]
    ttraj = ttraj[0:iteration*nstep]
    
    des_x=des_x[0:iteration*nstep]
    des_z=des_z[0:iteration*nstep]
    des_y=des_y[0:iteration*nstep]
    des_theta=des_theta[0:iteration*nstep]
    
    #fig = plt.figure()
    #fig,axs = plt.subplots(3,3)

    #axs[0,0].plot(ttraj,xtraj[:,0],'b')
    #axs[0,0].plot(ttraj,des_x,'r')
#     ylabel('x');

    #axs[0,1].plot(ttraj,xtraj[:,1],'b')
    #axs[0,1].plot(ttraj,des_y,'r')
#     ylabel('y');

    #axs[0,2].plot(ttraj,xtraj[:,2],'b')
    #axs[0,2].plot(ttraj,des_z,'r')
    #plt.show()
#     ylabel('z');

#     subplot(3,3,4)
#     plot(ttraj,xtraj(:,4),'b');
#     hold on
#     plot(ttraj,des_vx,'r');
#     ylabel('vx');

#     subplot(3,3,5)
#     plot(ttraj,xtraj(:,5),'b');
#     hold on
#     plot(ttraj,des_vy,'r');
#     ylabel('vy');

#     subplot(3,3,6)
#     plot(ttraj,xtraj(:,6),'b');
#     hold on
#     plot(ttraj,des_vz,'r');
#     ylabel('vz');

#     subplot(3,3,7)
#     plot(ttraj,xtraj(:,7),'b');
#     hold on
#     plot(ttraj,des_phi,'r');
#     ylabel('phi');

#     subplot(3,3,8)
#     plot(ttraj,xtraj(:,8),'b');
#     hold on
#     plot(ttraj,des_theta,'r');
#     ylabel('theta');

#     subplot(3,3,9)
#     plot(ttraj,xtraj(:,9),'b');
#     hold on
#     plot(ttraj,des_psi,'r');
#     ylabel('psi');

    """plt.figure()
    plt.plot(ttraj,F,'b')
    plt.ylabel('thrust')
    
    plt.figure()
    plt.plot(ttraj,Mx,label = 'Mx')
    plt.plot(ttraj,My,label = 'My')
    plt.plot(ttraj,Mz,label = 'Mz')
    plt.ylabel('controls')
    
    plt.figure()
    plt.plot(xtraj[:,0],xtraj[:,2],'b')
    plt.plot(des_x,des_z,'r')
    plt.ylabel('traj')

    plt.figure()
    plt.plot(ttraj,Fa1x,'r')
    plt.plot(ttraj,Fa1y,'g')
    plt.plot(ttraj,Fa1z,'b')
    plt.ylabel('Fa')

    plt.figure()
    plt.plot(ttraj,taux,'r')
    plt.plot(ttraj,tauy,'g')
    plt.plot(ttraj,tauz,'b')
    plt.ylabel('tau')

    plt.figure()
    plt.plot(ttraj,xtraj[:,7],'b')
    plt.plot(ttraj,des_theta,'r')
    plt.ylabel('theta')

    plt.figure()
    plt.plot(ttraj,xtraj[:,10],'b')
    plt.plot(ttraj,des_thetadot,'r')
    plt.ylabel('thetadot')"""
             
# %         figure(9)
# %     plot(ttraj,xtraj(:,10),'b');
# %     hold on
# %     plot(ttraj,des_phidot,'r');
# %     ylabel('phidot')
# %     
# %             figure(10)
# %         plot(ttraj,xtraj(:,12),'b');
# %     hold on
# %     plot(ttraj,des_psidot,'r');
# %     ylabel('psidot')
# %     figure(8)
# %     plot(ttraj,des_thrust,'b');
# %     ylabel('des thrust');
    
    plt.show()
    print('finished')
    t_out = ttraj
    s_out = xtraj

    return t_out, s_out