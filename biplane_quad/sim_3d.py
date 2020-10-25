from globals import flag, hover_t, hover_z, hover_x, hover_y
from globals import t, s
from globals import systemParameters
from scipy.integrate import solve_ivp
from init_state import init_state
from quadEOM import quadEOM
import numpy as np
global t,s

#quick fix for matlab
end=-1

def sim_3d(trajhandle, controlhandle):
    max_time = 10000; #6
    #% parameters for simulation
    BQ = systemParameters()
    # *********************** INITIAL CONDITIONS ***********************
    print('Setting initial conditions...')

    tstep    = 0.01#; % this determines the time step at which the solution is given%0.02
    cstep    = 0.04#; % image capture time interval 0.06
    max_iter = max_time/cstep#; % max iteration
    nstep    = int(cstep/tstep)#;
    time     = 0#; % current time
    # % Get start position
    # des_start = trajhandle(0, []);
    # x0    = init_state(des_start.pos,des_start.vel,des_start.rot,des_start.omega);

    store_shape = int(max_iter*nstep)
    xtraj = np.zeros((store_shape, 12));
    ttraj = np.zeros(store_shape);
    des_x = np.zeros(store_shape);
    des_z = np.zeros(store_shape);
    des_y = np.zeros(store_shape);
    des_vx = np.zeros(store_shape);
    des_vy = np.zeros(store_shape);
    des_vz = np.zeros(store_shape);
    des_phi = np.zeros(store_shape);
    des_psi = np.zeros(store_shape);
    F = np.zeros(store_shape);
    Mx = np.zeros(store_shape);
    My = np.zeros(store_shape);
    Mz = np.zeros(store_shape);
    Fa1x = np.zeros(store_shape);
    Fa1y = np.zeros(store_shape);
    Fa1z = np.zeros(store_shape);
    taux = np.zeros(store_shape);
    tauy = np.zeros(store_shape);
    tauz = np.zeros(store_shape);
    des_theta = np.zeros(store_shape);
    des_thrust=np.zeros(store_shape);
    des_thetadot=np.zeros(store_shape);
    des_phidot=np.zeros(store_shape);
    des_psidot=np.zeros(store_shape);
    
    des_start = trajhandle(0, []);
    x0    = init_state(des_start.pos,des_start.vel,des_start.rot,des_start.omega);   
    x0[4]=1.6;

    t = 0

    #%% ************************* RUN SIMULATION *************************
    print('Simulation Running....');

    # % Main loop
    for iter in range(1,int(max_iter)):

        # timeint = time:tstep:time +cstep;
        timeint = np.arange(time, time+cstep+tstep/10, tstep)
        #print('time', timeint)
        ##lambda t,y: rhs_2nd_order_ode(t,y,a,b)
        ##Since we cannot pass the variable scorrecty into it,
        #[tsave, xsave] =
        results = solve_ivp(lambda t,y: quadEOM(t, y, controlhandle, trajhandle, BQ), (time, time+cstep), x0, t_eval=timeint)
        #print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        #print(results.t, results.y)
        xsave = results.y
        tsave = results.t
        #x    = results.y[end, :].T
        
        #print('xsave', xsave.shape)
        #print('xtraj', xtraj.shape)
        # Save to traj
        xtraj[(iter-1)*nstep:iter*nstep,:] = (xsave.T)[0:-1,:];
        ttraj[(iter-1)*nstep:iter*nstep] = tsave[0:-1];
        break
        
        for i in range(1,nstep+1):
    # %         if (flag ==1)
    # %             trajhandle = @time_traj_hover;
    # %         else if (flag ==2)
    # %                 trajhandle = @time_traj_fortrans;
    # %             else if (flag ==3)
    # %                     trajhandle = @time_traj_backtrans;
    # %                 end
    # %             end
    # %         end
            if (backt_t>0 and ttraj[(iter-1)*nstep+i-1]>=backt_t):
                trajhandle = time_traj_land;
                des_state = trajhandle(ttraj[(iter-1)*nstep+i], []);
                #des_state.pos(2);
                des_state.pos[1]=des_state.pos[1]+backt_y;
                des_state.pos[0]=des_state.pos[0]+backt_x;
            elif (cruise_t>0 and ttraj[(iter-1)*nstep+i]>cruise_t):
                trajhandle = time_traj_backtrans;
                des_state = trajhandle(ttraj[(iter-1)*nstep+i], []);
                des_state.pos[2]=des_state.pos[2]+cruise_z;
                des_state.pos[1]=des_state.pos[1]+cruise_y;
                des_state.pos[0]=des_state.pos[0]+cruise_x;
            elif (fort_t>0 and ttraj((iter-1)*nstep+i)>fort_t):
                trajhandle = time_traj_cruise;
                des_state = trajhandle(ttraj[(iter-1)*nstep+i], []);
                des_state.pos[2]=des_state.pos[2]+fort_z;
                des_state.pos[1]=des_state.pos[1]+fort_y;
                des_state.pos[0]=des_state.pos[0]+fort_x;
            elif (hover_t>0 and ttraj((iter-1)*nstep+i)>hover_t):
                trajhandle = time_traj_fortrans;
                des_state = trajhandle(ttraj[(iter-1)*nstep+i], []);
                des_state.pos[2]=des_state.pos[2]+hover_z;
                des_state.pos[1]=des_state.pos[1]+hover_y;
                des_state.pos[0]=des_state.pos[0]+hover_x;
                
            else:
                trajhandle = time_traj_hover;
                des_state = trajhandle(ttraj((iter-1)*nstep+i), []);
                des_state.pos(3);
                #%des_state.pos(3)=des_state.pos(3)+1;
            """    
    %         else
    %             trajhandle = @time_traj_cruise;
    %             des_state = trajhandle(ttraj((iter-1)*nstep+i), []);
    %             des_state.pos(1)=des_state.pos(1)+5.13;         
           """
            des_z[(iter-1)*nstep+i]=des_state.pos[2];
            des_x[(iter-1)*nstep+i]=des_state.pos[0];
            des_y[(iter-1)*nstep+i]=des_state.pos[1];
            des_vx[(iter-1)*nstep+i]=des_state.vel[0];
            des_vy[(iter-1)*nstep+i]=des_state.vel[1];
            des_vz[(iter-1)*nstep+i]=des_state.vel[2];
            des_phi[(iter-1)*nstep+i]=des_state.rot[0];
            des_theta[(iter-1)*nstep+i]=des_state.rot[1];
            des_thetadot[(iter-1)*nstep+i]=des_state.omega[1];
            des_phidot[(iter-1)*nstep+i]=des_state.omega[0];
            des_psidot[(iter-1)*nstep+i]=des_state.omega[2];

            des_psi[(iter-1)*nstep+i]=des_state.rot[2];
            des_thrust[(iter-1)*nstep+i]=des_state.control[0];
            s1 = xsave[i,:];
            current_state = stateToQd(s1);
            [F1, Fa1, M1, tau_a1] = controlhandle(tsave(i), current_state, des_state, BQ);
     
            F[(iter-1)*nstep+i]=F1;
            Mx[(iter-1)*nstep+i]=M1[0];
            My[(iter-1)*nstep+i]=M1[1];
            Mz[(iter-1)*nstep+i]=M1[2];
            Fa1x[(iter-1)*nstep+i]=Fa1[0];
            Fa1y[(iter-1)*nstep+i]=Fa1[1];
            Fa1z[(iter-1)*nstep+i]=Fa1[2];
            taux[(iter-1)*nstep+i]=tau_a1[0];
            tauy[(iter-1)*nstep+i]=tau_a1[1];
            tauz[(iter-1)*nstep+i]=tau_a1[2];
        break
        time = time + cstep; #% Update simulation time
        
      #  %t = toc;
    iter;

    #%% ************************* POST PROCESSING *************************
    #% Truncate xtraj and ttraj
    xtraj = xtraj[1:iter*nstep,:];
    ttraj = ttraj[1:iter*nstep];
    des_x=des_x[1:iter*nstep];
    des_z=des_z[1:iter*nstep];
    des_y=des_y[1:iter*nstep];
    des_theta=des_theta[1:iter*nstep];

    """
        figure(1)
        subplot(3,3,1)
        plot(ttraj,xtraj(:,1),'b');
        hold on
        plot(ttraj,des_x,'r');
        ylabel('x');
        
        subplot(3,3,2)
        plot(ttraj,xtraj(:,2),'b');
        hold on
        plot(ttraj,des_y,'r');
        ylabel('y');
        
        subplot(3,3,3)
        plot(ttraj,xtraj(:,3),'b');
        hold on
        plot(ttraj,des_z,'r');
        ylabel('z');
        
        subplot(3,3,4)
        plot(ttraj,xtraj(:,4),'b');
        hold on
        plot(ttraj,des_vx,'r');
        ylabel('vx');
        
        subplot(3,3,5)
        plot(ttraj,xtraj(:,5),'b');
        hold on
        plot(ttraj,des_vy,'r');
        ylabel('vy');
        
        subplot(3,3,6)
        plot(ttraj,xtraj(:,6),'b');
        hold on
        plot(ttraj,des_vz,'r');
        ylabel('vz');
        
        subplot(3,3,7)
        plot(ttraj,xtraj(:,7),'b');
        hold on
        plot(ttraj,des_phi,'r');
        ylabel('phi');
           
        subplot(3,3,8)
        plot(ttraj,xtraj(:,8),'b');
        hold on
        plot(ttraj,des_theta,'r');
        ylabel('theta');
        
        subplot(3,3,9)
        plot(ttraj,xtraj(:,9),'b');
        hold on
        plot(ttraj,des_psi,'r');
        ylabel('psi');
        
        figure(2)
        plot(ttraj,F,'b');
        ylabel('thrust');
        
        figure(3)
        plot(ttraj,Mx,'r');
        hold on
        plot(ttraj,My,'g');
        hold on
        plot(ttraj,Mz,'b');
        ylabel('controls');
        
        figure(4)
        plot(xtraj(:,1),xtraj(:,3),'b');
        hold on
        plot(des_x,des_z,'r');
        ylabel('traj');
        
        figure(5)
        plot(ttraj,Fa1x,'r');
        hold on
        plot(ttraj,Fa1y,'g');
        hold on
        plot(ttraj,Fa1z,'b');
        ylabel('Fa');
        
        figure(6)
        plot(ttraj,taux,'r');
        hold on
        plot(ttraj,tauy,'g');
        hold on
        plot(ttraj,tauz,'b');
        ylabel('tau');

        figure(7)
        plot(ttraj,xtraj(:,8),'b');
        hold on
        plot(ttraj,des_theta,'r');
        ylabel('theta');
        
        figure(8)
        plot(ttraj,xtraj(:,11),'b');
        hold on
        plot(ttraj,des_thetadot,'r');
        ylabel('thetadot');
    %         figure(9)
    %     plot(ttraj,xtraj(:,10),'b');
    %     hold on
    %     plot(ttraj,des_phidot,'r');
    %     ylabel('phidot')
    %     
    %             figure(10)
    %         plot(ttraj,xtraj(:,12),'b');
    %     hold on
    %     plot(ttraj,des_psidot,'r');
    %     ylabel('psidot')
    %     figure(8)
    %     plot(ttraj,des_thrust,'b');
    %     ylabel('des thrust');
        
    disp('finished.')
    """
    t_out = ttraj
    s_out = xtraj

    return t_out, s_out