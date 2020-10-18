import globals
from globals import systemParameters

def sim_3d(trajhandle, controlhandle):
    max_time = 10; #6
    #% parameters for simulation
    BQ = systemParameters()

    # *********************** INITIAL CONDITIONS ***********************
    print('Setting initial conditions...')

    tstep    = 0.01#; % this determines the time step at which the solution is given%0.02
    cstep    = 0.04#; % image capture time interval 0.06
    max_iter = max_time/cstep#; % max iteration
    nstep    = cstep/tstep#;
    time     = 0#; % current time
    # % Get start position
    des_start = trajhandle(0, []);
    x0    = init_state(des_start.pos,des_start.vel,des_start.rot,des_start.omega);

    #%x0(9)=1.6;
    xtraj = zeros(max_iter*nstep, length(x0));
    ttraj = zeros(max_iter*nstep, 1);
    des_x = zeros(max_iter*nstep, 1);
    des_z = zeros(max_iter*nstep, 1);
    des_y = zeros(max_iter*nstep, 1);
    des_vx = zeros(max_iter*nstep, 1);
    des_vy = zeros(max_iter*nstep, 1);
    des_vz = zeros(max_iter*nstep, 1);
    des_phi = zeros(max_iter*nstep, 1);
    des_psi = zeros(max_iter*nstep, 1);
    F = zeros(max_iter*nstep, 1);
    Mx = zeros(max_iter*nstep, 1);
    My = zeros(max_iter*nstep, 1);
    Mz = zeros(max_iter*nstep, 1);
    Fa1x = zeros(max_iter*nstep, 1);
    Fa1y = zeros(max_iter*nstep, 1);
    Fa1z = zeros(max_iter*nstep, 1);
    taux = zeros(max_iter*nstep, 1);
    tauy = zeros(max_iter*nstep, 1);
    tauz = zeros(max_iter*nstep, 1);
    des_theta = zeros(max_iter*nstep, 1);
    des_thrust=zeros(max_iter*nstep, 1);
    des_thetadot=zeros(max_iter*nstep, 1);
    des_phidot=zeros(max_iter*nstep, 1);
    des_psidot=zeros(max_iter*nstep, 1);

    x       = x0;    #    % state
    #%% ************************* RUN SIMULATION *************************
    print('Simulation Running....');
    # % Main loop
    for iter in range(1,max_iter):

        timeint = time:tstep:time +cstep;
        
        # %tic;
        # % Run simulation
        [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, BQ), timeint, x);
        x    = xsave(end, :)';
       
        % Save to traj
        xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

        
        for i=1:nstep
    %         if (flag ==1)
    %             trajhandle = @time_traj_hover;
    %         else if (flag ==2)
    %                 trajhandle = @time_traj_fortrans;
    %             else if (flag ==3)
    %                     trajhandle = @time_traj_backtrans;
    %                 end
    %             end
    %         end
            if (backt_t>0 && ttraj((iter-1)*nstep+i)>=backt_t)
                trajhandle = @time_traj_land;
                des_state = trajhandle(ttraj((iter-1)*nstep+i), []);
                des_state.pos(3);
                des_state.pos(2)=des_state.pos(2)+backt_y;
                des_state.pos(1)=des_state.pos(1)+backt_x;
            elseif (cruise_t>0 && ttraj((iter-1)*nstep+i)>cruise_t)
                trajhandle = @time_traj_backtrans;
                des_state = trajhandle(ttraj((iter-1)*nstep+i), []);
                des_state.pos(3)=des_state.pos(3)+cruise_z;
                des_state.pos(2)=des_state.pos(2)+cruise_y;
                des_state.pos(1)=des_state.pos(1)+cruise_x;
            elseif (fort_t>0 && ttraj((iter-1)*nstep+i)>fort_t)
                trajhandle = @time_traj_cruise;
                des_state = trajhandle(ttraj((iter-1)*nstep+i), []);
                des_state.pos(3)=des_state.pos(3)+fort_z;
                des_state.pos(2)=des_state.pos(2)+fort_y;
                des_state.pos(1)=des_state.pos(1)+fort_x;
            elseif (hover_t>0 && ttraj((iter-1)*nstep+i)>hover_t)
                trajhandle = @time_traj_fortrans;
                des_state = trajhandle(ttraj((iter-1)*nstep+i), []);
                des_state.pos(3)=des_state.pos(3)+hover_z;
                des_state.pos(2)=des_state.pos(2)+hover_y;
                des_state.pos(1)=des_state.pos(1)+hover_x;
                
            else
                trajhandle = @time_traj_hover;
                des_state = trajhandle(ttraj((iter-1)*nstep+i), []);
                des_state.pos(3);
                %des_state.pos(3)=des_state.pos(3)+1;
            end
                
    %         else
    %             trajhandle = @time_traj_cruise;
    %             des_state = trajhandle(ttraj((iter-1)*nstep+i), []);
    %             des_state.pos(1)=des_state.pos(1)+5.13;         
           
            des_z((iter-1)*nstep+i)=des_state.pos(3);
           
            des_x((iter-1)*nstep+i)=des_state.pos(1);
            des_y((iter-1)*nstep+i)=des_state.pos(2);
            des_vx((iter-1)*nstep+i)=des_state.vel(1);
            des_vy((iter-1)*nstep+i)=des_state.vel(2);
            des_vz((iter-1)*nstep+i)=des_state.vel(3);
            des_phi((iter-1)*nstep+i)=des_state.rot(1);
            des_theta((iter-1)*nstep+i)=des_state.rot(2);
            des_thetadot((iter-1)*nstep+i)=des_state.omega(2);
            des_phidot((iter-1)*nstep+i)=des_state.omega(1);
            des_psidot((iter-1)*nstep+i)=des_state.omega(3);

            des_psi((iter-1)*nstep+i)=des_state.rot(3);
            des_thrust((iter-1)*nstep+i)=des_state.control(1);
            s1 = xsave(i,:);
            current_state = stateToQd(s1);
            [F1, Fa1, M1, tau_a1] = controlhandle(tsave(i), current_state, des_state, BQ);
     
            F((iter-1)*nstep+i)=F1;
            Mx((iter-1)*nstep+i)=M1(1);
            My((iter-1)*nstep+i)=M1(2);
            Mz((iter-1)*nstep+i)=M1(3);
            Fa1x((iter-1)*nstep+i)=Fa1(1);
            Fa1y((iter-1)*nstep+i)=Fa1(2);
            Fa1z((iter-1)*nstep+i)=Fa1(3);
            taux((iter-1)*nstep+i)=tau_a1(1);
            tauy((iter-1)*nstep+i)=tau_a1(2);
            tauz((iter-1)*nstep+i)=tau_a1(3);

         end

        time = time + cstep; % Update simulation time
        
        %t = toc;
    iter;

    end

    %% ************************* POST PROCESSING *************************
    % Truncate xtraj and ttraj
    xtraj = xtraj(1:iter*nstep,:);
    ttraj = ttraj(1:iter*nstep);
    des_x=des_x(1:iter*nstep);
    des_z=des_z(1:iter*nstep);
    des_y=des_y(1:iter*nstep);
    des_theta=des_theta(1:iter*nstep);


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
    t_out = ttraj;
    s_out = xtraj;

return t_out, s_out