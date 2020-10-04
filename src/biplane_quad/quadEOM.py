def quadEOM(t, s, controlhandle, trajhandle, BQ):
% QUADEOM Wrapper function for solving quadrotor equation of motion
% 	quadEOM takes in time, state vector, controller, trajectory generator
% 	and parameters and output the derivative of the state vector, the
% 	actual calcution is done in quadEOM_readonly.
%
% INPUTS:
% t             - 1 x 1, time
% s             - 12 x 1, state vector = [x, y, z, xd, yd, zd, psi, theta, phi, psidot, thetadot, phidot ]
% controlhandle - function handle of your controller
% trajhandle    - function handle of your trajectory generator
% params        - struct, output from sys_params() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot          - 12 x 1, derivative of state vector s

% convert state to quad stuct for control
global flag hover_x hover_y hover_z fort_x fort_y fort_z backt_t cruise_z cruise_x cruise_y backt_x backt_y backt_z
current_state = stateToQd(s);
 
if (flag ==1)
    trajhandle = @time_traj_hover;
elseif (flag ==2)
    trajhandle = @time_traj_fortrans;
elseif (flag ==3)
    trajhandle = @time_traj_cruise;
elseif (flag == 4)
    trajhandle = @time_traj_backtrans;
elseif (flag == 5)
    trajhandle = @time_traj_land;
end

% Get desired_state
desired_state = trajhandle(t, current_state);

if (flag ==2)
    desired_state.pos(3)=desired_state.pos(3)+hover_z;
    desired_state.pos(2)=desired_state.pos(2)+hover_y;
    desired_state.pos(1)=desired_state.pos(1)+hover_x;
    
elseif (flag==3)
    desired_state.pos(1)=desired_state.pos(1)+fort_x;
    desired_state.pos(2)=desired_state.pos(2)+fort_y;
    desired_state.pos(3)=desired_state.pos(3)+fort_z;
    
elseif (flag==4)
    desired_state.pos(1)=desired_state.pos(1)+cruise_x;
    desired_state.pos(2)=desired_state.pos(2)+cruise_y;
    desired_state.pos(3)=desired_state.pos(3)+cruise_z;
elseif (flag==5)
    desired_state.pos(1)=desired_state.pos(1)+backt_x;
    desired_state.pos(2)=desired_state.pos(2)+backt_y;
    desired_state.pos(3)=desired_state.pos(3);
end
% get control outputs
[F, Fa, M, tau_a] = controlhandle(t, current_state, desired_state, BQ);

% compute derivative
sdot = quadEOM_readonly(t, s, F, Fa, M, tau_a, BQ);

end
function sdot = quadEOM(t, s, controlhandle, trajhandle, BQ)
% QUADEOM Wrapper function for solving quadrotor equation of motion
% 	quadEOM takes in time, state vector, controller, trajectory generator
% 	and parameters and output the derivative of the state vector, the
% 	actual calcution is done in quadEOM_readonly.
%
% INPUTS:
% t             - 1 x 1, time
% s             - 12 x 1, state vector = [x, y, z, xd, yd, zd, psi, theta, phi, psidot, thetadot, phidot ]
% controlhandle - function handle of your controller
% trajhandle    - function handle of your trajectory generator
% params        - struct, output from sys_params() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot          - 12 x 1, derivative of state vector s

% convert state to quad stuct for control
global flag hover_x hover_y hover_z fort_x fort_y fort_z backt_t cruise_z cruise_x cruise_y backt_x backt_y backt_z
current_state = stateToQd(s);
 
if (flag ==1)
    trajhandle = @time_traj_hover;
elseif (flag ==2)
    trajhandle = @time_traj_fortrans;
elseif (flag ==3)
    trajhandle = @time_traj_cruise;
elseif (flag == 4)
    trajhandle = @time_traj_backtrans;
elseif (flag == 5)
    trajhandle = @time_traj_land;
end

% Get desired_state
desired_state = trajhandle(t, current_state);

if (flag ==2)
    desired_state.pos(3)=desired_state.pos(3)+hover_z;
    desired_state.pos(2)=desired_state.pos(2)+hover_y;
    desired_state.pos(1)=desired_state.pos(1)+hover_x;
    
elseif (flag==3)
    desired_state.pos(1)=desired_state.pos(1)+fort_x;
    desired_state.pos(2)=desired_state.pos(2)+fort_y;
    desired_state.pos(3)=desired_state.pos(3)+fort_z;
    
elseif (flag==4)
    desired_state.pos(1)=desired_state.pos(1)+cruise_x;
    desired_state.pos(2)=desired_state.pos(2)+cruise_y;
    desired_state.pos(3)=desired_state.pos(3)+cruise_z;
elseif (flag==5)
    desired_state.pos(1)=desired_state.pos(1)+backt_x;
    desired_state.pos(2)=desired_state.pos(2)+backt_y;
    desired_state.pos(3)=desired_state.pos(3);
end
% get control outputs
[F, Fa, M, tau_a] = controlhandle(t, current_state, desired_state, BQ);

% compute derivative
sdot = quadEOM_readonly(t, s, F, Fa, M, tau_a, BQ);

return sdot
