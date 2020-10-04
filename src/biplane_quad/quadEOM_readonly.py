function sdot = quadEOM_readonly(t, s, F, Fa, M, tau_a, BQ)

% QUADEOM_READONLY Solve quadrotor equation of motion
%   quadEOM_readonly calculate the derivative of the state vector
%
% INPUTS:
% t      - 1 x 1, time
% s      - 12 x 1, state vector = [x, y, z, xd, yd, zd, psi, theta, phi, psidot, thetadot, phidot]
% F      - 1 x 1, thrust output from controller (only used in simulation)
% Fa     - 3 x 1, aerodynamic force at the current state from AeroFEst
% tau_a  - 3 x 1, aerodynamic moment at the current state from AeroMEst
% M      - 3 x 1, moments output from controller (only used in simulation)
% params - struct, output from nanoplus() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot   - 12 x 1, derivative of state vector s

%************ EQUATIONS OF MOTION ************************

% Assign states
x = s(1);
y = s(2);
z = s(3);
xdot = s(4);
ydot = s(5);
zdot = s(6);
phic = s(7);
thetac = s(8);
psic = s(9);
phidotc = s(10);
thetadotc = s(11);
psidotc = s(12);

% Acceleration
R=eul2rotm([phic thetac psic]);
accel=(R*([0;0;F]+Fa)/BQ.m)-[0; 0; BQ.g];

% Angular acceleration
% omega=[1 0 -sin(thetac);0 cos(phic) cos(thetac)*sin(phic);0 -sin(phic) cos(thetac)*cos(phic)]*[phidotc; thetadotc; psidotc];
% pqrdot   = (BQ.J)\(M - cross(omega, BQ.J*omega)+tau_a);
% eulddot = [1 sin(phic)*tan(thetac) cos(phic)*tan(thetac);0 cos(phic) -sin(phic);0 sin(phic)*sec(thetac) sec(thetac)*cos(phic)]*pqrdot;
Eul_dot=[phidotc;thetadotc;psidotc];
A=[1 0 -sin(thetac);
    0 cos(phic) sin(phic)*cos(thetac);
    0 -sin(phic) cos(phic)*cos(thetac)];
Omega=A*Eul_dot;

Omega_dot   = (BQ.J)\(M+tau_a-cross(Omega',(BQ.J*Omega)')');

B=[0 0 cos(thetac)*Eul_dot(2);
    0 sin(phic)*Eul_dot(1) -sin(phic)*sin(thetac)*Eul_dot(2)-cos(phic)*cos(thetac)*Eul_dot(1);
    0 cos(phic)*Eul_dot(1) -sin(thetac)*cos(phic)*Eul_dot(2)+sin(phic)*cos(thetac)*Eul_dot(1)];

eulddot=(A)\(Omega_dot-B*Eul_dot);

%eulddot   = (BQ.J)\(M+tau_a);
% Assemble sdot
sdot = zeros(12,1);
sdot(1)  = xdot;
sdot(2)  = ydot;
sdot(3)  = zdot;
sdot(4)  = accel(1);
sdot(5)  = accel(2);
%sdot(5)  = 0;
sdot(6)  = accel(3);
sdot(7)  = phidotc;
sdot(8)  = thetadotc;
sdot(9)  = psidotc;
sdot(10) = eulddot(1);
%sdot(10) = 0;
sdot(11) = eulddot(2);
sdot(12) = eulddot(3);
%sdot(12) = 0;
sdot;
end
