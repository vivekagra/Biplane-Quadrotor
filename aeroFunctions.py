from globals import system_paramters

def momentEstimate(eul, x_dot, omega, Fa):
	BQ = systemParameters()
	rho=1.225
	Moment_aero=(np.zeros(3)).T

	R=eul2rotm(eul)
	xb_dot=R'*x_dot;
	V=norm(x_dot);

	if (V==0) return;
	end

	Rq2w = [
	    0 0 1;
	    0 1 0;
	    -1 0 0];
	xw_dot = Rq2w*xb_dot;

	if (xw_dot(1)~=0)
	    alpha=atan2(-xw_dot(3),xw_dot(1));
	    beta=atan2(xw_dot(2),xw_dot(1));
	else
	    alpha=0;
	    beta=0;
	end

	omega_w = Rq2w*omega;
	p_w = omega_w(1,1);
	q_w = omega_w(2,1);
	r_w = omega_w(3,1);

	C_l = BQ.Cl_beta*beta + BQ.Cl_p*p_w*BQ.b/(2*V) + BQ.Cl_r*r_w*BQ.b/(2*V);
	C_m = BQ.Cm_0 + BQ.Cm_alpha*alpha + BQ.Cm_q*q_w*BQ.c/(2*V);
	C_n = BQ.Cn_beta*beta + BQ.Cn_p*p_w*BQ.b/(2*V) + BQ.Cn_r*r_w*BQ.b/(2*V);

	Mx_w_ac = 0.5*rho*(V^2)*BQ.S*BQ.c*C_l ;
	My_w_ac = 0.5*rho*(V^2)*BQ.S*BQ.c*C_m ;
	Mz_w_ac = 0.5*rho*(V^2)*BQ.S*BQ.c*C_n ;

	M_ac = [Mx_w_ac;My_w_ac;Mz_w_ac];% wing frame
	% r = BQ.x_ac - BQ.x_cg;
	% M_cg = M_ac + cross(r',(Rq2w*Fa)')';
	r = BQ.x_ac(1) - BQ.x_cg(1);
	% L = -Fa(1);
	% M_cg = M_ac + [0;r*L;0]; % wing frame

	% Moment_aero = BQ.wing_n*Rq2w'*M_cg; % body frame

	Moment_aero = [0;BQ.wing_n*My_w_ac+r*Fa(1);0];
	end

def AeroFEst(eul,x_dot,omega):
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
BQ = System_Parameters;
Fa=[0;0;0];
Fa_w=[0;0;0];
alpha=0;
beta=0;a

p=omega(1,1);
q=omega(2,1);
r=omega(3,1);
rho=1.225;
% Eul=quat2eul((quat/norm(quat))');
% psi=Eul(1,1);
% theta=Eul(1,2);
% phi=Eul(1,3);
R=eul2rotm(eul);
xb_dot=R'*x_dot;
V=norm(x_dot);

if (V==0) return;
end

Rq2w = [
    0 0 1;
    0 1 0;
    -1 0 0];
xw_dot = Rq2w*xb_dot;

% alpha=atan2(-xw_dot(3),xw_dot(1));
% beta=atan2(xw_dot(2),xw_dot(1));

if (xw_dot(1)~=0)
    alpha=atan2(-xw_dot(3),xw_dot(1));
    beta=atan2(xw_dot(2),xw_dot(1));
else
    alpha=0;
    beta=0;
end
% alpha=atan2(xb_dot(1),xb_dot(3));
% beta=atan2(xb_dot(2),xb_dot(3));

% alpha = abs(alpha);
% beta = abs(beta);

A=[ sin(alpha)*cos(beta)    -sin(alpha)*sin(beta)      cos(alpha) ;
    sin(beta)               cos(beta)                 0       ;
    -cos(alpha)*cos(beta)   cos(alpha)*sin(beta)     sin(alpha)   ];

sigma_a = (1 + exp(-BQ.M*(alpha-BQ.alpha0)) + exp(BQ.M*(alpha+BQ.alpha0)))/(( 1 + exp(-BQ.M*(alpha-BQ.alpha0)))*(1 + exp(BQ.M*(alpha+BQ.alpha0))));


CLofalpha = (1-sigma_a)*(BQ.CL0+BQ.CL_alpha*alpha) + sigma_a*(2*sign(alpha)*(sin(alpha)^2)*cos(alpha));
%     CLofalpha = (2*sign(alpha)*(sin(alpha)^2)*cos(alpha));


CL = CLofalpha + BQ.CLq*(q*BQ.c/2*V);
CD = BQ.CD_0 + BQ.k*(CL^2) + 1*((sin(alpha))^2);

omega_w = Rq2w*omega;
p_w = omega_w(1,1);
q_w = omega_w(2,1);
r_w = omega_w(3,1);

CY = BQ.CY_beta*beta + BQ.CY_p*(p_w*BQ.b/(2*V)) + BQ.CY_r*(r_w*BQ.b/(2*V));

L = 0.5*rho*(V^2)*BQ.S*CL ;
D = 0.5*rho*(V^2)*BQ.S*CD ;
Y = 0.5*rho*(V^2)*BQ.S*CY ;

Fa=BQ.wing_n*A*[-D;Y;-L];
Fa(2,1)=-Fa(2,1);
Fa(3,1)=-Fa(3,1);
Fa_w = [L;D;Y];

	