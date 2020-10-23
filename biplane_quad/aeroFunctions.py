from globals import systemParameters
import numpy as np
from numpy import sin as sin
from numpy import cos as cos
from numpy import exp as exp
from numpy import arctan2 as atan2

from utils import eul2rotm

def momentEstimate(eul, x_dot, omega, Fa):
	BQ = systemParameters()
	rho=1.225
	Moment_aero=(np.zeros(3)).T
	R=np.array(eul2rotm(eul))
	x_dot = np.array(x_dot).T
	# print('test momEST',R, x_dot)
	xb_dot=np.dot(R.T,x_dot);
	V=np.linalg.norm(x_dot);

	if (V==0):
		return

	Rq2w = np.array(
		[[0, 0, 1],
	    [0, 1, 0],
	    [-1, 0, 0]])
	xw_dot = np.dot(Rq2w, xb_dot)

	if (xw_dot[0]!=0):
	    alpha=np.arctan2(-xw_dot[2],xw_dot[0])
	    beta=np.arctan2(xw_dot[1],xw_dot[0])
	else:
	    alpha=0
	    beta=0	

	omega_w = np.dot(Rq2w,omega)
	p_w = omega_w[0,0]
	q_w = omega_w[1,0]
	r_w = omega_w[2,0]

	C_l = BQ.Cl_beta*beta + BQ.Cl_p*p_w*BQ.b/(2*V) + BQ.Cl_r*r_w*BQ.b/(2*V);
	C_m = BQ.Cm_0 + BQ.Cm_alpha*alpha + BQ.Cm_q*q_w*BQ.c/(2*V);
	C_n = BQ.Cn_beta*beta + BQ.Cn_p*p_w*BQ.b/(2*V) + BQ.Cn_r*r_w*BQ.b/(2*V);

	Mx_w_ac = 0.5*rho*(V**2)*BQ.S*BQ.c*C_l ;
	My_w_ac = 0.5*rho*(V**2)*BQ.S*BQ.c*C_m ;
	Mz_w_ac = 0.5*rho*(V**2)*BQ.S*BQ.c*C_n ;

	M_ac = np.array([
		Mx_w_ac,
		My_w_ac,
		Mz_w_ac])#;% wing frame
	# % r = BQ.x_ac - BQ.x_cg;
	# % M_cg = M_ac + cross(r',(Rq2w*Fa)')';
	r = BQ.x_ac[0] - BQ.x_cg[0];
	# % L = -Fa(1);
	# % M_cg = M_ac + [0;r*L;0]; % wing frame

	# % Moment_aero = BQ.wing_n*Rq2w'*M_cg; % body frame
	Moment_aero = np.array([0,BQ.wing_n*My_w_ac+r*Fa[0],0]).T


def forceEstimate(eul,x_dot,omega):
	print("x_dot", x_dot)
	BQ = systemParameters();
	Fa=np.array([0,0,0])
	Fa_w=np.array([0,0,0])
	alpha=0
	beta=0
	p=omega[0,0]
	q=omega[1,0]
	r=omega[2,0]
	rho=1.225
	R=np.array(eul2rotm(eul))
	xb_dot=np.dot(R.T,x_dot)
	V=np.linalg.norm(x_dot)
	if(np.isnan(V)):
		raise ValueError("nan arrives")
    ######## v==0 

	Rq2w = np.array([
	    [0, 0, 1],
	    [0, 1, 0],
	    [-1, 0, 0]])
	xw_dot = np.dot(Rq2w,xb_dot)

	#alpha=atan2(-xw_dot(3),xw_dot(1));
	#beta=atan2(xw_dot(2),xw_dot(1));

	if (xw_dot[0]!=0):
	    alpha=atan2(-xw_dot[2],xw_dot[0])
	    beta=atan2(xw_dot[1],xw_dot[0])
	else:
	    alpha=0;
	    beta=0;
    # alpha=atan2(xb_dot(1),xb_dot(3));
	# beta=atan2(xb_dot(2),xb_dot(3));

    # alpha = abs(alpha);
    # beta = abs(beta);

	A = np.array([[sin(alpha)*cos(beta),   -sin(alpha)*sin(beta), cos(alpha)],
               [sin(beta)           , cos(beta)                ,0       ],
	           [-cos(alpha)*cos(beta),   cos(alpha)*sin(beta),     sin(alpha)]])

	sigma_a = (1 + exp(-BQ.M*(alpha-BQ.alpha0)) + exp(BQ.M*(alpha+BQ.alpha0)))/(( 1 + exp(-BQ.M*(alpha-BQ.alpha0)))*(1 + exp(BQ.M*(alpha+BQ.alpha0))))


	CLofalpha = (1-sigma_a)*(BQ.CL0+BQ.CL_alpha*alpha) + sigma_a*(2*sin(alpha)*(sin(alpha)**2)*cos(alpha));
    #CLofalpha = (2*sign(alpha)*(sin(alpha)^2)*cos(alpha));


	CL = CLofalpha + BQ.CLq*(q*BQ.c/2*V);
	CD = BQ.CD_0 + BQ.k*(CL**2) + 1*((sin(alpha))**2);
	print(V)
	omega_w = Rq2w*omega
	p_w = omega_w[0,0]
	q_w = omega_w[1,0]
	r_w = omega_w[2,0]
	print(omega_w, p_w, q_w, r_w)
	CY = BQ.CY_beta*beta + BQ.CY_p*(p_w*BQ.b/(2*V)) + BQ.CY_r*(r_w*BQ.b/(2*V));

	L = 0.5*rho*(V**2)*BQ.S*CL 
	D = 0.5*rho*(V**2)*BQ.S*CD 
	Y = 0.5*rho*(V**2)*BQ.S*CY 

	#print("Fahere", Fa)
	#print("A", A)
	Fa=BQ.wing_n*A.dot(np.array([-D,Y,-L]))
	#print("Fahere", Fa)
	Fa[1]=-Fa[1]
	Fa[2]=-Fa[2]
	Fa_w = np.array([L,D,Y])

	return Fa, Fa_w, alpha, beta