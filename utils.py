import numpy as np

class Qd:
    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.rot = np.zeros(3)
        self.omega = np.zeros(3)


traj_=[0,2.38979695502618,4.77382734355769,7.14528763663408,9.48490559304162,11.7624254933948,13.9512841836096,16.0345450467868,18.0067099114244,19.8700348284321,21.6287988676316,23.2857311047559,24.8402723340366,26.2877458941240,27.6193335556213,28.8228663097017,29.8847503912196,30.7933647838816,31.5441847624783,32.1418301282528,32.5984934225259,32.9319963157787,33.1636946944331,33.3161073635660,33.4096158103627,33.4613806883770,33.4856560949199,33.4943969621540,33.4965072850043,33.4968046882397,33.4968470135881]
traj_dot_=[20,19.9724889788625,19.9054338738792,19.7416622069201,19.3548827835231,18.7119196613496,17.8832905898648,16.9620656312847,16.0357984279122,15.1437658811142,14.2843353051331,13.4356129152420,12.5655528323586,11.6384523427997,10.6218661694046,9.49406078577192,8.25382692926901,6.94033271619121,5.62765043956652,4.38811966456048,3.27694328802304,2.33256649079139,1.57575681807551,1.00240998170344,0.585901796834056,0.299863510269067,0.123040818227709,0.0357850765749011,0.00610735459558673,0.000856677804326638,0]

def eul2rotm(eulerAngles):
	phi = eulerAngles[0]
	theta = eulerAngles[1]
	psi = eulerAngles[2]

	rotMatrix = [[np.cos(theta)*np.cos(psi),-np.cos(phi)*np.sin(psi)+np.sin(phi)*np.sin(theta)*np.cos(psi), np.sin(phi)*np.sin(psi)+np.cos(phi)*np.sin(theta)*np.cos(psi)],
		        [np.cos(theta)*np.sin(psi), np.cos(phi)*np.cos(psi)+np.sin(phi)*np.sin(theta)*np.sin(psi), -np.sin(phi)*np.cos(psi)+np.cos(phi)*np.sin(theta)*np.sin(psi)],
		        [-np.sin(theta,             np.sin(phi)*np.cos(theta),                                     np.cos(phi)*np.cos(theta)]]

    return rotMatrix

def traj_gen(traj=traj_, traj_dot_, flag):
	n=4
	N=31

	if(flag == 4 or flag == 5):
		Tf = 3.58
	elif (flag == 2 or flag == 3):
		Tf = 3.1

	f = np.zeros(30*n, 1)
	for i in range(n):
		t0 = (i-1)*Tf/(N-1)
		tf = i*Tf/(N-1)
		M = [[1, t0, t0**2, t0**3],
			 [1, tf, tf**2, tf**3],
			 [0, 1,  2*t0,  3*t0**2],
			 [0, 1,  2*tf,  3*tf**2]]
		b = traj[[i, i+1, i, i+1]]
		A = np.linalg.solve(M,b)	
return  f((1:n) + n*(i-1))=A;

def stateToQd(x):
	qd = Qd()
	
	qd.pos = x[0:3]
	qd.vel = x[3:6]
	qd.rot = x[6:9]
	qd.omega = x[9:12]
	return qd
