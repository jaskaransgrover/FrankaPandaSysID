import numpy as np

for i in range(nlink):
	omega[:,i]=np.matmul(R,omega[:,i-1]) + np.matmul(z[:,i],thetadot[i])
	omegadot[:,i]=np.matmul(R,omegadot[:,i-1]) + np.cross(np.matmul(R,omega[:,i-1]),np.matmul(z[:,i],thetadot[i])) + np.matmul(z_i,thetaddot[i])
	term      = vdot[:,i-1] + np.cross(omegadot[:,i-1],P) + np.cross(omega[:,i-1],np.cross(omega[:,i-1],P))
	vdot[:,i] = np.matmul(R,term)
	fci[:,i]  = m[I]*xi[:,i] + np.cross(omegadot[:,i],mhi[:,i]) + np.cross(omega[:,i],np.cross(omega))