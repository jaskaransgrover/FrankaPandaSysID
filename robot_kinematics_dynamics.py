import pdb
import numpy as np
from scipy.linalg import sqrtm


class robot_kinematics_and_dynamics(object):
	def __init__(self,z,kinematic_params,dynamic_params):
		self.l1 = kinematic_params[0]
		self.l2 = kinematic_params[1]
		self.m1 = dynamic_params[0]
		self.m2 = dynamic_params[1]
		q       = z[0:2]
		qdot    = z[2:4]
		self.q  = q
		self.qdot = qdot


	def forward_kinematics(self):
		q  		= self.q
		q1 		= q[0]
		q2 		= q[1]
		l1 		= self.l1
		l2 		= self.l2

		px1 	= l1*np.cos(q1)  
		py1 	= l1*np.sin(q1)  

		px2 	= px1 + l2*np.cos(q1+q2)
		py2 	= py1 + l2*np.sin(q1+q2)

		x1  	= np.array([px1,py1])
		x2  	= np.array([px2,py2])

		return x1,x2


	def angle_jacobian(self):

		q  		= self.q
		q1 		= q[0]
		q2 		= q[1]
		l1 		= self.l1
		l2 		= self.l2

		J 		= np.matrix([[-l2*np.sin(q1+q2) - l1*np.sin(q2),-l2*np.sin(q1+q2)],[l2*np.cos(q1+q2) + l1*np.cos(q1), l2*np.cos(q1+q2)]])

		return J


	def kinematic_parameter_jacobian(self):
		q       = self.q
		qdot  	= self.qdot
		q1    	= q[0]
		q2    	= q[1]
		q1dot 	= qdot[0]
		q2dot 	= qdot[1]

		R 		= np.matrix([[-q1dot*np.sin(q1), -np.sin(q1+q2)*(q1dot + q2dot)],[q1dot*np.cos(q1),  np.cos(q1 + q2)*(q1dot + q2dot)]]);

		return R


	def inertia_matrix(self):
		qdot  		= self.qdot
		q1    		= q[0]
		q2    	    = q[1]
		s1          = np.sin(q1);
		s2          = np.sin(q2);
		c1          = np.cos(q1);
		c2          = np.cos(q2);
		l1 			= self.l1
		l2 			= self.l2
		m1 			= self.m1
		m2 			= self.m2
		r1          = l1/2 ; 
		r2          = l2/2 ; 
		Iz1         = (m1*(l1**2))/12;
		Iz2         = (m2*(l2**2))/12;
		alpha       = (Iz1) + (Iz2) + (m1*r1^2) + (m2*(l1^2  + r2^2));
		beta        = m2*l1*r2;
		delta       = (Iz2) + (m2*(r2**2));
		M           = np.matrix([[(alpha)+(2*beta*c2),(delta)+(beta*c2)],[(delta)+(beta*c2), delta]]);

		return M



	def coriolois_matrix(self):
		qdot  		= self.qdot
		q1    		= q[0]
		q2    		= q[1]
		q1dot 		= qdot[0]
		q2dot 		= qdot[1]
		s1          = np.sin(q1);
		s2          = np.sin(q2);
		c1          = np.cos(q1);
		c2          = np.cos(q2);
		l1 			= self.l1
		l2 			= self.l2
		m1 			= self.m1
		m2 			= self.m2
		r1          = l1/2 ; 
		r2          = l2/2 ; 
		Iz1         = (m1*(l1**2))/12;
		Iz2         = (m2*(l2**2))/12;
		alpha       = (Iz1) + (Iz2) + (m1*r1^2) + (m2*(l1^2  + r2^2));
		beta        = m2*l1*r2;
		delta       = (Iz2) + (m2*(r2**2));
		C           = np.matrix([[-beta*s2*q2dot, -beta*s2*(q11dot+q2dot)],[beta*s2*q1dot, 0]]);
		return C



	def gravity(self):
		return np.zeros(2,1)



	def return_znext(self):
		q       = self.q
		qdot  	= self.qdot

		M           = self.inertia_matrix()
		C           = self.coriolois_matrix()
		vec         = (u - np.matmul(C,qdot))
		qddot       = np.matmul(np.inverse(M),vec)

		znext       = np.zeros((4,1))
		zdot        = np.array([qdot,qddot])
		znext       = np.array([q,qdot]) + dt*zdot

		return znext
		












