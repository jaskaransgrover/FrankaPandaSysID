import pdb
import numpy as np

class KinematicParamsEstimator(object):
	def __init__(self,kinematic_params_initial_est,regressor_and_meas,filter_parameters):

		self.thetacap    = kinematic_params_initial_est
		n_kinematic      = np.size(kinematic_params_initial_est)

		R           	 = regressor_and_meas["parameter_jacobian"] # jacobian of end effector velocity with respect to kinematic parameters
		x           	 = regressor_and_meas["end_effector_state"] # end effector state
		n_eff            = np.size(x) 								# depends on whether we consider roll,pitch,yaw, planar arm or 3D arm
		
		self.Rf         = np.zeros((n_eff,n_kinematic))   			# filtered version of jacobian
		self.xf         = np.zeros((n_eff,)) 			    		# filtered version of end effector state
		
		self.D          = np.zeros((n_kinematic,n_kinematic))    	# needed to compute the parameter update law
		self.I          = np.zeros((n_kinematic,))               	# needed to compute the parameter update law

		self.K           = filter_parameters["K"]  				     # low-pass filter gain
		self.lambda_f    = filter_parameters["lambda_f"]			# forgetting factor 
		self.dt          = filter_parameters["dt"]					# sampling time
		self.Gamma       = filter_parameters["Gamma"]				# adaptation gain
		self.tol         = filter_parameters["tol"]					# tolerance to check against zero

		# self.thetacap0   = self.thetacap
		# self.W           = self.Rf
		# self.Q           = self.W.transpose()@self.W
		# self.C           = self.W.transpose()@x
		# self.eta         = x - self.xf


	def update_theta_kinematic(self,regressor_and_meas):

		K         = self.K											# low-pass filter gain
		lambda_f  = self.lambda_f									# forgetting factor 
		dt        = self.dt 			    						# sampling time
		Gamma     = self.Gamma 										# adaptation gain
		tol       = self.tol 										# tolerance to check against zero

		R         = regressor_and_meas["parameter_jacobian"] 		# jacobian of end effector velocity with respect to kinematic parameters
		x         = regressor_and_meas["end_effector_state"] 		# end effector state
		
		Rfdot    = (1/K)*(R-self.Rf) 					
		xfdot    = (1/K)*(x-self.xf)     				

		Ddot     = (-lambda_f*self.D) + np.matmul(np.transpose(self.Rf),self.Rf)
		Idot     = (-lambda_f*self.I) + np.matmul(np.transpose(self.Rf),xfdot)
		P        = (self.I - np.matmul(self.D,self.thetacap))
		P = P.astype(np.float)

		if np.linalg.norm(P)<tol:
			thetacapdot = np.zeros(np.shape(self.thetacap)) 
		else:
			thetacapdot = np.matmul(Gamma,(P/np.linalg.norm(P))) 				   # parameter update law, adaptive law

		self.Rf         = self.Rf + dt*Rfdot 					   
		self.xf         = self.xf + dt*xfdot 					   
		self.D   		= self.D  + dt*Ddot
		self.I   		= self.I  + dt*Idot
		self.thetacap   = self.thetacap + dt*thetacapdot

		# K         = self.K											# low-pass filter gain
		# lambda_f  = self.lambda_f									# forgetting factor 
		# dt        = self.dt 			    						# sampling time
		# Gamma     = self.Gamma 										# adaptation gain
		# tol       = self.tol 										# tolerance to check against zero

		# R         = regressor_and_meas["parameter_jacobian"] 		# jacobian of end effector velocity with respect to kinematic parameters
		# x         = regressor_and_meas["end_effector_state"] 		# end effector state
		# e         = (x-self.xf)			
		# xfdot     = (R@self.thetacap0) + (K*e) 
		# Wdot      = (-K*self.W) + R
		# etadot    = (-K*self.eta)
		# Qdot      = np.transpose(self.W)@self.W
		# Cdot      = np.transpose(self.W)@((self.W@self.thetacap0) + e - self.eta)

		# thetacapdot = Gamma@(self.C - (self.Q@self.thetacap))


		# self.xf         = self.xf  + dt*xfdot 					   
		# self.W          = self.W    + dt*Wdot 					   
		# self.eta   		= self.eta  + dt*etadot
		# self.Q   		= self.Q    + dt*Qdot
		# self.C   		= self.C    + dt*Cdot
		# self.thetacap   = self.thetacap + dt*thetacapdot
		

		return self.thetacap














