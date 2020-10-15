import pdb
import numpy as np

class kinematic_params_estimator(object):
	def __init__(self,kinematic_params,regressor_and_meas,filter_parameters):
		self.thetacap    = kinematic_params
		n_kinematic      = np.size(kinematic_params)


		self.R           = regressor_and_meas["parameter_jacobian"] # jacobian of end effector velocity with respect to kinematic parameters
		self.x           = regressor_and_meas["end_effector_state"] # end effector state
		n_eff            = np.size(x) 								# depends on whether we consider roll,pitch,yaw, planar arm or 3D arm
		
		self._Rf         = np.zeros((n_eff,n_kinematic))   			# filtered version of jacobian
		self._xf         = np.zeros((n_eff,1)) 			    		# filtered version of end effector state
		
		self._D          = np.zeros((n_kinematic,n_kinematic))    	# needed to compute the parameter update law
		self._I          = np.zeros((n_kinematic,1))               	# needed to compute the parameter update law

		self.l1          = filter_parameters["l1"]  				# low-pass filter gain
		self.lambda_k    = filter_parameters["lambda_k"]			# forgetting factor 
		self.dt          = filter_parameters["dt"]					# sampling time
		self.Gamma       = filter_parameters["Gamma"]				# adaptation gain
		self.tol         = filter_parameters["tol"]					# tolerance to check against zero


	def update_theta_kinematic(self):

		l1        = self.l1											# low-pass filter gain
		lambda_k  = self.lambda_k									# forgetting factor 
		dt        = self.dt 			    						# sampling time
		Gamma     = self.Gamma 										# adaptation gain
		tol       = self.tol 										# tolerance to check against zero

		R         = self.R 											# jacobian of end effector velocity with respect to kinematic parameters
		x         = self.x 											# end effector state

		_Rfdot    = (1/l1)*(R-self._Rf) 					
		_xfdot    = (1/l1)*(x-self._xf)     				

		_Ddot     = -lambda_k*self._D + np.matmul(_Rf.transpose(),_Rf)
		_Idot     = -lambda_k*self._I + np.matmul(_Rf.transpose(),_xfdot)
		_P        = _I - np.matmul(_D,self.thetacap)

		if np.norm(_P)<tol:
			thetacapdot = np.zeros((np_kinematic,1)) 
		else:
			thetacapdot = -Gamma*(_P/np.norm(_P)) 				   # parameter update law, adaptive law


		self._Rf        = self._Rf + dt*_Rfdot 					   
		self._xf        = self._xf + dt*_xfdot 					   
		self._D   		= self._D  + dt*_Ddot
		self._I   		= self._I  + dt*_Idot
		self.thetacap   = self.thetacap + dt*thetacapdot

		return self.thetacap














