import pdb
import numpy as np

class dynamic_params_estimator(object):
	def __init__(self,dynamic_params,regressor_and_meas,filter_parameters):

		self.thetacap    = dynamic_params
		n_dynamic        = np.size(dynamic_params)

		self.D           = regressor_and_meas["parameter_jacobian"] # jacobian of end effector velocity with respect to kinematic parameters
		self.tau         = regressor_and_meas["end_effector_state"] # end effector state
		n_eff            = np.size(tau) 								# depends on whether we consider roll,pitch,yaw, planar arm or 3D arm
		
		self._Rf         = np.zeros((n_eff,n_dynamic))   			# filtered version of jacobian
		self._xf         = np.zeros((n_eff,1)) 			    		# filtered version of end effector state
		
		self._U          = np.zeros((n_dynamic,n_dynamic))    		# needed to compute the parameter update law
		self._B          = np.zeros((n_dynamic,1))               	# needed to compute the parameter update law

		self.delta       = filter_parameters["delta"]				# forgetting factor 
		self.dt          = filter_parameters["dt"]					# sampling time
		self.Gamma       = filter_parameters["Gamma"]				# adaptation gain
		self.tol         = filter_parameters["tol"]					# tolerance to check against zero


	def update_theta_dynamic(self):

		
		delta     = self.delta   									# forgetting factor 
		dt        = self.dt 			    						# sampling time
		Gamma     = self.Gamma 										# adaptation gain
		tol       = self.tol 										# tolerance to check against zero

		S         = self.S 											# jacobian of end effector velocity with respect to kinematic parameters
		tau       = self.tau									    # end effector state

		_Udot     = -self.delta*_U + np.matmul(S.transpose(),S)			
		_Bdot     = -self.delta*_B + np.matmul(S.transpose(),tau)
		_E        = np.matmul(_U,self.thetacap)-_B

		if np.norm(_E)<tol:
			thetacapdot = 0*self.thetacap
		else:
			thetacapdot = -Gamma*(np.matmul(np.transpose(_U),_E)/np.norm(_E)) 				   # parameter update law, adaptive law

		self._U        = self._U + dt*_Udot 					   
		self._B        = self._B + dt*_Bdot 					   
		self.thetacap  = self.thetacap + dt*thetacapdot

		return self.thetacap














