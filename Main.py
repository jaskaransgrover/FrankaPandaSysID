import numpy as np
import time
import math
from robot_kinematics_dynamics import *
from kinematic_params_estimator import *


PI           = math.pi

n_kinematic  = 2
n_dynamic    = 2

Gamma_d      = np.eye(n_dynamic)

K            = 0.01
lambda_f     = 0.015
Gamma_k      = 0.025*np.eye(n_kinematic)
T_final      = 100.0
Ntime        = 100000
Tspan        = np.linspace(0,T_final,Ntime)
dt           = Tspan[2]-Tspan[1]
tolerance    = 0.00001

l1           = 1
l2           = 0.5
m1           = 1 
m2           = 1 



filter_parameters       = {"K":K,"lambda_f":lambda_f,"dt":dt,"Gamma":Gamma_k,"tol":tolerance}
kinematic_parameters    = np.zeros((n_kinematic,))
kinematic_parameters[0] = l1
kinematic_parameters[1] = l2
dynamic_parameters      = np.zeros((n_dynamic,))
dynamic_parameters[0]   = m1
dynamic_parameters[1]   = m2


t            = Tspan[0]
a1           = 2;
a2           = 1
b1           = 1
b2           = 0.5
c1           = 0
c2           = PI/4
q1           = a1*np.sin(2*PI*t*b1 + c1)
q2           = a2*np.sin(2*PI*t*b2 + c2)
q1dot        = (2*PI*a1*b1)*np.cos(2*PI*t*b1 + c1)
q2dot        = (2*PI*a2*b2)*np.cos(2*PI*t*b2 + c2)
z            = np.array([q1,q2,q1dot,q2dot])

R            = RobotKinematicsDynamics(z,kinematic_parameters,dynamic_parameters)


kinematic_parameters_initial_est = kinematic_parameters + np.random.rand(2,)
x2                               = R.forward_kinematics(z)
RPJ                              = R.kinematic_parameter_jacobian(z)
filter_parameters                = {"K":K,"lambda_f":lambda_f,"dt":dt,"Gamma":Gamma_k,"tol":tolerance}
regressor_and_meas               = {"parameter_jacobian":RPJ,"end_effector_state":x2}


E            = KinematicParamsEstimator(kinematic_parameters_initial_est,regressor_and_meas,filter_parameters)
sdes         = np.array([0.7,0.7])




if __name__ == "__main__":
    for i in range(Ntime):
        t      = Tspan[i]
        q1     = a1*np.sin(2*PI*t*b1 + c1)
        q2     = a2*np.sin(2*PI*t*b2 + c2)
        q1dot  = (2*PI*a1*b1)*np.cos(2*PI*t*b1 + c1)
        q2dot  = (2*PI*a2*b2)*np.cos(2*PI*t*b2 + c2)
        z      = np.array([q1,q2,q1dot,q2dot])

        x2     = R.forward_kinematics(z)
        RPJ    = R.kinematic_parameter_jacobian(z)

        regressor_and_meas = {"parameter_jacobian":RPJ,"end_effector_state":x2}
        thetacap           = E.update_theta_kinematic(regressor_and_meas)
        print(thetacap)

        # x2     = R.forward_kinematics(z)
        # J      = R.angle_jacobian(z)
        # q      = z[0:2]
        # qdot   = z[2:4]
        # x2dot  = J@qdot
        # Jdot   = R.jacobian_dot(z)
        # M,C    = R.inertia_and_coriolis(z)
        # kp     = 3 
        # kv     = 4 
        # stuff1 = (-kp*(x2 - sdes))-(kv*x2dot)
        # stuff2 = stuff1 - (Jdot@qdot)
        # H      = J@np.linalg.inv(M)
        # stuff3 = (np.linalg.inv(H))@(stuff2)
        # u      = stuff3 + (C@qdot)
        # zdot   = R.return_zdot(u,z)
        # z      = z + dt*zdot
        
