import numpy as np
import time
import math
import robot_kinematics_dynamics


PI           = math.pi

n_kinematic  = 2
n_dynamic    = 2

Gamma_d      = np.eye(n_dynamic)

K            = 10
lambda_f     = 1
dt           = 0.01
Gamma_k      = np.eye(n_kinematic)
tolerance    = 0.001

l1           = 1
l2           = 0.5
m1           = 2 
m2           = 3 


filter_parameters       = {"K":K,"lambda_f":lambda_f,"dt":dt,"Gamma":Gamma_k,"tol":tolerance}
kinematic_parameters    = np.zeros((n_kinematic,1))
kinematic_parameters[0] = l1
kinematic_parameters[1] = l2

T_final      = 10
Tspan        = np.linspace(0,T_final,T_final/dt)

a1           = 0.5
a2           = 0.4
b1           = 0.8
b2           = 0.2
c1           = 0
c2           = PI/4


if __name__ == "__main__":

    for t in Tspan:
        q1  = a1*np.sin(2*PI*t*b1 + c1)
        q2  = a2*np.sin(2*PI*t*b2 + c2)

        q1dot  = (2*PI*b1*a1)*np.cos(2*PI*t*b1 + c1)
        q2dot  = (2*PI*b2*a2)*np.cos(2*PI*t*b2 + c2)

        q   = np.array([q1,q2])
        qdot= np.array([q1dot,q2dot])

