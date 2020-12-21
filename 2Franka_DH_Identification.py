import os
import numpy as np
import sympy 
from sympy import *
import time
import math
from kinematic_params_estimator import *
import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt
from numpy import pi


e            = 0.00000001

DHtrue = np.array([  [0,   0.333,   0, 0],
                     [0,   0,   0,   -pi/2],
                     [0,   0.316,   0,   pi/2],
                     [0,   0,   0.0825,  pi/2],
                     [0,   0.384,   -0.0825,   -pi/2],
                     [0,   0,   0,   pi/2],
                     [0,   0,   0.088,   pi/2]])

def round2zero(m, e):
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            if (isinstance(m[i,j], Float) and np.absolute(m[i,j]) < e):
                m[i,j] = 0
    return m

def round2zerof(m, e):
    if (isinstance(m, Float) and np.absolute(m) < e):
                m = 0
    return m

def forward_kinematics(q, DH1, base):
    '''
    the forward kinematics

    '''
    DH1[:,0]=q
    nlink = DH1.shape[0]
    M = []
    # pos = []
    M.append(np.identity(4))

    for i in range(nlink):
        e  =  0.0000001
        ct = round2zerof(cos(DH1[i][0]),e)
        st = round2zerof(sin(DH1[i][0]),e)
        ca = round2zerof(cos(DH1[i][3]),e)
        sa = round2zerof(sin(DH1[i][3]),e)

        R = np.array([[ct, -st , 0.0],
                      [st* ca, ct*ca , -sa],
                      [st* sa, ct*sa , ca]])

        T = np.array([[DH1[i][2]],
                      [-DH1[i][1] * sa],
                      [DH1[i][1] * ca]])
        T = round2zero(T,e)


        RT = np.block([[R,                T],
                       [np.zeros((1, 3)), 1]])

        M_tmp = np.matmul(M[i], RT)
        M.append(M_tmp)

    endpos = M_tmp[0:3,3] 
    return endpos


PI           = math.pi
n_kinematic  = 14
K            = 0.1
lambda_f     = 0.5
Gamma_k      = 2*np.eye(n_kinematic)
dtdes        = 0.01
T_final      = 10
Ntime        = int(T_final/dtdes)
Tspan        = np.linspace(0,T_final,Ntime)
dt           = Tspan[2]-Tspan[1]
t0           = Tspan[0]
tolerance    = 0.00001

# p.connect(p.GUI)
# urdfRootPath=pybullet_data.getDataPath()
# pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True)
# # tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.0,0,0])
# p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])
# p.setTimestep = dt


PI            = math.pi
Amp           = np.array([PI/3,PI/4,PI,PI/5,0,PI/4,PI/2])
b             = 2*np.ones(7,)
c             = np.zeros(7,)
Offset        = np.array([0,0,0,-PI/3,0,PI/2,0])
q             = np.zeros(7,)
qd            = np.zeros(7,)
Q_c           = np.zeros((7,Ntime))

P_expected    = np.zeros((3,Ntime))
P_get         = np.zeros((3,Ntime))

for m in range(7):
    q[m]     = (Amp[m]*np.sin(2*PI*t0*b[m] + c[m])) + Offset[m]
    # p.setJointMotorControl2(pandaUid, m, p.POSITION_CONTROL,q[m])
# p.stepSimulation()

# pos    = p.getLinkState(pandaUid, 5)
x20    = np.zeros((3,1))
RPJ0   = np.zeros((3,8))
regressor_and_meas      = {"parameter_jacobian":RPJ0,"end_effector_state":x20}


filter_parameters       = {"K":K,"lambda_f":lambda_f,"dt":dt,"Gamma":Gamma_k,"tol":tolerance}


dtrue = DHtrue[:,1]
atrue = DHtrue[:,2]
thetatrue = np.append(atrue,dtrue)
kinematic_parameters    =thetatrue
kinematic_parameters_initial_est = kinematic_parameters + np.random.rand(n_kinematic,)

E            = KinematicParamsEstimator(kinematic_parameters_initial_est,regressor_and_meas,filter_parameters)


if __name__ == "__main__":


    q0 = sympy.Symbol('q0')
    q1 = sympy.Symbol('q1')
    q2 = sympy.Symbol('q2')
    q3 = sympy.Symbol('q3')
    q4 = sympy.Symbol('q4')
    q5 = sympy.Symbol('q5')
    q6 = sympy.Symbol('q6')

    q0_dot = sympy.Symbol('q0_dot')
    q1_dot = sympy.Symbol('q1_dot')
    q2_dot = sympy.Symbol('q2_dot')
    q3_dot = sympy.Symbol('q3_dot')
    q4_dot = sympy.Symbol('q4_dot')
    q5_dot = sympy.Symbol('q5_dot')
    q6_dot = sympy.Symbol('q6_dot')

    a0 = sympy.Symbol('a0')
    a1 = sympy.Symbol('a1')
    a2 = sympy.Symbol('a2')
    a3 = sympy.Symbol('a3')
    a4 = sympy.Symbol('a4')
    a5 = sympy.Symbol('a5')
    a6 = sympy.Symbol('a6')

    d0 = sympy.Symbol('d0')
    d1 = sympy.Symbol('d1')
    d2 = sympy.Symbol('d2')
    d3 = sympy.Symbol('d3')
    d4 = sympy.Symbol('d4')
    d5 = sympy.Symbol('d5')
    d6 = sympy.Symbol('d6')


    DH = np.array([[0,   d0,   a0, 0],
                   [0,   d1,   a1,   -pi/2],
                   [0,   d2,   a2,   pi/2],
                   [0,   d3,   a3,  pi/2],
                   [0,   d4,   a4,   -pi/2],
                   [0,   d5,   a5,   pi/2],
                   [0,   d6,   a6,   pi/2]])

    base = np.array([0,0,0]) 
    q    = np.array([q0,q1,q2,q3,q4,q5,q6])
    qdot = np.array([q0_dot,q1_dot,q2_dot,q3_dot,q4_dot,q5_dot,q6_dot])
    d    = np.array([d0,d1,d2,d3,d4,d5,d6])
    a    = np.array([a0,a1,a2,a3,a4,a5,a6])
    theta = np.append(a,d)


    epo  = forward_kinematics(q, DH, base)
    epo  = Matrix(epo)
    J    = epo.jacobian(q)
    Q1   = J@qdot
    Q1   = Matrix(Q1)
    R    = Q1.jacobian(theta)
    Rnum = lambdify([q,qdot], R , modules='numpy')

    dtrue = DHtrue[:,1]
    atrue = DHtrue[:,2]
    thetatrue = np.append(atrue,dtrue)
    print("thetatrue")
    print(thetatrue)


    for i in range(Ntime):
        t      = Tspan[i]
        print(t)
        for m in range(7):
            q[m]     = (Amp[m]*np.sin(2*PI*t*b[m] + c[m])) + Offset[m]
            qd[m]    = (2*PI*Amp[m]*b[m]*np.cos(2*PI*t*b[m] + c[m]))
            

        angs = q
        DHtrue[:,0]    = np.squeeze(angs)
        cartesian_goal = forward_kinematics(np.squeeze(angs),DHtrue, np.array([0,0,0]))
        x2             = cartesian_goal
        RPJ            = Rnum(q,qd)

        regressor_and_meas = {"parameter_jacobian":RPJ,"end_effector_state":x2}
        thetacap           = E.update_theta_kinematic(regressor_and_meas)
        print(thetacap)

    dtrue = DHtrue[:,1]
    atrue = DHtrue[:,2]
    thetatrue = np.append(atrue,dtrue)
    print("thetatrue")
    print(thetatrue)
    print("thetaest")
    print(thetacap)



       
        
