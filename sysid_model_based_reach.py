import os
import pybullet as p
import pybullet_data
import math
import numpy as np
import time
import matplotlib.pyplot as plt
from numpy import pi
from numpy import cos, sin



DH = np.array([[0,   0.333,   0, -pi/2],
                     [0,   0,   0,   pi/2],
                     [0,   0.316,   0.088,   pi/2],
                     [0,   0,   -0.088,  -pi/2],
                     [0,   0.384,   0,   pi/2],
                     [0,   0,   0.088,   pi/2],
                     [0,   0.227,   0,   0]])

DH = np.array([[0,   0.333,   0, 0],
                     [0,   0,   0,   -pi/2],
                     [0,   0.316,   0,   pi/2],
                     [0,   0,   0.0825,  pi/2],
                     [0,   0.384,   -0.0825,   -pi/2],
                     [0,   0,   0,   pi/2],
                     [0,   0,   0.088,   pi/2]])

# def forward_kinematics(theta, DH, base):
#     '''
#     the forward kinematics
#     '''
    
#     nlink = DH.shape[0]
#     M = []
#     # pos = []
#     M.append(np.identity(4))

#     for i in range(nlink):
#         R = np.array([[cos(DH[i][0]), -sin(DH[i][0]) * cos(DH[i][3]), sin(DH[i][0]) * sin(DH[i][3])],
#                       [sin(DH[i][0]), cos(DH[i][0]) * cos(DH[i][3]), -cos(DH[i][0])*sin(DH[i][3])],
#                       [0, sin(DH[i][3]), cos(DH[i][3])]])

#         T = np.array([[DH[i][2] * cos(DH[i][0])],
#                       [DH[i][2] * sin(DH[i][0])],
#                       [DH[i][1]]])

#         RT = np.block([[R,                T],
#                        [np.zeros((1, 3)), 1]])

#         M_tmp = np.matmul(M[i], RT)
#         M.append(M_tmp)

#     endpos = M_tmp[0:3,3] 
#     return endpos


def forward_kinematics(theta, DH, base):
    '''
    the forward kinematics
    '''
    
    nlink = DH.shape[0]
    M = []
    # pos = []
    M.append(np.identity(4))

    for i in range(nlink):
        R = np.array([[cos(DH[i][0]), -sin(DH[i][0]) , 0],
                      [sin(DH[i][0])* cos(DH[i][3]), cos(DH[i][0])* cos(DH[i][3]) , -sin(DH[i][3])],
                      [sin(DH[i][0])* sin(DH[i][3]), cos(DH[i][0])* sin(DH[i][3]), cos(DH[i][3])]])

        T = np.array([[DH[i][2]],
                      [-DH[i][1] * sin(DH[i][3])],
                      [DH[i][1] * cos(DH[i][3])]])

        RT = np.block([[R,                T],
                       [np.zeros((1, 3)), 1]])

        M_tmp = np.matmul(M[i], RT)
        M.append(M_tmp)

    endpos = M_tmp[0:3,3] 
    return endpos

if __name__ == "__main__":
    p.connect(p.GUI)
    urdfRootPath=pybullet_data.getDataPath()
    pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"),useFixedBase=True)
    # tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.0,0,0])
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])

    Ntime         = 400
    T_final       = 2
    Tspan         = np.linspace(0,T_final,Ntime)
    dt            = Tspan[2]-Tspan[1]
    t             = Tspan[0]
    p.setTimestep = dt


    PI            = math.pi
    a             = np.array([PI/3,PI/2,PI/2,PI/5,0,PI/4,PI/2])
    b             = np.array([1,1,1,1,1,1,1])
    c             = np.zeros(7,)
    d             = np.array([0,0,0,-PI/3,0,PI/2,0])
    q             = np.zeros(7,)
    Q_c           = np.zeros((7,Ntime))
    Q_a           = np.zeros((7,Ntime))

    P_expected    = np.zeros((3,Ntime))
    P_get         = np.zeros((3,Ntime))

    for m in range(7):
        q[m]     = (a[m]*np.sin(2*PI*0*b[m] + c[m])) + d[m]
        p.setJointMotorControl2(pandaUid, m, p.POSITION_CONTROL,q[m])
    p.stepSimulation()

    for i in range(1,Ntime):
        t = Tspan[i]
        time.sleep(dt)
        for m in range(7):
            q[m]     = (a[m]*np.sin(2*PI*t*b[m] + c[m])) + d[m]
            Q_c[m,i] = q[m]
            p.setJointMotorControl2(pandaUid, m, p.POSITION_CONTROL,q[m])

        
        p.stepSimulation()
        for m in range(7):
            angle_vel = p.getJointState(pandaUid,m)
            Q_a[m,i]  = angle_vel[0]


        angs = Q_a[:,i]
        DH[:,0] = np.squeeze(angs)
        cartesian_goal = forward_kinematics(np.squeeze(angs),DH, np.array([0,0,0]))
        print("Goal from my calculation") 
        print(cartesian_goal)
        print("\n")
        pos = p.getLinkState(pandaUid, 5)
        print("Goal from my FM") 
        print(pos[0])

        for k in range(3):
            P_expected[k,i]=cartesian_goal[k]
            P_get[k,i]=pos[0][k]

        
        


    # fig = plt.figure()
    # fig.subplots_adjust(hspace=0.4, wspace=0.4)

    # for i in range(1,8):
    #     print(i)
    #     x = Tspan
    #     y1 = Q_c[i-1,:]
    #     y2 = Q_a[i-1,:]
    #     ax = fig.add_subplot(2, 4, i)
    #     line1=ax.plot(x, y1, color='k')
    #     line2=ax.plot(x, y2, color='r')
    #     plt.xlabel('Time (s)')
    #     num_joint = str(i) 
    #     s1 = "Joint "
    #     s2 = s1 + num_joint

    #     plt.ylabel(s2)
    #     # plt.legend((line1, line2), ('Commanded', 'Measured'))

    # mng = plt.get_current_fig_manager()

    # plt.show()
    # print("jaskaran")

    fig = plt.figure()
    fig.subplots_adjust(hspace=0.4, wspace=0.4)

    for i in range(1,4):
        print(i)
        x = Tspan
        y1 = P_expected[i-1,:]
        y2 = P_get[i-1,:]
        ax = fig.add_subplot(1, 3, i)
        line1=ax.plot(x, y1, color='k')
        line2=ax.plot(x, y2, color='r',linestyle='dashed')
        plt.xlabel('Time (s)')
        num_joint = str(i) 
        s1 = "Joint "
        s2 = s1 + num_joint

        plt.ylabel(s2)
        # plt.legend((line1, line2), ('Commanded', 'Measured'))

    mng = plt.get_current_fig_manager()

    plt.show()
    print("jaskaran")




