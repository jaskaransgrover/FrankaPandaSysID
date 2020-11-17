import sympy 
from sympy import *
import numpy as np
from numpy import pi

def forward_kinematics(theta, DH, base):
    '''
    the forward kinematics
    '''

    nlink = 7
    M = []
    # pos = []
    M.append(np.identity(4))

    for i in range(nlink):
        R = np.array([[cos(theta[i]), -sin(theta[i]) * cos(DH[i][3]), sin(theta[i]) * sin(DH[i][3])],
                      [sin(theta[i]), cos(theta[i]) * cos(DH[i][3]), -cos(theta[i])*sin(DH[i][3])],
                      [0, sin(DH[i][3]), cos(DH[i][3])]])

        T = np.array([[DH[i][2] * cos(DH[i][0])],
                      [DH[i][2] * sin(DH[i][0])],
                      [DH[i][1]]])

        RT = np.block([[R,                T],
                       [np.zeros((1, 3)), 1]])

        M_tmp = np.matmul(M[i], RT)
        M.append(M_tmp)

    endpos = np.matmul(M_tmp[0:3,0:3], np.zeros(3)) + M_tmp[0:3,3] + np.squeeze(base)
    return endpos


if __name__ == "__main__":

  DH = np.array([[0,   0.333,   0, -pi/2],
                     [0,   0,   0,   pi/2],
                     [0,   0.316,   0.088,   pi/2],
                     [0,   0,   -0.088,  -pi/2],
                     [0,   0.384,   0,   pi/2],
                     [0,   0,   0.088,   pi/2],
                     [0,   0.227,   0,   0]])
  base = np.array([[0.5],[0],[-0.65]]) 

  q0 = sympy.Symbol('q0')
  q1 = sympy.Symbol('q1')
  q2 = sympy.Symbol('q2')
  q3 = sympy.Symbol('q3')
  q4 = sympy.Symbol('q4')
  q5 = sympy.Symbol('q5')
  q6 = sympy.Symbol('q6')

  theta = np.array([q0,q1,q2,q3,q4,q5,q6])

  epo  = forward_kinematics(theta, DH, base)
  epo  = Matrix(epo)
  R    = epo.jacobian(theta)