import sympy 
from sympy import *
import numpy as np
from numpy import pi

e = 0.00000000001;
nlink = 7

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

def forward_kinematics(theta, DH, base):
    '''
    the forward kinematics

    '''
    DH[:,0]=theta
    nlink = DH.shape[0]
    M = []
    # pos = []
    M.append(np.identity(4))

    for i in range(nlink):
        ct = round2zerof(cos(DH[i][0]),e)
        st = round2zerof(sin(DH[i][0]),e)
        ca = round2zerof(cos(DH[i][3]),e)
        sa = round2zerof(sin(DH[i][3]),e)

        R = np.array([[ct, -st , 0],
                      [st* ca, ct*ca , -sa],
                      [st* sa, ct*sa , ca]])

        T = np.array([[DH[i][2]],
                      [-DH[i][1] * sa],
                      [DH[i][1] * ca]])
        T = round2zero(T,e)

        RT = np.block([[R,                T],
                       [np.zeros((1, 3)), 1]])

        M_tmp = np.matmul(M[i], RT)
        M.append(M_tmp)

    endpos = M_tmp[0:3,3] 
    return endpos


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

  # alpha0 = sympy.Symbol('alpha0')
  # alpha1 = sympy.Symbol('alpha1')
  # alpha2 = sympy.Symbol('alpha2')
  # alpha3 = sympy.Symbol('alpha3')
  # alpha4 = sympy.Symbol('alpha4')
  # alpha5 = sympy.Symbol('alpha5')
  # alpha6 = sympy.Symbol('alpha6')

  d0 = sympy.Symbol('d0')
  d1 = sympy.Symbol('d1')
  d2 = sympy.Symbol('d2')
  d3 = sympy.Symbol('d3')
  d4 = sympy.Symbol('d4')
  d5 = sympy.Symbol('d5')
  d6 = sympy.Symbol('d6')

  xb = sympy.Symbol('xb')
  yb = sympy.Symbol('yb')
  zb = sympy.Symbol('zb')

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
  # R    = simplify(R)
  Qnum = lambdify([theta,q,qdot], Q1 , modules='numpy')
  Rnum = lambdify([q,qdot], R , modules='numpy')
  
  
  q_conf_0  = np.array([pi/3,pi/4,pi/3,pi/3,pi/3,pi,pi/4])
  qd_conf_0 = np.array([6,6,4,0.0,1,2,0])
  d_conf_0  = np.array([1,2,3,4,5,6,7])
  a_conf_0  = np.array([3,5,2,1,7,4,0])
  theta_conf_0 = np.append(a_conf_0,d_conf_0)


  print(Qnum(theta_conf_0,q_conf_0,qd_conf_0))
  print(Rnum(q_conf_0,qd_conf_0).shape)
  

  # h1 = Qnum(theta_conf_0,q_conf_0,qd_conf_0)

  # h2 = Rnum(q_conf_0,qd_conf_0)@theta_conf_0

  # print(h1)

  # print(h2)
  # h2 = h2.reshape((3,1))
  # print(h1-h2)



  # R    = epo.jacobian(d)
  # R    =  simplify(R)
  # print(R)
  # t1        = epo

  
  # t2        = np.matmul(R,d)
  # t2        = t2.reshape((3,1))
  # print(t2.shape)
  # remainder = t1-t2
  # remainder = simplify(remainder)
  # print(remainder)
  # g_func = lambdify(theta, R, modules='numpy')
  # 
  # 
  # k = np.asarray(k)
  # print(k.shape)