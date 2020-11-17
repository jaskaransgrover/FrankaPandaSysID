import numpy as np
from numpy import cos, sin
from math import inf


def forward_kinematic_map(q):
  # a = np.array([0.0,0.0,0.0,0.0825,-0.0825,0.0,0.088,0.0])
  # d = np.array([0.333,0.0,0.316,0.0,0.384,0.0,0.0,0.107])
  # alpha = np.array([0.0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2,0.0])
  # nlink = 7
  # DH = np.zeros((nlink,4))
  # for i in range(nlink):
  #   DH[i][0] = q[i]
  #   DH[i][1] = d[i]
  #   DH[i][2] = a[i]
  #   DH[i][3] = alpha[i]

  # M = np.eye(4)
  # for i in range(nlink):
  #     R = np.array([[cos(DH[i][0]), -sin(DH[i][0]) * cos(DH[i][3]), +sin(DH[i][0]) * sin(DH[i][3])],
  #                   [sin(DH[i][0]), +cos(DH[i][0]) * cos(DH[i][3]), -cos(DH[i][0]) * sin(DH[i][3])],
  #                   [0, sin(DH[i][3]), cos(DH[i][3])]])
  #     T = np.array([[DH[i][2] * cos(DH[i][0])],
  #                   [DH[i][2] * sin(DH[i][0])],
  #                   [DH[i][1]]])
  #     RT = np.block([[R,                T],
  #                    [np.zeros((1, 3)), 1]])
  #     M = np.matmul(M, RT)
  # return M

  DH = np.array([[0,   0.333,   0, -pi/2],
                   [0,   0,   0,   pi/2],
                   [0,   0.316,   0.088,   pi/2],
                   [0,   0,   -0.088,  -pi/2],
                   [0,   0.384,   0,   pi/2],
                   [0,   0,   0.088,   pi/2],
                   [0,   0.227,   0,   0]])

  DH[:,0] = theta
  nlink = DH.shape[0]
  M = []
    # pos = []
  M.append(np.identity(4))

  for i in range(nlink):
      R = np.array([[cos(DH[i][0]), -sin(DH[i][0]) * cos(DH[i][3]), sin(DH[i][0]) * sin(DH[i][3])],
                    [sin(DH[i][0]), cos(DH[i][0]) * cos(DH[i][3]), -cos(DH[i][0])*sin(DH[i][3])],
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


  

q = np.array([0,np.pi/2,0,0,0,np.pi,0])
H = forward_kinematic_map(q)
print(H)
position = H[0:3,3]
print(position)

