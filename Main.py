import numpy as np
import evaluator, agent, env
import time
import progressbar

np_kinematic = 4
np_dynamic   = 5
dt           = 0.01
Gamma_k      = np.eye(np_kinematic)
Gamma_d      = np.eye(np_dynamic)
lambda_f     = 1


if __name__ == "__main__":

    for t in range(1000):

