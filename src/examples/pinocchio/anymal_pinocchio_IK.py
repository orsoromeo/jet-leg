from __future__ import print_function

from jet_leg.robots.anymal.anymal_kinematics import anymalKinematics
from pinocchio.utils import *

import time

LF_foot = np.array([0.3735, 0.33, -0.5])
RF_foot = np.array([0.3735, -0.33, -0.5])
LH_foot = np.array([-0.3735, 0.33, -0.5])
RH_foot = np.array([-0.3735, -0.33, -0.5])
feet_pos_des = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

print(feet_pos_des)

kin = anymalKinematics()
start_time = time.time()
q = kin.fixedBaseInverseKinematics(feet_pos_des)
print('total time is ',time.time()-start_time)
print('q is:', q.T)
print('\nresult: %s' % q.flatten().tolist())

