import numpy as np
from jet_leg.robot.hyq_kinematics import HyQKinematics

kin = HyQKinematics('anymal')

LF_foot = np.array([0.3735, 0.33, -0.5])
RF_foot = np.array([0.3735, -0.33, -0.5])
LH_foot = np.array([-0.3735, 0.33, -0.5])
RH_foot = np.array([-0.3735, -0.33, -0.5])
starting_contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
foot_vel = np.array([0.0, 0.0, 0.0, 0.0])

q = kin.inverse_kin(starting_contacts, foot_vel, 'anymal')
print "Pinocchio's IK for Anymal: ", q

