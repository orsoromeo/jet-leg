import numpy as np
from context import jet_leg
import time
from numpy import array
import ikpy
from jet_leg.hyq_kinematics import HyQKinematics


ik_LH = ikpy.chain.Chain.from_urdf_file("../resources/urdfs/hyq/urdf/leg/hyq_leg_LH.urdf")
ik_RH = ikpy.chain.Chain.from_urdf_file("../resources/urdfs/hyq/urdf/leg/hyq_leg_RH.urdf")


LF_foot = np.array([0.3735, 0.33, -0.5])
RF_foot = np.array([0.3735, -0.33, -0.5])
LH_foot = np.array([-0.3735, 0.33, -0.5])
RH_foot = np.array([-0.3735, -0.33, -0.5])
starting_contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

target_frame = np.eye(4)
target_frame[:3, 3] = LF_foot
print target_frame
ik_LF = ikpy.chain.Chain.from_urdf_file("../resources/urdfs/hyq/urdf/leg/hyq_leg_LF.urdf")
q = ik_LF.inverse_kinematics(target_frame)
print "ikpy LF: ", q

target_frame = np.eye(4)
target_frame[:3, 3] = RF_foot
print target_frame
ik_RF = ikpy.chain.Chain.from_urdf_file("../resources/urdfs/hyq/urdf/leg/hyq_leg_RF.urdf")
q = ik_RF.inverse_kinematics(target_frame)
print "ikpy RF: ", q

target_frame = np.eye(4)
target_frame[:3, 3] = LH_foot
print target_frame
q = ik_LH.inverse_kinematics(target_frame)
print "ikpy LH: ", q

target_frame = np.eye(4)
target_frame[:3, 3] = RH_foot
print target_frame
q = ik_RH.inverse_kinematics(target_frame)
print "ikpy RH: ", q
#real_frame = ik_LF.forward_kinematics(q)
#print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_frame[:3, 3]))

hyqKin = HyQKinematics()

hyqKin.init_jacobians()
hyqKin.init_homogeneous()

foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])

#print starting_contacts

q = hyqKin.inverse_kin(starting_contacts, foot_vel)
print "hardcoded ik: ", q