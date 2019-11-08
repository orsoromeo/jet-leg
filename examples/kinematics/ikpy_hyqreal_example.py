# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 06:10:18 2018

@author: Romeo Orsolino
"""

from jet_leg.hyqreal_kinematics import HyQRealKinematics
import numpy as np


epsilon = 1e-03
LF_foot = np.array([0.4571, 0.3612, -0.5074])
RF_foot = np.array([0.4571, -0.3612, -0.5074])
LH_foot = np.array([-0.429867, 0.3612, -0.5074])
RH_foot = np.array([-0.429867, -0.3612, -0.5074])
starting_contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
hyqrealKin = HyQRealKinematics()
foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])

q_leg_ikpy = hyqrealKin.leg_inverse_kin_ikpy(0, starting_contacts)

#for legID in range(0,4):
#    q_leg_ikpy = hyqrealKin.leg_inverse_kin_ikpy(legID, starting_contacts)

print "joint pos ", q_leg_ikpy

q_leg_ikpy = hyqrealKin.leg_inverse_kin_ikpy(1, starting_contacts)
print "joint pos ", q_leg_ikpy

q_leg_ikpy = hyqrealKin.leg_inverse_kin_ikpy(2, starting_contacts)
print "joint pos ", q_leg_ikpy

q_leg_ikpy = hyqrealKin.leg_inverse_kin_ikpy(3, starting_contacts)
print "joint pos ", q_leg_ikpy

des_q = [-0.2, 0.75, -1.5]