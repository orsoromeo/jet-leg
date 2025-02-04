# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 06:10:18 2018

@author: Romeo Orsolino
"""

from context import jet_leg

from jet_leg.robot.hyq_kinematics import HyQKinematics

import numpy as np
import random
import time

import unittest

class TestStringMethods():

    assertPrecision = 3

    def test_leg_ikpy(self):
        epsilon = 1e-03
        LF_foot = np.array([0.3735, 0.33, -0.5])
        RF_foot = np.array([0.3735, -0.33, -0.5])
        LH_foot = np.array([-0.3735, 0.33, -0.5])
        RH_foot = np.array([-0.3735, -0.33, -0.5])
        starting_contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
        hyqKin = HyQKinematics()
        foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])

        for legID in range(0,4):
            q_leg_ikpy = hyqKin.leg_inverse_kin_ikpy(legID, starting_contacts)
            hyqKin.init_jacobians()
            hyqKin.init_homogeneous()

            q = hyqKin.inverse_kin(starting_contacts, foot_vel)
            q_leg_hardcoded = q[3*legID +0: 3*legID + 3]
            self.assertTrue(q_leg_ikpy[0] - q_leg_hardcoded[0] < epsilon)
            self.assertTrue(q_leg_ikpy[1] - q_leg_hardcoded[1] < epsilon)
            self.assertTrue(q_leg_ikpy[2] - q_leg_hardcoded[2] < epsilon)

    def test_kinematic_loop_default_configuration(self):

        random.seed(9001)
        hyqKin = HyQKinematics()
        hyqKin.init_jacobians()
        hyqKin.init_homogeneous()
        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
        LF_foot = np.array([0.3735, 0.207, -0.6])
        RF_foot = np.array([0.3735, -0.207, -0.6])
        LH_foot = np.array([-0.3735, 0.207, -0.6])
        RH_foot = np.array([-0.3735, -0.207, -0.6])
        starting_contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

        q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = hyqKin.inverse_kin(np.transpose(starting_contacts[:,0]),
                                                                                np.transpose(foot_vel[:,0]),
                                                                                np.transpose(starting_contacts[:,1]),
                                                                                np.transpose(foot_vel[:,1]),
                                                                                np.transpose(starting_contacts[:,2]),
                                                                                np.transpose(foot_vel[:,2]))

        hyqKin.update_homogeneous(q)
        hyqKin.update_jacobians(q)
        new_contacts = hyqKin.forward_kin(q)

        self.assertTrue((LF_foot - new_contacts[0,:] < self.epsilon).all())
        self.assertTrue((RF_foot - new_contacts[1,:] < self.epsilon).all())
        self.assertTrue((LH_foot - new_contacts[2,:] < self.epsilon).all())
        self.assertTrue((RH_foot - new_contacts[3,:] < self.epsilon).all())

    def test_position_is_out_of_workspace(self):

        random.seed(9001)
        for j in range(0,10):
            hyqKin = HyQKinematics()

            hyqKin.init_jacobians()
            hyqKin.init_homogeneous()

            foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])

            foot_z = random.uniform(-1,-0.8)
            LF_foot = np.array([0.3, 0.2, foot_z])
            RF_foot = np.array([0.3, -0.2, foot_z])
            LH_foot = np.array([-0.3, 0.2, foot_z])
            RH_foot = np.array([-0.3, -0.2, foot_z])
            contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

            q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = hyqKin.inverse_kin(np.transpose(contacts[:,0]),
                                                                                    np.transpose(foot_vel[:,0]),
                                                                                    np.transpose(contacts[:,1]),
                                                                                    np.transpose(foot_vel[:,1]),
                                                                                    np.transpose(contacts[:,2]),
                                                                                    np.transpose(foot_vel[:,2]))
            self.assertTrue(isOutOfWorkspace)

    def test_inverse_kinematics(self):

        hyqKin = HyQKinematics()

        hyqKin.init_jacobians()
        hyqKin.init_homogeneous()

        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])

        LF_foot = np.array([0.3, 0.2, -0.5])
        RF_foot = np.array([0.3, -0.2, -0.58])
        LH_foot = np.array([-0.3, 0.2, -0.58])
        RH_foot = np.array([-0.3, -0.2, -0.58])
        contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        #print contacts

        start_time = time.time()
        q = hyqKin.inverse_kin(contacts, foot_vel)
        print('total time', time.time()- start_time)
        print q
        hyqKin.update_homogeneous(q)
        hyqKin.update_jacobians(q)
        new_contacts = hyqKin.forward_kin(q)
        print new_contacts

        #self.assertTrue((LF_foot - new_contacts[0,:] < self.epsilon).all())
        #self.assertTrue((RF_foot - new_contacts[1,:] < self.epsilon).all())
        #self.assertTrue((LH_foot - new_contacts[2,:] < self.epsilon).all())
        #self.assertTrue((RH_foot - new_contacts[3,:] < self.epsilon).all())

        new_q = hyqKin.inverse_kin(new_contacts, foot_vel)

        print new_q

        #self.assertTrue( (q - new_q < self.epsilon).all())



    def test_forward_kinematics(self):

        hyqKin = HyQKinematics()

        hyqKin.init_jacobians()
        hyqKin.init_homogeneous()

        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])

        LF_foot = np.array([0.3, 0.2, -0.58])
        RF_foot = np.array([0.3, -0.2, -0.58])
        LH_foot = np.array([-0.3, 0.2, -0.58])
        RH_foot = np.array([-0.3, -0.2, -0.58])
        contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

        #print contacts

        q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = hyqKin.inverse_kin(np.transpose(contacts[:,0]),
                                                                                np.transpose(foot_vel[:,0]),
                                                                                np.transpose(contacts[:,1]),
                                                                                np.transpose(foot_vel[:,1]),
                                                                                np.transpose(contacts[:,2]),
                                                                                np.transpose(foot_vel[:,2]))

        #print q
        hyqKin.update_homogeneous(q)
        hyqKin.update_jacobians(q)
        new_contacts = hyqKin.forward_kin(q)
        #print new_contacts

        self.assertTrue((LF_foot - new_contacts[0,:] < self.epsilon).all())
        self.assertTrue((RF_foot - new_contacts[1,:] < self.epsilon).all())
        self.assertTrue((LH_foot - new_contacts[2,:] < self.epsilon).all())
        self.assertTrue((RH_foot - new_contacts[3,:] < self.epsilon).all())

        #self.assertEqual(LF_foot.tolist(), new_contacts[0,:].tolist())       
        #self.assertEqual(RF_foot.tolist(), new_contacts[1,:].tolist())
        #self.assertEqual(LH_foot.tolist(), new_contacts[2,:].tolist())
        #self.assertEqual(RH_foot.tolist(), new_contacts[3,:].tolist())


'''main'''
test = TestStringMethods()
test.test_inverse_kinematics()