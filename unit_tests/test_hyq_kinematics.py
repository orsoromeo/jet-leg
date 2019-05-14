# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 06:10:18 2018

@author: Romeo Orsolino
"""

from context import jet_leg

from jet_leg.hyq_kinematics import HyQKinematics

import numpy as np
import random
import ikpy

import unittest

class TestStringMethods(unittest.TestCase):

    assertPrecision = 3


    def test_ikpy(self):
        epsilon = 1e-03
        print "test IKPY"
        my_chain = ikpy.chain.Chain.from_urdf_file("../resources/urdfs/hyq/urdf/leg/hyq_leg_LF.urdf")

        target_vector = [0.3735, 0.207, -0.5]
        target_frame = np.eye(4)
        target_frame[:3, 3] = target_vector
        print target_frame
        q = my_chain.inverse_kinematics(target_frame)
        q_LF_ikpy = q[1:4]
        #print "ikpy: ",q, q_LF_ikpy

        hyqKin = HyQKinematics()

        hyqKin.init_jacobians()
        hyqKin.init_homogeneous()

        foot_vel = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])

        LF_foot = np.array(target_vector)
        RF_foot = np.array([0.3735, -0.207, -0.5])
        LH_foot = np.array([-0.3735, 0.207, -0.5])
        RH_foot = np.array([-0.3735, -0.207, -0.5])
        starting_contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

        q = hyqKin.inverse_kin(starting_contacts, foot_vel)
        q_LF_hardcoded = q[0:3]
        #print "hardcoded ik: ", q, q_LF_hardcoded
        print q_LF_ikpy - q_LF_hardcoded, epsilon

        self.assertTrue((q_LF_ikpy[0] - q_LF_hardcoded[0] < epsilon).all())


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

        new_q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = hyqKin.inverse_kin(np.transpose(new_contacts[:,0]),
                                                                                    np.transpose(foot_vel[:,0]),
                                                                                    np.transpose(new_contacts[:,1]),
                                                                                    np.transpose(foot_vel[:,1]),
                                                                                    np.transpose(new_contacts[:,2]),
                                                                                    np.transpose(foot_vel[:,2]))

        #print new_q


        self.assertTrue( (q - new_q < self.epsilon).all())



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