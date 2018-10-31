# -*- coding: utf-8 -*-
"""
Created on Sun Jul 15 17:22:17 2018

@author: romeoorsolino
"""
from context import jet_leg

from jet_leg.hyq_kinematics import HyQKinematics
from jet_leg.math_tools import Math
from jet_leg.constraints import Constraints

import numpy as np
import random

import unittest

class TestStringMethods(unittest.TestCase):
    
    epsilon = 10e-02
    assertPrecision = 3
    
    def test_2D_jacobians(self):
        math = Math()
        random.seed(9001)
        constraint_mode = 'ONLY_ACTUATION'
        nc = 1
        ng = 4
        friction_coeff = 0.6
        
        axisZ = np.array([[0.0], [0.0], [1.0]])
        n1 = np.transpose(np.transpose(math.rpyToRot(1.5,1.5,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2

        normals = np.vstack([n1, n2, n3, n4])
        J_LF = np.eye((2))
        J_RF = np.eye((2))
        J_LH = np.eye((2))
        J_RH = np.eye((2))
        constraint = Constraints()
        
        G, h, isConstraintOk, actuationPolygons = constraint.getInequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
        #print G, h
     
        for j in range(0,len(h)):
            self.assertTrue(h[j] > self.epsilon)
            
        self.assertTrue(h[0]/G[0,2] - 120.0 < self.epsilon)
        self.assertTrue(h[2]/G[2,0] - 80.0 < self.epsilon)
        self.assertTrue(h[4]/G[3,1] - 120.0 < self.epsilon)

    def test_3D_jacobians(self):
        math = Math()
        random.seed(9001)
        constraint_mode = 'ONLY_ACTUATION'
        nc = 1
        ng = 4
        friction_coeff = 0.6
        
        axisZ = np.array([[0.0], [0.0], [1.0]])
        n1 = np.transpose(np.transpose(math.rpyToRot(1.5,1.5,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2

        normals = np.vstack([n1, n2, n3, n4])
        J_LF = np.eye((3))
        J_RF = np.eye((3))
        J_LH = np.eye((3))
        J_RH = np.eye((3))
        constraint = Constraints()
        
        G, h, isConstraintOk, actuationPolygons = constraint.getInequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
        #print G, h
     
        for j in range(0,len(h)):
            self.assertTrue(h[j] > self.epsilon)
            
        self.assertTrue(h[0]/G[0,2] - 120.0 < self.epsilon)
        self.assertTrue(h[2]/G[2,0] - 80.0 < self.epsilon)
        self.assertTrue(h[4]/G[3,1] - 120.0 < self.epsilon)
        
    def test_2D_jacobians_and_actuation_polygons(self):
        math = Math()
        random.seed(9001)
        constraint_mode = 'ONLY_ACTUATION'
        nc = 1
        ng = 4
        friction_coeff = 0.6
        
        axisZ = np.array([[0.0], [0.0], [1.0]])
        n1 = np.transpose(np.transpose(math.rpyToRot(1.5,1.5,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2

        normals = np.vstack([n1, n2, n3, n4])

        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
                
        LF_foot = np.array([0.3, 0.2, -0.5])
        RF_foot = np.array([0.3, -0.2, -0.5])
        LH_foot = np.array([-0.2, 0.2, -0.5])
        RH_foot = np.array([-0.3, -0.2, -0.5])

        contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        constraint = Constraints()
        kin = HyQKinematics()
        q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = kin.inverse_kin(np.transpose(contacts[:,0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contacts[:,1]),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contacts[:,2]),
                                                    np.transpose(foot_vel[:,2]))
        if (not isOutOfWorkspace):
            kin.update_jacobians(q)
            #print J_LF
            G, h, isConstraintOk, actuationPolygons = constraint.getInequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
 
        for j in range(0,len(h)):
            self.assertTrue(h[j] > self.epsilon)
            
    def test_3D_jacobians_and_actuation_polygons(self):
        math = Math()
        random.seed(9001)
        constraint_mode = 'ONLY_ACTUATION'
        nc = 1
        ng = 4
        friction_coeff = 0.6
        
        axisZ = np.array([[0.0], [0.0], [1.0]])
        n1 = np.transpose(np.transpose(math.rpyToRot(1.5,1.5,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2

        normals = np.vstack([n1, n2, n3, n4])

        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
                
        LF_foot = np.array([0.3, 0.2, -0.5])
        RF_foot = np.array([0.3, -0.2, -0.5])
        LH_foot = np.array([-0.2, 0.2, -0.5])
        RH_foot = np.array([-0.3, -0.2, -0.5])

        contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        constraint = Constraints()
        kin = HyQKinematics()
        q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = kin.inverse_kin(np.transpose(contacts[:,0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contacts[:,1]),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contacts[:,2]),
                                                    np.transpose(foot_vel[:,2]))

        J_LF, J_RF, J_LH, J_RH = kin.update_jacobians(q)
        
        if (not isOutOfWorkspace):
            kin.update_jacobians(q)
            #print J_LF
            G, h, isConstraintOk, actuationPolygons = constraint.getInequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
 
        for j in range(0,len(h)):
            self.assertTrue(h[j] > self.epsilon)
            
    def test_2D_and_3D_jacobians_and_actuation_polygons(self):
        math = Math()
        random.seed(9001)
        constraint_mode = 'ONLY_ACTUATION'
        nc = 1
        ng = 4
        friction_coeff = 0.6
        
        axisZ = np.array([[0.0], [0.0], [1.0]])
        n1 = np.transpose(np.transpose(math.rpyToRot(1.5,1.5,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2

        normals = np.vstack([n1, n2, n3, n4])

        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
                
        LF_foot = np.array([0.3, 0.2, -0.5])
        RF_foot = np.array([0.3, -0.2, -0.5])
        LH_foot = np.array([-0.2, 0.2, -0.5])
        RH_foot = np.array([-0.3, -0.2, -0.5])

        contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        constraint = Constraints()
        kin = HyQKinematics()
        q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = kin.inverse_kin(np.transpose(contacts[:,0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contacts[:,1]),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contacts[:,2]),
                                                    np.transpose(foot_vel[:,2]))
        if (not isOutOfWorkspace):
            kin.update_jacobians(q)
            #print J_LF
            G, h, isConstraintOk, actuationPolygons = constraint.getInequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
 
        J_LF_3D, J_RF_3D, J_LH_3D, J_RH_3D = kin.update_jacobians(q)
        
        if (not isOutOfWorkspace):
            kin.update_jacobians(q)
            #print J_LF
            G_new, h_new, isConstraintOk, actuationPolygons = constraint.getInequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF_3D, J_RF_3D, J_LH_3D, J_RH_3D)
 
        for j in range(0,len(h)):
            self.assertTrue(h[j] - h_new[j] < self.epsilon)
            for i in range(0, np.size(G,1)):
                #print G[j,i]/h[j]
                #print G_new[j,i]/h_new[j]
                self.assertTrue(G[j,i]/h[j] - G_new[j,i]/h_new[j] < self.epsilon)