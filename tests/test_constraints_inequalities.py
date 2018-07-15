# -*- coding: utf-8 -*-
"""
Created on Sun Jul 15 17:22:17 2018

@author: romeoorsolino
"""
from context import legsthrust

from legsthrust.hyq_kinematics import HyQKinematics
from legsthrust.math_tools import Math
from legsthrust.constraints import Constraints

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
        
        G, h, isConstraintOk = constraint.inequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
        print G, h
     
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
        
        G, h, isConstraintOk = constraint.inequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
        print G, h
     
        for j in range(0,len(h)):
            self.assertTrue(h[j] > self.epsilon)
            
        self.assertTrue(h[0]/G[0,2] - 120.0 < self.epsilon)
        self.assertTrue(h[2]/G[2,0] - 80.0 < self.epsilon)
        self.assertTrue(h[4]/G[3,1] - 120.0 < self.epsilon)