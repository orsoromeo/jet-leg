# -*- coding: utf-8 -*-
"""
Created on Mon Dec 17 11:39:13 2018

@author: rorsolino
"""

import numpy as np

class RigidBodyDynamics:
    def __init__(self): 
        self.LX = 0
        self.LY = 1
        self.LZ = 2
        self.AX = 3
        self.AY = 4
        self.AZ = 5

    def computeCentroidalWrench(self, robotsMass, com_pos_WF, externalWrench, comLinAcc = [0., 0., 0.], comAngAcc = [0., 0., 0.], comAngVel = [[0.], [0.], [0.]], gravity = - 9.81):

        extForce = externalWrench[0:3]
        extTau = externalWrench[3:6]
        static_linear = np.array([0.0, 0.0, robotsMass * gravity]) + extForce
        static_angular = np.cross(com_pos_WF, static_linear) + extTau
        inertial_linear = np.multiply(robotsMass, comLinAcc)
        robotInertia = np.eye(3)  # TODO: add the inertial matrix here
        coriolis = robotInertia.dot(comAngVel)
        coriolis = np.cross(np.transpose(comAngVel), np.transpose(coriolis))
        coriolis = coriolis[0]
        inertial_angular = np.matmul(robotInertia, comAngAcc) + coriolis
        linear_aggr_wrench = inertial_linear - static_linear
        angular_aggr_wrench = inertial_angular - static_angular
        total_wrench = np.hstack([linear_aggr_wrench, angular_aggr_wrench])

        return total_wrench
