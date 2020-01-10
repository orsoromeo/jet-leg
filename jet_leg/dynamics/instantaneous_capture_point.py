# -*- coding: utf-8 -*-
"""
Created on Tue Dec 17 10:54:31 2019

@author: Romeo Orsolino
"""

import numpy as np

class InstantaneousCapturePoint():
    def __init__(self):
        self.icp = [0.0, 0.0]

    def compute(self, params):
        robot_height = 0.4
        return self.computeInstantaneousCapturePoint(params.comPositionWF, params.comLinVel, robot_height)

    def computeInstantaneousCapturePoint(self, com_pos_WF, com_vel_WF, robot_height, gravity = -9.81):
        omega = np.sqrt(-gravity/robot_height)
        icp = com_pos_WF[0:2] + com_vel_WF[0:2]/omega
        return icp

