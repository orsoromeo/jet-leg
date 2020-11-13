# -*- coding: utf-8 -*-
"""
Created on Fri 19 June 2020

@author: Romeo Orsolino
"""

import numpy as np
from jet_leg.dynamics.instantaneous_capture_point import InstantaneousCapturePoint

class ZeroMomentPoint():
    def __init__(self):
        self.zmp = [0.0, 0.0]
        self.Icp = InstantaneousCapturePoint()

    def compute(self, params):
        robot_height = self.Icp.computeAverageRobotHeight(params)
        print "avg height", robot_height
        return self.computeZeroMomentPoint(params.comPositionWF, params.comLinAcc, robot_height)


    def computeZeroMomentPoint(self, com_pos_WF, com_acc_WF, robot_height, gravity = -9.81):
        omega = np.sqrt(-gravity/robot_height)
        print "com_acc_WF", com_acc_WF
        omega_sqrd = omega**2
        self.zmp = com_pos_WF[0:2] - com_acc_WF[0:2]/omega_sqrd
        return self.zmp

