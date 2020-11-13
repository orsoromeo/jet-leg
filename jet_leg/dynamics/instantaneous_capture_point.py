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
        robot_height = self.computeAverageRobotHeight(params)
        print "avg height", robot_height
        return self.computeInstantaneousCapturePoint(params.comPositionWF, params.comLinVel, robot_height)

    def computeAverageRobotHeight(self, params):
        avg_foot_height = 0.0
        nc = np.sum(params.stanceFeet)
        stanceID = params.getStanceIndex(params.stanceFeet)
        # plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)
        for j in range(0, nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
            idx = int(stanceID[j])
            avg_foot_height += params.contactsWF[idx][2]

        numberOfStanceFeet = sum(params.stanceFeet)
        avg_foot_height = avg_foot_height/numberOfStanceFeet
        avg_height = params.comPositionWF[2] - avg_foot_height
        return avg_height

    def computeInstantaneousCapturePoint(self, com_pos_WF, com_vel_WF, robot_height, gravity = -9.81):
        omega = np.sqrt(-gravity/robot_height)
        print "vel", com_vel_WF
        icp = com_pos_WF[0:2] + com_vel_WF[0:2]/omega
        return icp

