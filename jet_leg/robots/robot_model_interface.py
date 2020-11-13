# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 05:34:42 2018

@author: romeo orsolino
"""
import numpy as np

from jet_leg.robots.hyq.hyq_model import HyqModel
from jet_leg.robots.anymal.anymal_model import AnymalModel
from jet_leg.robots.hyqreal.hyqreal_model import HyqrealModel


class RobotModelInterface:
    def __init__(self, robot_name):
        self.robotName = robot_name
        if self.robotName == 'hyq':
            self.robotModel = HyqModel()
        elif self.robotName == 'anymal_boxy' or self.robotName == 'anymal_coyote':
            self.robotModel = AnymalModel(self.robotName)
        elif self.robotName == 'hyqreal':
            self.robotModel = HyqrealModel()
        else:
            print "Warning! could not set robot model!"

        self.joint_torque_limits = self.robotModel.joint_torque_limits
        self.contact_torque_limits = self.robotModel.contact_torque_limits
        self.trunkMass = self.robotModel.trunkMass