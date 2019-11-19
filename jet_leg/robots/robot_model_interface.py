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
        elif self.robotName == 'anymal':
            self.robotModel = AnymalModel()
        elif self.robotName == 'hyqreal':
            self.robotModel = HyqrealModel()