# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 05:34:42 2018

@author: romeo orsolino
"""
import numpy as np

from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from jet_leg.robots.hyq.hyq_kinematics import HyQKinematics
from jet_leg.robots.anymal.anymal_kinematics import anymalKinematics
from jet_leg.robots.hyqreal.hyqreal_kinematics import hyqrealKinematics


class KinematicsInterface:
    def __init__(self, robot_name):

        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()
        self.robotName = robot_name
        if self.robotName == 'hyq':
            self.hyqKin = HyQKinematics()
        if self.robotName == 'anymal':
            self.anymalKin = anymalKinematics()
        elif self.robotName == 'hyqreal':
            self.hyqrealKin = hyqrealKinematics()

    def get_jacobians(self):
        if self.robotName == 'hyq':
            return self.hyqKin.getLegJacobians()
        elif self.robotName == 'hyqreal':
            return self.hyqrealKin.getLegJacobians()
        elif self.robotName == 'anymal':
            return self.anymalKin.getLegJacobians()

    def inverse_kin(self, contactsBF, foot_vel):

        if self.robotName == 'hyq':
            q = self.hyqKin.fixedBaseInverseKinematics(contactsBF, foot_vel)
            return q
        elif self.robotName == 'hyqreal':
            q = self.hyqrealKin.fixedBaseInverseKinematics(contactsBF)
            return q
        elif self.robotName == 'anymal':
            q = self.anymalKin.fixedBaseInverseKinematics(contactsBF)
            return q

