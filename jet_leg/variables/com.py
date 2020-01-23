"""
Created on Jan 23 2020

@author: Romeo Orsolino
"""
import numpy as np
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters

class CoM:
    def __init__(self, ip_params):
        self.name = "com"
        self.pos = ip_params.comPositionWF
        self.eulerAngles = [ip_params.roll, ip_params.pitch, ip_params.yaw]
        self.linVel = ip_params.comLinVel

        self.linAcc = ip_params.comLinAcc
        self.angAcc = ip_params.comAngAcc

class Foot:
    def __init__(self, foot_name, ip_params):
        self.name = foot_name
        self.pos = ip_params.contactsWF

class Feet:
    def __init__(self):
        self.footLF = Foot("lf_foot")
        self.footRF = Foot("rf_foot")
        self.footLH = Foot("lh_foot")
        self.footRH = Foot("rh_foot")
