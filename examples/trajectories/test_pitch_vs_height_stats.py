# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.feasibility.find_feasible_trajectories import FeasibilityAnalysis

import matplotlib.pyplot as plt

from copy import copy

plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = "lemo_EP0"
params = IterativeProjectionParameters(robot)
comp_dyn = ComputationalDynamics(robot)
params.setDefaultValuesWrtWorld()

''' Generate trajectory of footholds'''
step_height = 0.48
pitch = -0.7
des_height = 0.36

f = FeasibilityAnalysis()
f.test_pitch_and_height(
    params, comp_dyn, step_height, des_height, pitch)

# fig = plt.figure()
# plt.plot(time_list, margin_list)
# plt.grid()
# plt.ylabel("margin [m]")
# plt.xlabel("time [s]")
# plt.show()
