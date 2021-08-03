# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.feasibility.kinematic_feasibility import KinematicFeasibility

import matplotlib.pyplot as plt

from copy import copy

plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = "lemo_EP0"
params = IterativeProjectionParameters(robot)
params.setDefaultValuesWrtWorld()

''' Generate trajectory of footholds'''
step_height = 0.1

hip_y_kin_lim = -1.30899694
knee_kin_lim = -2.86233997

f = KinematicFeasibility()
optimize_height_and_pitch = False
f.test_hipy_vs_knee_position_limits(optimize_height_and_pitch,
                                    params, robot, step_height, hip_y_kin_lim, knee_kin_lim)

# fig = plt.figure()
# plt.plot(time_list, margin_list)
# plt.grid()
# plt.ylabel("margin [m]")
# plt.xlabel("time [s]")
# plt.show()
