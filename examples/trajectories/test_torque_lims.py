# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.feasibility.torques_feasibility import TorquesFeasibility

import matplotlib.pyplot as plt

from copy import copy

plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = "lemo_EP0"
params = IterativeProjectionParameters(robot)
params.setDefaultValuesWrtWorld()

''' Generate trajectory of footholds'''
step_height = 0.4

hip_y_tau_lim = 10.0
knee_tau_lim = 20.0

f = TorquesFeasibility()
optimize_height_and_pitch = False
f.test_hipy_vs_knee_torque(optimize_height_and_pitch,
                           params, robot, step_height, hip_y_tau_lim, knee_tau_lim)

# fig = plt.figure()
# plt.plot(time_list, margin_list)
# plt.grid()
# plt.ylabel("margin [m]")
# plt.xlabel("time [s]")
# plt.show()
