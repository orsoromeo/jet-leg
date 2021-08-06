# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np
import sys
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.feasibility.payload_feasibility import PayloadFeasibility

import matplotlib.pyplot as plt

from copy import copy

plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = sys.argv[1]
params = IterativeProjectionParameters(robot)
params.setDefaultValuesWrtWorld()

''' Generate trajectory of footholds'''
step_height = 0.2

vertical_force = -98.0
horizontal_location = 0.0

f = PayloadFeasibility(params.pin)
optimize_height_and_pitch = False
f.test_payload_amplitude_vs_payload_pos(optimize_height_and_pitch,
                                        params, robot, step_height, vertical_force, horizontal_location)

# fig = plt.figure()
# plt.plot(time_list, margin_list)
# plt.grid()
# plt.ylabel("margin [m]")
# plt.xlabel("time [s]")
# plt.show()
