# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""
import numpy as np
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.feasibility.height_and_pitch_feasibility import HeightPitchFeasibility

import matplotlib.pyplot as plt

plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = "lemo_EP0"
params = IterativeProjectionParameters(robot)
comp_dyn = ComputationalDynamics(robot)
params.setDefaultValuesWrtWorld()

pitch_min = 0.0
pitch_max = -0.8
N_pitch = 21

pitch_range = np.linspace(pitch_min, pitch_max, num=N_pitch)

des_height_min = 0.3
des_height_max = 0.4
N_height = 11
height_range = np.linspace(des_height_min, des_height_max, num=N_height)

feas = HeightPitchFeasibility()

data = np.zeros([N_pitch, N_height])
for p in range(0, N_pitch):
    params.setDefaultValuesWrtWorld()
    for h in range(0, N_height):
        params.setDefaultValuesWrtWorld()
        print('indices', p, h)
        step_height = 0.0
        while (feas.test_pitch_and_height(params, comp_dyn, step_height, height_range[h], pitch_range[p])):
            data[p, h] = step_height
            print('data', p, h, data[p, h])
            step_height += 0.025
            print('step height', step_height)
            params.setDefaultValuesWrtWorld()

print('generate colormap', data)
plt.figure()
plt.pcolormesh(data)
plt.colorbar()
plt.ylabel("body pitch [rad]")
plt.xlabel("body des height [m]")
plt.xticks(np.linspace(0, N_height, num=N_height), height_range)
plt.yticks(np.linspace(0, N_pitch, num=N_pitch), pitch_range)
plt.show()
