# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""
import numpy as np
import sys
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.feasibility.payload_feasibility import PayloadFeasibility

import matplotlib.pyplot as plt

plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = sys.argv[1]
params = IterativeProjectionParameters(robot)
comp_dyn = ComputationalDynamics(robot, params.pin)
params.setDefaultValuesWrtWorld()

min_load = 0.0
max_load = -250.0
N_load = 26
load_range = np.linspace(min_load, max_load, num=N_load)

min_lever = -0.3
max_lever = 0.3
N_lever = 11
lever_range = np.linspace(min_lever, max_lever, num=N_lever)
lever_range = [round(k, 2) for k in lever_range]

optimize_height_and_pitch = False
feas = PayloadFeasibility(params.pin)

data = np.zeros([N_lever, N_load])
for lever_idx in range(0, N_lever):
    for load_idx in range(0, N_load):
        print('indices', lever_idx, load_idx)
        step_height = 0.0
        while (feas.test_payload_amplitude_vs_payload_pos(optimize_height_and_pitch,
                                                          params, robot, step_height, load_range[load_idx], lever_range[lever_idx])):
            data[lever_idx, load_idx] = step_height
            print('data', lever_idx, load_idx, data[lever_idx, load_idx])
            step_height += 0.025
            print('step height', step_height)

print('generate colormap', data)
plt.figure()
plt.pcolormesh(data)
plt.colorbar()
plt.ylabel("horizontal load position [m]")
plt.xlabel("payload [N]")
print(lever_range)
print(load_range)
plt.xticks(np.linspace(0, N_load, num=N_load), load_range)
plt.yticks(np.linspace(0, N_lever, num=N_lever), lever_range)
plt.show()
