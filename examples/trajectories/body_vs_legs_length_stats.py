# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""
import numpy as np
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.feasibility.body_params_feasibility import BodyParamsFeasibility

import matplotlib.pyplot as plt

plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = "lemo_EP0"
params = IterativeProjectionParameters(robot)
comp_dyn = ComputationalDynamics(robot)
params.setDefaultValuesWrtWorld()

links_length_min = 0.25
links_length_max = 0.4
N_links_l = 16

links_range = np.linspace(links_length_min, links_length_max, num=N_links_l)

body_length_min = 0.25
body_length_max = 0.45
N_body_l = 11
body_range = np.linspace(body_length_min, body_length_max, num=N_body_l)

optimize_height_and_pitch = False
feas = BodyParamsFeasibility()

data = np.zeros([N_body_l, N_links_l])
for b in range(0, N_body_l):
    for l in range(0, N_links_l):
        print('indices', b, l)
        step_height = 0.0
        while (feas.test_body_vs_links_length(optimize_height_and_pitch,
                                              params, robot, step_height, body_range[b], links_range[l])):
            data[b, l] = step_height
            print('data', b, l, data[b, l])
            step_height += 0.025
            print('step height', step_height)

print('generate colormap', data)
plt.figure()
plt.pcolormesh(data)
plt.colorbar()
plt.ylabel("body length [m]")
plt.xlabel("links length [m]")
print(body_range)
print(links_range)
plt.xticks(np.linspace(0, N_links_l, num=N_links_l), links_range)
plt.yticks(np.linspace(0, N_body_l, num=N_body_l), body_range)
plt.show()
