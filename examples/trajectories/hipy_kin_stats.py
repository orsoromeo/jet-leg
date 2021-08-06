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
from jet_leg.feasibility.kinematic_feasibility import KinematicFeasibility

import matplotlib.pyplot as plt

plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = sys.argv[1]
params = IterativeProjectionParameters(robot)
comp_dyn = ComputationalDynamics(robot)
params.setDefaultValuesWrtWorld()

hip_y_default_min = -1.30
hip_y_default_max = 3.4
amplitude = (hip_y_default_max - hip_y_default_min)/2.0

hipy_min_min = hip_y_default_min - amplitude
hipy_min_max = hip_y_default_min + amplitude
N_hipy = 15

hipy_min_range = np.linspace(hipy_min_min, hipy_min_max, num=N_hipy)
hipy_min_range = [round(h, 2) for h in hipy_min_range]

hip_y_max_min = hip_y_default_max - amplitude
hip_y_max_max = hip_y_default_max + amplitude
N_knee = 15
hipy_max_range = np.linspace(hip_y_max_min, hip_y_max_max, num=N_knee)
hipy_max_range = [round(k, 2) for k in hipy_max_range]

optimize_height_and_pitch = False
feas = KinematicFeasibility(params.pin)

data = np.zeros([N_knee, N_hipy])
for kin_k in range(0, N_knee):
    for kin_h in range(0, N_hipy):
        print('indices', kin_k, kin_h)
        step_height = 0.0
        while (feas.test_hipy_position_limits(optimize_height_and_pitch,
                                              params, robot, step_height, hipy_min_range[kin_h], hipy_max_range[kin_k])):
            data[kin_k, kin_h] = step_height
            print('data', kin_k, kin_h, data[kin_k, kin_h])
            step_height += 0.025
            print('step height', step_height)

print('generate colormap', data)
plt.figure()
plt.pcolormesh(data)
plt.colorbar()
plt.ylabel("Min Knee Lim [rad]")
plt.xlabel("Min Hip Lim [rad]")
print(hipy_max_range)
print(hipy_min_range)
plt.xticks(np.linspace(0, N_hipy, num=N_hipy), hipy_min_range)
plt.yticks(np.linspace(0, N_knee, num=N_knee), hipy_max_range)
plt.show()
