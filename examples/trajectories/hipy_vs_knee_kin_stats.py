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
knee_default_min = -2.86

hipy_kin_min = hip_y_default_min - 60.0/180.0*np.pi
hipy_kin_max = hip_y_default_min + 60.0/180.0*np.pi
N_hipy = 7

hipy_range = np.linspace(hipy_kin_min, hipy_kin_max, num=N_hipy)
hipy_range = [round(h, 2) for h in hipy_range]

knee_kin_min = knee_default_min - 30.0/180.0*np.pi
knee_kin_max = knee_default_min + 30.0/180.0*np.pi
N_knee = 13
knee_range = np.linspace(knee_kin_min, knee_kin_max, num=N_knee)
knee_range = [round(k, 2) for k in knee_range]

optimize_height_and_pitch = False
feas = KinematicFeasibility(params.pin)

data = np.zeros([N_knee, N_hipy])
for kin_k in range(0, N_knee):
    for kin_h in range(0, N_hipy):
        print('indices', kin_k, kin_h)
        step_height = 0.0
        while (feas.test_hipy_vs_knee_position_limits(optimize_height_and_pitch,
                                                      params, robot, step_height, hipy_range[kin_h], knee_range[kin_k])):
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
print(knee_range)
print(hipy_range)
plt.xticks(np.linspace(0, N_hipy, num=N_hipy), hipy_range)
plt.yticks(np.linspace(0, N_knee, num=N_knee), knee_range)
plt.show()
