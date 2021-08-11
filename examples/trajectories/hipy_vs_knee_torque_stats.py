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
from jet_leg.feasibility.torques_feasibility import TorquesFeasibility

import matplotlib.pyplot as plt

plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = sys.argv[1]
params = IterativeProjectionParameters(robot)
comp_dyn = ComputationalDynamics(robot, params.pin)
params.setDefaultValuesWrtWorld()

hipy_tau_min = 0
hipy_tau_max = 150
N_hipy_tau = 16

hipy_range = np.linspace(hipy_tau_min, hipy_tau_max, num=N_hipy_tau)

knee_tau_min = 0
knee_tau_max = 200
N_knee_tau = 21
knee_range = np.linspace(knee_tau_min, knee_tau_max, num=N_knee_tau)

optimize_height_and_pitch = False
feas = TorquesFeasibility(params.pin)

data = np.zeros([N_knee_tau, N_hipy_tau])
for tau_k in range(0, N_knee_tau):
    for tau_h in range(0, N_hipy_tau):
        print('indices', tau_k, tau_h)
        step_height = 0.0
        while (feas.test_hipy_vs_knee_torque(optimize_height_and_pitch,
                                             params, robot, step_height, hipy_range[tau_h], knee_range[tau_k])):
            data[tau_k, tau_h] = step_height
            print('data', tau_k, tau_h, data[tau_k, tau_h])
            step_height += 0.025
            print('step height', step_height)

print('generate colormap', data)
plt.figure()
plt.pcolormesh(data)
plt.colorbar()
plt.ylabel("Max Knee torque [Nm]")
plt.xlabel("Max Hip Y torque [Nm]")
print(knee_range)
print(hipy_range)
plt.xticks(np.linspace(0, N_hipy_tau, num=N_hipy_tau), hipy_range)
plt.yticks(np.linspace(0, N_knee_tau, num=N_knee_tau), knee_range)
plt.show()
