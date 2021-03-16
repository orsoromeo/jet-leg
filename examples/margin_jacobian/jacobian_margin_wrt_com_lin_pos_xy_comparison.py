"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from copy import copy
import random
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization.jacobians import Jacobians
from plot_learned_com_pos_margin_jac_xy_comparison import LearnedMargin
import matplotlib
import matplotlib.pyplot as plt

plt.rc('text', usetex=True)
plt.rc('font', family='serif')

plt.close('all')
math = Math()

def computeAnalyticMarginAndDerivatives(stanceFeet, robot):

    params = IterativeProjectionParameters(robot)
    params.setDefaultValues()

    randomSwingLeg = random.randint(0, 3)
    tripleStance = False  # if you want you can define a swing leg using this variable
    if tripleStance:
        print 'Swing leg', randomSwingLeg
        stanceFeet[randomSwingLeg] = 0
    print 'stanceLegs ', stanceFeet
    params.setActiveContacts(stanceFeet)

    params_com_x = copy(params)
    params_com_y = copy(params)
    params_com_z = copy(params)

    jac = Jacobians(robot)

    delta_pos_range = 0.79
    delta_pos_range_z = 0.4
    dx = 0.02
    num_of_tests = delta_pos_range/dx
    delta_pos_range_vec_x = np.linspace(-delta_pos_range/2.0, delta_pos_range/2.0, num_of_tests)
    delta_pos_range_vec_y = np.linspace(-delta_pos_range/2.0, delta_pos_range/2.0, num_of_tests)
    delta_pos_range_vec_z = np.linspace(-delta_pos_range_z/2.0, delta_pos_range_z/2.0, num_of_tests)
    print "number of tests", num_of_tests

    pos_margin_x, jac_com_pos_x = jac.plotMarginAndJacobianWrtComPosition(params_com_x,delta_pos_range_vec_x, 0) # dm / dx
    pos_margin_y, jac_com_pos_y = jac.plotMarginAndJacobianWrtComPosition(params_com_y,delta_pos_range_vec_y, 1) # dm / dy
    pos_margin_z, jac_com_pos_z = jac.plotMarginAndJacobianWrtComPosition(params_com_z,delta_pos_range_vec_z, 2) # dm / dz

    return pos_margin_x, jac_com_pos_x, pos_margin_y, jac_com_pos_y, pos_margin_z, jac_com_pos_z, delta_pos_range_vec_x, delta_pos_range_vec_y, delta_pos_range_vec_z


def plotAnalyticMarginAndDerivatives(idx1, idx2, pos_margin_x, jac_com_pos_x, pos_margin_y, jac_com_pos_y, pos_margin_z, jac_com_pos_z, delta_pos_range_vec_x, delta_pos_range_vec_y, delta_pos_range_vec_z ):
    plt.subplot(idx1)
    plt.plot(delta_pos_range_vec_x, pos_margin_x, 'r-o', markersize=2, label='analytic')

    plt.subplot(idx2)
    plt.plot(delta_pos_range_vec_y, pos_margin_y, 'r-o', markersize=2, label='analytic')

### Plotting

### X axis
robot_name = 'anymal_coyote'
learnedMargin = LearnedMargin()

fig1 = plt.figure(1)
#fig8.suptitle("RF and LH feet in swing")
contacts = [1, 1, 1, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(321, 322, mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1111stance.txt', 321, 322)

contacts = [1, 1, 0, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(323, 324, mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1101stance.txt', 323, 324)

contacts = [1, 0, 1, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(325, 326, mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1011stance.txt', 325, 326)

learnedMargin.set_plot_properties(-0.05, 0.15)
fig1.savefig('../../figs/paper_margin_pos.pdf')
fig1.savefig('../../figs/paper_margin_pos.png')

plt.show()