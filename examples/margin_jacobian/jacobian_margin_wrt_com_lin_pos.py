"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from numpy import array
from copy import copy
import random
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization.jacobians import Jacobians
from plot_learned_com_pos_margin_jac import LearnedMargin

import matplotlib.pyplot as plt

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


def plotAnalyticMarginAndDerivatives(pos_margin_x, jac_com_pos_x, pos_margin_y, jac_com_pos_y, pos_margin_z, jac_com_pos_z, delta_pos_range_vec_x, delta_pos_range_vec_y, delta_pos_range_vec_z ):
    plt.subplot(431)
    plt.plot(delta_pos_range_vec_x, pos_margin_x, 'r-o', markersize=2, label='analytic (iterative projection)')

    plt.subplot(432)
    plt.plot(delta_pos_range_vec_y, pos_margin_y, 'r-o', markersize=2, label='analytic (iterative projection)')

    plt.subplot(433)
    plt.plot(delta_pos_range_vec_z, pos_margin_z, 'r-o', markersize=2, label='analytic (iterative projection)')

    plt.subplot(434)
    plt.plot(delta_pos_range_vec_x, jac_com_pos_x[0, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(437)
    plt.plot(delta_pos_range_vec_x, jac_com_pos_x[1, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(4, 3, 10)
    plt.plot(delta_pos_range_vec_x, jac_com_pos_x[2, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(435)
    plt.plot(delta_pos_range_vec_y, jac_com_pos_y[0, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(438)
    plt.plot(delta_pos_range_vec_y, jac_com_pos_y[1, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(4, 3, 11)
    plt.plot(delta_pos_range_vec_y, jac_com_pos_y[2, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(436)
    plt.plot(delta_pos_range_vec_z, jac_com_pos_z[0, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(439)
    plt.plot(delta_pos_range_vec_z, jac_com_pos_z[1, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(4, 3, 12)
    plt.plot(delta_pos_range_vec_z, jac_com_pos_z[2, :], 'r-o', markersize=2, label='analytic (finite-diff)')

### Plotting

### X axis
robot_name = 'anymal_coyote'
learnedMargin = LearnedMargin()

fig1 = plt.figure(1)
fig1.suptitle("Analytic vs. Learned stability margin\n 4 stance feet (1111)")
contacts = [1, 1, 1, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1111stance.txt')
learnedMargin.set_plot_properties()

fig2 = plt.figure(2)
fig2.suptitle("Analytic vs. Learned stability margin\n LF foot in swing (0111)")
contacts = [0, 1, 1, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('0111stance.txt')
learnedMargin.set_plot_properties()

fig3 = plt.figure(3)
fig3.suptitle("Analytic vs. Learned stability margin\n RF foot in swing (1011)")
contacts = [1, 0, 1, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1011stance.txt')
learnedMargin.set_plot_properties()

fig4 = plt.figure(4)
fig4.suptitle("Analytic vs. Learned stability margin\n LH foot in swing (1101)")
contacts = [1, 1, 0, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1101stance.txt')
learnedMargin.set_plot_properties()

fig5 = plt.figure(5)
fig5.suptitle("Analytic vs. Learned stability margin\n RH foot in swing (1110)")
contacts = [1, 1, 1, 0]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1110stance.txt')
learnedMargin.set_plot_properties()

fig6 = plt.figure(6)
fig6.suptitle("Analytic vs. Learned stability margin\n 2 stance feet (0110)")
contacts = [0, 1, 1, 0]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('0110stance.txt')
learnedMargin.set_plot_properties()

fig7 = plt.figure(7)
fig7.suptitle("Analytic vs. Learned stability margin\n 2 stance feet (1001)")
contacts = [1, 0, 0, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1001stance.txt')
learnedMargin.set_plot_properties()

plt.show()