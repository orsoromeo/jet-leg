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
from plot_learned_com_acc_margin_jac_xy import LearnedAccMargin

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

    delta_acc_range = 20.0
    delta_acc_range_z = 10.0
    dx = 0.2
    num_of_tests = delta_acc_range/dx
    delta_range_vec_x = np.linspace(-delta_acc_range/2.0, delta_acc_range/2.0, num_of_tests)
    delta_range_vec_y = np.linspace(-delta_acc_range/2.0, delta_acc_range/2.0, num_of_tests)
    delta_range_vec_z = np.linspace(-delta_acc_range_z/2.0, delta_acc_range_z/2.0, num_of_tests)
    print "number of tests", num_of_tests

    pos_margin_x, jac_com_pos_x = jac.plotMarginAndJacobianWrtComLinAcceleration(params_com_x, delta_range_vec_x, 0) # dm / dx
    pos_margin_y, jac_com_pos_y = jac.plotMarginAndJacobianWrtComLinAcceleration(params_com_y, delta_range_vec_y, 1) # dm / dy
    pos_margin_z, jac_com_pos_z = jac.plotMarginAndJacobianWrtComLinAcceleration(params_com_z, delta_range_vec_z, 2) # dm / dz

    return pos_margin_x, jac_com_pos_x, pos_margin_y, jac_com_pos_y, pos_margin_z, jac_com_pos_z, delta_range_vec_x, delta_range_vec_y, delta_range_vec_z


def plotAnalyticMarginAndDerivatives(pos_margin_x, jac_com_pos_x, pos_margin_y, jac_com_pos_y, pos_margin_z, jac_com_pos_z, delta_pos_range_vec_x, delta_pos_range_vec_y, delta_pos_range_vec_z ):
    plt.subplot(321)
    plt.plot(delta_pos_range_vec_x, pos_margin_x, 'r-o', markersize=2, label='analytic (iterative projection)')

    plt.subplot(322)
    plt.plot(delta_pos_range_vec_y, pos_margin_y, 'r-o', markersize=2, label='analytic (iterative projection)')

    plt.subplot(323)
    plt.plot(delta_pos_range_vec_x, jac_com_pos_x[0, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(325)
    plt.plot(delta_pos_range_vec_x, jac_com_pos_x[1, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(324)
    plt.plot(delta_pos_range_vec_y, jac_com_pos_y[0, :], 'r-o', markersize=2, label='analytic (finite-diff)')

    plt.subplot(326)
    plt.plot(delta_pos_range_vec_y, jac_com_pos_y[1, :], 'r-o', markersize=2, label='analytic (finite-diff)')



### Plotting

### X axis
robot_name = 'anymal_coyote'
folder = 'com_acceleration'
learnedMargin = LearnedAccMargin()

fig1 = plt.figure(1)
fig1.suptitle("Analytic vs. Learned stability margin\n 4 stance feet (1111)")
contacts = [1, 1, 1, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1111stance.txt')
learnedMargin.set_plot_properties()
fig1.savefig('../../figs/1111stance_acc.pdf')

fig2 = plt.figure(2)
fig2.suptitle("Analytic vs. Learned stability margin\n LF foot in swing (0111)")
contacts = [0, 1, 1, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('0111stance.txt')
learnedMargin.set_plot_properties()
fig2.savefig('../../figs/0111stance_acc.pdf')

fig3 = plt.figure(3)
fig3.suptitle("Analytic vs. Learned stability margin\n RF foot in swing (1011)")
contacts = [1, 0, 1, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1011stance.txt')
learnedMargin.set_plot_properties()
fig3.savefig('../../figs/1011stance_acc.pdf')

fig4 = plt.figure(4)
fig4.suptitle("Analytic vs. Learned stability margin\n LH foot in swing (1101)")
contacts = [1, 1, 0, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1101stance.txt')
learnedMargin.set_plot_properties()
fig4.savefig('../../figs/1101stance_acc.pdf')

fig5 = plt.figure(5)
fig5.suptitle("Analytic vs. Learned stability margin\n RH foot in swing (1110)")
contacts = [1, 1, 1, 0]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1110stance.txt')
learnedMargin.set_plot_properties()
fig5.savefig('../../figs/1110stance_acc.pdf')

fig6 = plt.figure(6)
fig6.suptitle("Analytic vs. Learned stability margin\n 2 stance feet (0110)")
contacts = [0, 1, 1, 0]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('0110stance.txt')
learnedMargin.set_plot_properties()
fig6.savefig('../../figs/0110stance_acc.pdf')

fig7 = plt.figure(7)
fig7.suptitle("Analytic vs. Learned stability margin\n 2 stance feet (1001)")
contacts = [1, 0, 0, 1]
mx, jx, my, jy, mz, jz, vx, vy, vz = computeAnalyticMarginAndDerivatives(contacts, robot_name)
plotAnalyticMarginAndDerivatives(mx, jx, my, jy, mz, jz, vx, vy, vz)
learnedMargin.plot_learned_margin('1001stance.txt')
learnedMargin.set_plot_properties()
fig7.savefig('../../figs/1001stance_acc.pdf')

plt.show()
