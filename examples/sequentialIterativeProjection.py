# -*- coding: utf-8 -*-
"""
Created on Sat Aug  4 16:18:52 2018

@author: romeoorsolino
"""

import numpy as np
from context import jet_leg 
import time

import cvxopt
from cvxopt import matrix, solvers

from numpy import array, cos, cross, pi, sin
from numpy.random import random
from scipy.linalg import norm
from scipy.linalg import block_diag

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix

import matplotlib.pyplot as plt
from jet_leg.plotting_tools import Plotter
from jet_leg.constraints import Constraints
from jet_leg.hyq_kinematics import HyQKinematics
from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.vertex_based_projection import VertexBasedProjection
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters

''' MAIN '''
start_t_IPVC = time.time()
math = Math()
compDyn = ComputationalDynamics()
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION or ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode_IP = ['ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION']
useVariableJacobian = True
g = 9.81
trunk_mass = 85.
mu = 0.9


""" contact points """
LF_foot = np.array([0.3, 0.2, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.3, 0.2, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.5])

contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
stanceFeet = [1,1,1,1]
nc = np.sum(stanceFeet)
comWF = np.array([0.0, 0.0, 0.0])

''' plotting Iterative Projection points '''

axisZ= array([[0.0], [0.0], [1.0]])
n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2

normals = np.vstack([n1, n2, n3, n4])

LF_tau_lim = [50.0, 100.0, 100.0]
RF_tau_lim = [50.0, 100.0, 100.0]
LH_tau_lim = [50.0, 100.0, 100.0]
RH_tau_lim = [50.0, 100.0, 100.0]
torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
#comWF = np.array([0.0,0.0,0.0])
params = IterativeProjectionParameters()
params.setContactsPos(contacts)
params.setCoMPos(comWF)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTrunkMass(trunk_mass)

feasible, unfeasible, contact_forces = compDyn.LP_projection(params, useVariableJacobian, 0.2, 0.2)
print feasible
print unfeasible
#desired_direction = [-1.0, -1.0]

IP_points1, actuation_polygons, comp_time = compDyn.iterative_projection_bretl(params)

comWF = np.array([-0.205, 0.1, 0.0])
params.setCoMPos(comWF)
IP_points2, actuation_polygons, comp_time = compDyn.iterative_projection_bretl(params)

comWF = np.array([-0.131, 0.1, 0.0])
params.setCoMPos(comWF)
IP_points3, actuation_polygons, comp_time = compDyn.iterative_projection_bretl(params)

comWF = np.array([-0.17, 0.1, 0.0])
params.setCoMPos(comWF)
IP_points4, actuation_polygons, comp_time = compDyn.iterative_projection_bretl(params)

comWF = np.array([-0.18, 0.1, 0.0])
params.setCoMPos(comWF)
IP_points5, actuation_polygons, comp_time = compDyn.iterative_projection_bretl(params)

comWF = np.array([-0.19, 0.1, 0.0])
params.setCoMPos(comWF)
IP_points6, actuation_polygons, comp_time = compDyn.iterative_projection_bretl(params)
#IP_points_saturated_friction, actuation_polygons = compDyn.iterative_projection_bretl('ONLY_FRICTION', contacts, normals, trunk_mass, ng, mu, saturate_normal_force = True)

'''Plotting'''
plt.close('all')
plotter = Plotter()

plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
contacts = params.getContactsPos()
print nc, contacts
h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')

plotter.plot_polygon(np.transpose(IP_points1), color = 'b')
plotter.plot_polygon(np.transpose(IP_points2), color = 'g')
plotter.plot_polygon(np.transpose(IP_points3), color = 'r')
plotter.plot_polygon(np.transpose(IP_points4), color = 'y')
plotter.plot_polygon(np.transpose(IP_points5), color = 'k')
plotter.plot_polygon(np.transpose(IP_points6), color = 'b')
#plotter.plot_polygon(np.transpose(IP_points_saturated_friction), color = '--r')

feasiblePointsSize = np.size(feasible,0)
for j in range(0, feasiblePointsSize):
    if (feasible[j,2]<0.01)&(feasible[j,2]>-0.01):
        plt.scatter(feasible[j,0], feasible[j,1],c='g',s=50)
        lastFeasibleIndex = j
unfeasiblePointsSize = np.size(unfeasible,0)
print unfeasiblePointsSize
for j in range(0, unfeasiblePointsSize):
    if (unfeasible[j,2]<0.01)&(unfeasible[j,2]>-0.01):
        plt.scatter(unfeasible[j,0], unfeasible[j,1],c='r',s=50)
        lastUnfeasibleIndex = j
#h2 = plt.scatter(feasible[lastFeasibleIndex,0], feasible[lastFeasibleIndex,1],c='g',s=50, label='LP feasible')
#h3 = plt.scatter(unfeasible[lastUnfeasibleIndex,0], unfeasible[lastUnfeasibleIndex,1],c='r',s=50, label='LP unfeasible')

#
#print "final vertices: ", vx
#print "number of vertices: ", np.size(vx, 0)
plt.legend()
plt.show()

