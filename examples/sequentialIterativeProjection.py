# -*- coding: utf-8 -*-
"""
Created on Sat Aug  4 16:18:52 2018

@author: romeoorsolino
"""

import numpy as np

from context import legsthrust 

import time
import pylab
from pypoman.lp import solve_lp, GLPK_IF_AVAILABLE
#from pypoman.bretl import Vertex
#from pypoman.bretl import Polygon

import cvxopt
from cvxopt import matrix, solvers

from numpy import array, cos, cross, pi, sin
from numpy.random import random
from scipy.linalg import norm
from scipy.linalg import block_diag

from context import legsthrust 

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix

import matplotlib.pyplot as plt
from legsthrust.plotting_tools import Plotter
from legsthrust.constraints import Constraints
from legsthrust.hyq_kinematics import HyQKinematics
from legsthrust.math_tools import Math
from legsthrust.computational_dynamics import ComputationalDynamics
from legsthrust.vertex_based_projection import VertexBasedProjection

''' MAIN '''
start_t_IPVC = time.time()
math = Math()
compDyn = ComputationalDynamics()
# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION or ONLY_FRICTION
constraint_mode = 'ONLY_ACTUATION'
useVariableJacobian = True
trunk_mass = 100
mu = 0.8
# number of decision variables of the problem
n = nc*6

""" contact points """
LF_foot = np.array([0.3, 0.3, -0.58])
RF_foot = np.array([0.3, -0.3, -0.58])
LH_foot = np.array([-0.3, 0.3, -0.38])
RH_foot = np.array([-0.3, -0.2, -0.58])

contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts = contactsToStack[0:nc, :]


comWF = np.array([0.0, 0.0, 0.0])

''' plotting Iterative Projection points '''

axisZ= array([[0.0], [0.0], [1.0]])
n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2
        
normals = np.vstack([n1, n2, n3, n4])
feasible, unfeasible, contact_forces = compDyn.LP_projection(constraint_mode, contacts, normals, trunk_mass, mu, ng, nc, mu, useVariableJacobian, 0.05, 0.05)

desired_direction = [-1.0, -1.0]

IP_points1, actuation_polygons = compDyn.iterative_projection_bretl(constraint_mode, contacts, normals, trunk_mass, ng, mu, comWF)

comWF = np.array([-0.205, 0.1, 0.0])
IP_points2, actuation_polygons = compDyn.iterative_projection_bretl(constraint_mode, contacts, normals, trunk_mass, ng, mu, comWF)

comWF = np.array([-0.131, 0.1, 0.0])
IP_points3, actuation_polygons = compDyn.iterative_projection_bretl(constraint_mode, contacts, normals, trunk_mass, ng, mu, comWF)

comWF = np.array([-0.17, 0.1, 0.0])
IP_points4, actuation_polygons = compDyn.iterative_projection_bretl(constraint_mode, contacts, normals, trunk_mass, ng, mu, comWF)

comWF = np.array([-0.18, 0.1, 0.0])
IP_points5, actuation_polygons = compDyn.iterative_projection_bretl(constraint_mode, contacts, normals, trunk_mass, ng, mu, comWF)

comWF = np.array([-0.19, 0.1, 0.0])
IP_points6, actuation_polygons = compDyn.iterative_projection_bretl(constraint_mode, contacts, normals, trunk_mass, ng, mu, comWF)
#IP_points_saturated_friction, actuation_polygons = compDyn.iterative_projection_bretl('ONLY_FRICTION', contacts, normals, trunk_mass, ng, mu, saturate_normal_force = True)

'''Plotting'''
plt.close('all')
plotter = Plotter()

plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
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

for j in range(0, unfeasiblePointsSize):
    if (unfeasible[j,2]<0.01)&(unfeasible[j,2]>-0.01):
        plt.scatter(unfeasible[j,0], unfeasible[j,1],c='r',s=50)
        lastUnfeasibleIndex = j
h2 = plt.scatter(feasible[lastFeasibleIndex,0], feasible[lastFeasibleIndex,1],c='g',s=50, label='LP feasible')
h3 = plt.scatter(unfeasible[lastUnfeasibleIndex,0], unfeasible[lastUnfeasibleIndex,1],c='r',s=50, label='LP unfeasible')


print "final vertices: ", vx
print "number of vertices: ", np.size(vx, 0)
plt.legend()
plt.show()

