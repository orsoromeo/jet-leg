# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: rorsolino
"""

import time
import pylab
import pypoman
import numpy as np

from numpy import array, cross, dot, eye, hstack, vstack, zeros
from numpy.linalg import norm
from scipy.linalg import block_diag
from plotting_tools import Plotter
from constraints import Constraints
from kinematics import Kinematics
from math_tools import Math
from computational_dynamics import ComputationalDynamics
from anaylitic_test_three_contacts import AnalyticProjection

import matplotlib.pyplot as plt
from arrow3D import Arrow3D

plt.close('all')

# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

constraint_mode = 'only_friction'
# number of decision variables of the problem
n = nc*6

# contact positions
""" contact points """
LF_foot = np.array([0.3, 0.2, -.0])
RF_foot = np.array([0.3, -0.3, -.0])
LH_foot = np.array([-0.3, 0.2, -.0])
RH_foot = np.array([-0.3, -0.2, -.0])
contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

''' parameters to be tuned'''
g = 9.81
mass = 10.
mu = 1.

n1 = array([0.0, 0.0, 1.0])
n2 = array([0.0, 0.0, 1.0])
n3 = array([0.0, 0.0, 1.0])
math = Math()
n1, n2, n3 = (math.normalize(n) for n in [n1, n2, n3])
normals = np.vstack([n1, n2, n3])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.plot(contacts[0:3,0],contacts[0:3,1],'ko',markersize=15)
for j in range(0,nc):
    ax.scatter(contacts[j,0], contacts[j,1], contacts[j,2],c='b',s=100)
    a = Arrow3D([contacts[j,0], contacts[j,0]+normals[j,0]/10], [contacts[j,1], contacts[j,1]+normals[j,1]/10],[contacts[j,2], contacts[j,2]+normals[j,2]/10], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
    ax.add_artist(a)

comp_dyn = ComputationalDynamics()
IP_points = comp_dyn.iterative_projection_bretl(constraint_mode, contacts, normals, mass, ng, mu)

''' plotting Iterative Projection points '''

plotter = Plotter()
plotter.plot_polygon(np.transpose(IP_points))
#
feasible, unfeasible = comp_dyn.LP_projection(constraint_mode, contacts, normals, mass, mu, ng, nc, mu)

''' plotting LP test points '''
if np.size(feasible,0) != 0:
    ax.scatter(feasible[:,0], feasible[:,1], feasible[:,2],c='g',s=50)
if np.size(unfeasible,0) != 0:
    ax.scatter(unfeasible[:,0], unfeasible[:,1], unfeasible[:,2],c='r',s=50)

an_proj = AnalyticProjection()
vertices2d, simplices = an_proj.analytic_projection(constraint_mode, contacts, normals, mass, ng, mu)

for simplex in simplices:
    plt.plot(vertices2d[simplex, 0], vertices2d[simplex, 1], 'y-', linewidth=5.)


plt.show()