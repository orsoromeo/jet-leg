# -*- coding: utf-8 -*-
"""
Created on Sat May  5 14:35:48 2018

@author: Romeo Orsolino
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


start_time = time.time()
# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 8

constraint_mode = 'only_friction'
# number of decision variables of the problem
n = nc*6

# contact positions
""" contact points """
LF_foot = np.array([0.3, 0.2, -.5])
print LF_foot
RF_foot = np.array([0.3, -0.2, -.5])
LH_foot = np.array([-0.3, 0.2, -.5])
RH_foot = np.array([-0.3, -0.2, -.5])
contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

''' parameters to be tuned'''
g = 9.81
mass = 100.
mu = 1.

n1 = array([0.0, 0.0, 1.0])
n2 = array([0.0, 0.0, 1.0])
n3 = array([0.0, 0.0, 1.0])
math = Math()
n1, n2, n3 = (math.normalize(n) for n in [n1, n2, n3])
normals = np.vstack([n1, n2, n3])

comp_dyn = ComputationalDynamics()
comp_dyn.iterative_projection_bretl(constraint_mode, contacts, normals, mass, ng, mu)

print("--- %s seconds ---" % (time.time() - start_time))

# pypoman.plot_polygon(points)
