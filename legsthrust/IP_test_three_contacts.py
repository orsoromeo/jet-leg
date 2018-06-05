# -*- coding: utf-8 -*-
"""
Created on Sat May  5 14:35:48 2018

@author: Romeo Orsolino
"""

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
from computational_dynamics import ComputationalDynamics as CompDyn

#pylab.close("all")
    
    # pypoman.plot_polygon(points)

# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

constraint_mode = 'only_friction'


# contact positions
""" contact points """
LF_foot = np.array([0.3, 0.2, -.5])
RF_foot = np.array([0.3, -0.2, -.5])
LH_foot = np.array([-0.3, 0.2, -.5])
RH_foot = np.array([-0.3, -0.2, -.5])
contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

normal1 = array([0.0, 0.0, 1.0])
normal2 = array([0.0, 0.0, 1.0])
normal3 = array([0.0, 0.0, 1.0])
normals  = np.vstack([normal1 ,normal2, normal3])
''' parameters to be tuned'''
mass = 10.
mu = 1.

comp_dyn = CompDyn()
comp_dyn.iterative_projection_bretl(constraint_mode, contacts, normals, mu, mass, nc, ng)