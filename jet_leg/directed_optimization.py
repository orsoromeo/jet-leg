# -*- coding: utf-8 -*-
"""
Created on Sat Aug  4 21:57:55 2018

@author: romeoorsolino
"""
import numpy as np

#from context import jet_leg 

import time

from pypoman.lp import solve_lp, GLPK_IF_AVAILABLE
from pypoman.bretl import Vertex
from pypoman.bretl import Polygon

import random
import cvxopt
from cvxopt import matrix, solvers

from numpy import array, cos, cross, pi, sin
from numpy.random import random
from scipy.linalg import norm
from scipy.linalg import block_diag

from numpy import array, cross, dot, eye, hstack, vstack, zeros

import matplotlib.pyplot as plt
from jet_leg.plotting_tools import Plotter
from jet_leg.constraints import Constraints
from jet_leg.hyq_kinematics import HyQKinematics
from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.height_map import HeightMap

from path_sequential_iterative_projection import PathIterativeProjection
        
    
''' MAIN '''
start_t_IPVC = time.time()
math = Math()
compDyn = ComputationalDynamics()
pathIP = PathIterativeProjection()
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
i = 0
comTrajectoriesToStack = np.zeros((0,3))
terrain = HeightMap()

comWF = np.array([0.1, 0.1, 0.0])
optimizedVariablesToStack = np.zeros((0,3))
iterProj = PathIterativeProjection()        
for_iter = 0
for LH_x in np.arange(-0.8,-0.3, 0.1):
    for LH_y in np.arange(0.1,0.4, 0.1):
        for dir_y in np.arange(-0.2,0.0,0.2):
            desired_direction = np.array([-1.0, dir_y, 0.0])
            #print "direction: ", desired_direction
            """ contact points """
            LF_foot = np.array([0.4, 0.3, -0.5])
            RF_foot = np.array([0.4, -0.3, -0.5])
            terrainHeight = terrain.get_height(LH_x, LH_y)
            LH_foot = np.array([LH_x, LH_y, terrainHeight-0.5])
            #print "Terrain height: ", LH_foot        
            RH_foot = np.array([-0.3, -0.2, -0.5])
    
            contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
            contacts = contactsToStack[0:nc, :]
            print contacts
            
            newLimitPoint = pathIP.find_vertex_along_path(constraint_mode, contacts, comWF, desired_direction)

            for_iter += 1
            #print "for ",for_iter
            comTrajectoriesToStack = np.vstack([comTrajectoriesToStack, newLimitPoint])
            optimizedVariablesToStack = np.vstack([optimizedVariablesToStack, np.array([LH_x, LH_y, dir_y])])

print "Final CoM points ", comTrajectoriesToStack
print "Tested combinations: ", optimizedVariablesToStack

max_motion_indices = np.unravel_index(np.argsort(comTrajectoriesToStack[:,0], axis=None), comTrajectoriesToStack[:,0].shape)
max_motin_index = max_motion_indices[0][0]
print("Directed Iterative Projection: --- %s seconds ---" % (time.time() - start_t_IPVC))


''' plotting Iterative Projection points '''

axisZ= array([[0.0], [0.0], [1.0]])
n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2
        
normals = np.vstack([n1, n2, n3, n4])
IP_points, actuation_polygons, comp_time = compDyn.iterative_projection_bretl(constraint_mode, contacts, normals, trunk_mass, ng, mu)

#IP_points_saturated_friction, actuation_polygons = compDyn.iterative_projection_bretl('ONLY_FRICTION', contacts, normals, trunk_mass, ng, mu, saturate_normal_force = True)

'''Plotting'''
plt.close('all')
plotter = Plotter()

plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")

plt.plot(comTrajectoriesToStack[:,0], comTrajectoriesToStack[:,1], 'g^', markersize=20)
plt.plot(comTrajectoriesToStack[max_motin_index,0], comTrajectoriesToStack[max_motin_index,1], 'y^', markersize=25)
h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')

plt.plot(optimizedVariablesToStack[:,0], optimizedVariablesToStack[:,1], 'ko', markersize = 15)
plt.plot(optimizedVariablesToStack[max_motin_index,0], optimizedVariablesToStack[max_motin_index,1], 'yo', markersize=25)
plt.xlim(-0.9, 0.5)
plt.ylim(-0.7, 0.7)
plt.legend()
plt.show()

