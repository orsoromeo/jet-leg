# -*- coding: utf-8 -*-
"""
Created on Thu Oct 25 12:06:27 2018

@author: rorsolino
"""

import numpy as np
from context import jet_leg 
import time
from numpy import array

import matplotlib.pyplot as plt
from jet_leg.plotting_tools import Plotter
from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.height_map import HeightMap

from jet_leg.path_sequential_iterative_projection import PathIterativeProjection

    
''' MAIN '''

plt.close('all')
start_t_IPVC = time.time()

math = Math()
compDyn = ComputationalDynamics()
pathIP = PathIterativeProjection()
# number of contacts
number_of_contacts = 3

# ONLY_ACTUATION or ONLY_FRICTION
constraint_mode = 'ONLY_ACTUATION'
useVariableJacobian = True
trunk_mass = 100
mu = 0.8

comTrajectoriesToStack = np.zeros((0,3))
terrain = HeightMap()
comWF = np.array([0.1, 0.1, 0.0])
optimizedVariablesToStack = np.zeros((0,3))
iterProj = PathIterativeProjection()        

tolerance = 0.005

for LH_x in np.arange(-0.7,-0.2, 0.1):
    for LH_y in np.arange(-0.1,0.3, 0.1):
        angle = np.random.normal(np.pi, 0.1)
        desired_direction = np.array([np.cos(angle), np.sin(angle), 0.0])

        """ contact points """
        LF_foot = np.array([0.4, 0.3, -0.5])
        RF_foot = np.array([0.4, -0.3, -0.5])
        terrainHeight = terrain.get_height(LH_x, LH_y)
        LH_foot = np.array([LH_x, LH_y, terrainHeight-0.5])
        RH_foot = np.array([-0.3, -0.2, -0.5])
        contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        contacts = contactsToStack[0:number_of_contacts, :]
        print 'new search angle: ', angle
        desired_direction = np.array([np.cos(angle), np.sin(angle), 0.0])
        newLimitPoint, stackedErrors = pathIP.find_vertex_along_path(constraint_mode, contacts, comWF, desired_direction, tolerance)
        comTrajectoriesToStack = np.vstack([comTrajectoriesToStack, newLimitPoint])
        optimizedVariablesToStack = np.vstack([optimizedVariablesToStack, np.array([LH_x, LH_y, angle])])


print 'Errors convergence: ', stackedErrors

print("Actuation Region estimation time: --- %s seconds ---" % (time.time() - start_t_IPVC))

max_motion_indices = np.unravel_index(np.argsort(comTrajectoriesToStack[:,0], axis=None), comTrajectoriesToStack[:,0].shape)
max_motin_index = max_motion_indices[0][0]

'''Plotting'''
plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")

plt.plot(comTrajectoriesToStack[:,0], comTrajectoriesToStack[:,1], 'g^', markersize=20, label= 'Actuation region vertices')
plt.plot(comWF[0], comWF[1], 'ro', markersize=20, label= 'Initial CoM position used in the SIP alg.')
segment = np.vstack([comWF,newLimitPoint])
plt.plot(contacts[0:number_of_contacts,0],contacts[0:number_of_contacts,1],'ko',markersize=15, label='Stance feet')
plt.plot(comTrajectoriesToStack[max_motin_index,0], comTrajectoriesToStack[max_motin_index,1], 'y^', markersize=25, label= 'furthest vertex along the given search direction')
plt.plot(optimizedVariablesToStack[:,0], optimizedVariablesToStack[:,1], 'bo', markersize = 15, label = 'tested stance feet')
plt.plot(optimizedVariablesToStack[max_motin_index,0], optimizedVariablesToStack[max_motin_index,1], 'yo', markersize=20, label='selected foothold')

plt.xlim(-0.9, 0.5)
plt.ylim(-0.7, 0.7)
plt.legend()
plt.show()

