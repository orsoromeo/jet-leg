# -*- coding: utf-8 -*-
"""
Created on Thu Oct 25 10:58:03 2018

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
start_t_IPVC = time.time()

math = Math()
compDyn = ComputationalDynamics()
pathIP = PathIterativeProjection()
# number of contacts
number_of_contacts = 3

# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode = 'FRICTION_AND_ACTUATION'
useVariableJacobian = True
trunk_mass = 100
mu = 0.8

terrain = HeightMap()
comWF = np.array([0.1, 0.1, 0.0])
#iterProj = PathIterativeProjection()        

#print "direction: ", desired_direction
""" contact points """
LF_foot = np.array([0.4, 0.3, -0.5])
RF_foot = np.array([0.4, -0.3, -0.5])
terrainHeight = terrain.get_height(-.4, 0.3)
LH_foot = np.array([-.4, 0.3, terrainHeight-0.5])     
RH_foot = np.array([-0.3, -0.2, -0.5])

angle = np.random.normal(np.pi, 0.2)
desired_direction = np.array([np.cos(angle), np.sin(angle), 0.0])
contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

stanceLegs = [1,1,1,1]
stanceIndex = []
swingIndex = []
print 'stance', stanceLegs
for iter in range(0, 4):
    if stanceLegs[iter] == 1:
#               print 'new poly', stanceIndex, iter
        stanceIndex = np.hstack([stanceIndex, iter])
    else:
        swingIndex = iter

print stanceIndex, swingIndex

LF_tau_lim = [50.0, 100.0, 100.0]
RF_tau_lim = [50.0, 100.0, 100.0]
LH_tau_lim = [50.0, 100.0, 100.0]
RH_tau_lim = [50.0, 100.0, 100.0]
torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
comWF = np.array([0.0,0.0,0.0])

tolerance = 0.005
newLimitPoint, stackedErrors = pathIP.find_vertex_along_path(constraint_mode, contacts, comWF, desired_direction, stanceLegs, stanceIndex, swingIndex, torque_limits, tolerance)

print 'Errors convergence: ', stackedErrors

print("Path Sequential Iterative Projection: --- %s seconds ---" % (time.time() - start_t_IPVC))


'''Plotting Fig 1'''
plt.close('all')
plotter = Plotter()

plt.figure()
plt.grid()
plt.ylabel("Absolute error [cm]")
plt.xlabel("Iterations number")
plt.plot(np.abs(stackedErrors[:,0]), 'b', linewidth=2, label = 'X direction')
plt.plot(np.abs(stackedErrors[:,1]), 'r', linewidth=2, label = 'Y direction')
plt.legend()

'''Plotting Fig 2'''
plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")

plt.plot(newLimitPoint[0], newLimitPoint[1], 'g^', markersize=20, label= 'Actuation region vertex')
plt.plot(comWF[0], comWF[1], 'ro', markersize=20, label= 'Initial CoM position used in the SIP alg.')
segment = np.vstack([comWF,newLimitPoint])
plt.plot(segment[:,0], segment[:,1], 'b-', linewidth=2, label= 'Search direction')
plt.plot(contacts[0:number_of_contacts,0],contacts[0:number_of_contacts,1],'ko',markersize=15, label='Stance feet')

plt.xlim(-0.9, 0.5)
plt.ylim(-0.7, 0.7)
plt.legend()
plt.show()

