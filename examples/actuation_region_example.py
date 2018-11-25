# -*- coding: utf-8 -*-
"""
Created on Thu Oct 25 11:44:22 2018

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
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters

    
''' MAIN '''

plt.close('all')
start_t_IPVC = time.time()

math = Math()
compDyn = ComputationalDynamics()
pathIP = PathIterativeProjection()
# number of contacts
number_of_contacts = 3

# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode = ['ONLY_ACTUATION',
                   'ONLY_ACTUATION',
                   'ONLY_ACTUATION',
                   'ONLY_ACTUATION']
useVariableJacobian = True
trunk_mass = 100
mu = 0.8

comTrajectoriesToStack = np.zeros((0,3))
terrain = HeightMap()
comWF = np.array([0.1, 0.1, 0.0])
optimizedVariablesToStack = np.zeros((0,3))
iterProj = PathIterativeProjection()        

axisZ= array([[0.0], [0.0], [1.0]])
math = Math()
n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
# %% Cell 2
normals = np.vstack([n1, n2, n3, n4])
ng = 4
#print "direction: ", desired_direction
""" contact points """
#LF_foot = np.array([0.4, 0.3, -0.5])
#RF_foot = np.array([0.4, -0.3, -0.5])
#terrainHeight = terrain.get_height(-.4, 0.3)
#LH_foot = np.array([-.4, 0.3, terrainHeight-0.5])     
#RH_foot = np.array([-0.3, -0.2, -0.5])

LF_foot = np.array([0.3, 0.2, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.3, 0.2, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.5])
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


LF_tau_lim = [50.0, 100.0, 100.0]
RF_tau_lim = [50.0, 100.0, 100.0]
LH_tau_lim = [50.0, 100.0, 100.0]
RH_tau_lim = [50.0, 100.0, 100.0]
torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
comWF = np.array([0.0,0.0,0.0])
    
tolerance = 0.005

params = IterativeProjectionParameters()
params.setContactsPos(contacts)
params.setCoMPos(comWF)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceLegs)
params.setConstraintModes(constraint_mode)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTrunkMass(trunk_mass)
    
for angle in np.arange(-np.pi, np.pi, 0.1):
    print 'new search angle: ', angle
    desired_direction = np.array([np.cos(angle), np.sin(angle), 0.0])
    newLimitPoint, stackedErrors = pathIP.find_vertex_along_path(params, desired_direction, tolerance)
    comTrajectoriesToStack = np.vstack([comTrajectoriesToStack, newLimitPoint])


print 'Errors convergence: ', stackedErrors

print("Actuation Region estimation time: --- %s seconds ---" % (time.time() - start_t_IPVC))

'''Plotting'''
plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")

plt.plot(comTrajectoriesToStack[:,0], comTrajectoriesToStack[:,1], 'g--', linewidth=2)
plt.plot(comTrajectoriesToStack[:,0], comTrajectoriesToStack[:,1], 'g^', markersize=20, label= 'Actuation region vertices')
plt.plot(comWF[0], comWF[1], 'ro', markersize=20, label= 'Initial CoM position used in the SIP alg.')
segment = np.vstack([comWF,newLimitPoint])
print int(stanceIndex[0])
print contacts[int(stanceIndex[0])][0]
contactsX = [contacts[int(stanceIndex[0])][0], contacts[int(stanceIndex[1])][0], contacts[int(stanceIndex[2])][0]]
contactsY = [contacts[int(stanceIndex[0])][1], contacts[int(stanceIndex[1])][1], contacts[int(stanceIndex[2])][0]]
print contactsX, contactsY
plt.plot(contactsX,contactsY,'ko',markersize=15, label='Stance feet')

plt.xlim(-0.8, 0.6)
plt.ylim(-0.7, 0.7)
plt.legend()
plt.show()

