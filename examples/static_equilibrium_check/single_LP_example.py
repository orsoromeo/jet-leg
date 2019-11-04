# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import time
import pylab
import pypoman
import numpy as np

from context import jet_leg 

from numpy import array
from numpy.linalg import norm
from jet_leg.plotting_tools import Plotter
import random
from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters

import matplotlib.pyplot as plt
from jet_leg.arrow3D import Arrow3D
        
plt.close('all')
math = Math()
# number of contacts
#nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION

constraint_mode_IP = ['ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION']
useVariableJacobian = False

# contact positions
""" contact points """

LF_foot = np.array([0.3, 0.2, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.3, 0.2, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.5])

contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

''' parameters to be tuned'''
g = 9.81
trunk_mass = 25.
mu = 0.9

stanceFeet = [1,1,1,1]
randomSwingLeg = random.randint(0,3)
print 'Swing leg', randomSwingLeg
print 'stanceLegs ' ,stanceFeet

axisZ= array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
normals = np.vstack([n1, n2, n3, n4])

LF_tau_lim = [50.0, 100.0, 100.0]
RF_tau_lim = [50.0, 100.0, 100.0]
LH_tau_lim = [50.0, 100.0, 100.0]
RH_tau_lim = [50.0, 100.0, 100.0]
torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])

comWF = np.array([0.0,-0.3,0.0])
#
fig = plt.figure()
nc = np.sum(stanceFeet)
plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)

comp_dyn = ComputationalDynamics()
params = IterativeProjectionParameters()

params.setContactsPosBF(contacts)
params.setCoMPosWF(comWF)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(trunk_mass)

''' compute iterative projection '''
feasible_points = np.zeros((0,3))
unfeasible_points = np.zeros((0,3))
contact_forces = np.zeros((0,nc*3))  
LP_actuation_polygons, feasible_points, unfeasible_points, contact_forces = comp_dyn.compute_lp(params, feasible_points, unfeasible_points, contact_forces, comWF)
print 'is point feasible?', feasible_points
#print IP_points

''' plotting Iterative Projection points '''
feasiblePointsSize = np.size(feasible_points,0)
for j in range(0, feasiblePointsSize):
    plt.scatter(feasible_points[j,0], feasible_points[j,1],c='g',s=50)
    lastFeasibleIndex = j
unfeasiblePointsSize = np.size(unfeasible_points,0)

for j in range(0, unfeasiblePointsSize):
    plt.scatter(unfeasible_points[j,0], unfeasible_points[j,1],c='r',s=50)
    lastUnfeasibleIndex = j

    
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
#plt.legend()
plt.show()