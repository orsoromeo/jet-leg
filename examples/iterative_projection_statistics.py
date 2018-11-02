# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""


import numpy as np

from context import jet_leg 

from numpy import array
from numpy.linalg import norm
from jet_leg.plotting_tools import Plotter

from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics

import matplotlib as mpl
import matplotlib.pyplot as plt
        
plt.close('all')
math = Math()

# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION or ONLY_FRICTION
constraint_mode_IP = 'ONLY_ACTUATION'
useVariableJacobian = False
# number of decision variables of the problem
n = nc*6

''' parameters to be tuned'''
g = 9.81
trunk_mass = 85.
mu = 0.8
    
axisZ= array([[0.0], [0.0], [1.0]])

comp_dyn = ComputationalDynamics()
number_of_tests = 50
tests3contacts = np.zeros((number_of_tests))
tests4contacts = np.zeros((number_of_tests))  

for iter in range(0,number_of_tests):
    
    ''' random normals '''    
    randRoll = np.random.normal(0.0, 0.2)
    randPitch = np.random.normal(0.0, 0.2)
    randYaw = np.random.normal(0.0, 0.2)
    n1 = np.transpose(np.transpose(math.rpyToRot(randRoll,randPitch,randYaw)).dot(axisZ))
    randRoll = np.random.normal(0.0, 0.2)
    randPitch = np.random.normal(0.0, 0.2)
    randYaw = np.random.normal(0.0, 0.2)
    n2 = np.transpose(np.transpose(math.rpyToRot(randRoll,randPitch,randYaw)).dot(axisZ))
    randRoll = np.random.normal(0.0, 0.2)
    randPitch = np.random.normal(0.0, 0.2)
    randYaw = np.random.normal(0.0, 0.2)
    n3 = np.transpose(np.transpose(math.rpyToRot(randRoll,randPitch,randYaw)).dot(axisZ))
    normals = np.vstack([n1, n2, n3])
    
    """ contact points """
    sigma = 0.05 # mean and standard deviation
    randX = np.floor(np.random.normal(30, sigma))
    randY = np.floor(np.random.normal(20, sigma))
    LF_foot = np.array([randX/100, randY/100, -0.5])
    RF_foot = np.array([randX/100, -randY/100, -0.5])
    LH_foot = np.array([-randX/100, randY/100, -0.5])
    RH_foot = np.array([-randX/100, -randY/100, -0.5])
    
    contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
#    contacts = contactsToStack[0:nc, :]

    ''' compute iterative projection '''
    stanceLegs = [1 ,1, 1, 0]
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(constraint_mode_IP, stanceLegs, contacts, normals, trunk_mass, ng, mu)
    comp_time = comp_time * 1000.0
    tests3contacts[iter] = comp_time

nc = 4
LF_foot = np.array([0.3, 0.2, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.2, 0.0, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.5])
    
contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts = contactsToStack[0:nc, :]
for iter in range(0,number_of_tests):
    
    ''' random normals '''    
    randRoll = np.random.normal(0.0, 0.2)
    randPitch = np.random.normal(0.0, 0.2)
    randYaw = np.random.normal(0.0, 0.2)
    n1 = np.transpose(np.transpose(math.rpyToRot(randRoll,randPitch,randYaw)).dot(axisZ))
    randRoll = np.random.normal(0.0, 0.2)
    randPitch = np.random.normal(0.0, 0.2)
    randYaw = np.random.normal(0.0, 0.2)
    n2 = np.transpose(np.transpose(math.rpyToRot(randRoll,randPitch,randYaw)).dot(axisZ))
    randRoll = np.random.normal(0.0, 0.2)
    randPitch = np.random.normal(0.0, 0.2)
    randYaw = np.random.normal(0.0, 0.2)
    n3 = np.transpose(np.transpose(math.rpyToRot(randRoll,randPitch,randYaw)).dot(axisZ))
    randRoll = np.random.normal(0.0, 0.2)
    randPitch = np.random.normal(0.0, 0.2)
    randYaw = np.random.normal(0.0, 0.2)
    n4 = np.transpose(np.transpose(math.rpyToRot(randRoll,randPitch,randYaw)).dot(axisZ))
    normals = np.vstack([n1, n2, n3, n4])
    
    """ contact points """
    sigma = 0.05 # mean and standard deviation
    randX = np.floor(np.random.normal(30, sigma))
    randY = np.floor(np.random.normal(20, sigma))
    LF_foot = np.array([randX/100, randY/100, -0.5])
    RF_foot = np.array([randX/100, -randY/100, -0.5])
    LH_foot = np.array([-0.2, 0.0, -0.5])
    RH_foot = np.array([-0.3, -0.2, -0.5])
    
    contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
    contacts = contactsToStack[0:nc, :]

    stanceLegs = [1 ,1, 1, 1]
    ''' compute iterative projection '''
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(constraint_mode_IP, stanceLegs, contacts, normals, trunk_mass, ng, mu)
    comp_time = comp_time * 1000.0
    tests4contacts[iter] = comp_time
    
''' plotting Iterative Projection points '''
plotter = Plotter()

''' 2D figure '''
    
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()

fig = plt.figure()
plt.plot([1,2,3])
plt.subplot(121)
#print tests
plt.grid()
plt.hist(tests3contacts,bins=np.arange(0,20,0.1))
plt.title("Histogram on 1000 tests of the IP with 3 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")

plt.subplot(122)
#print tests
plt.grid()
plt.hist(tests4contacts,bins=np.arange(0,20,0.1))
plt.title("Histogram on 1000 tests of the IP with 3 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")

plt.show()
mpl.rcParams.update({'font.size': 15})
fig.savefig('../figs/initial_tests_IP/histogramIP.pdf')