# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""


import numpy as np

from context import jet_leg 

from numpy import array
from numpy.linalg import norm
from jet_leg.plotting.plotting_tools import Plotter

from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters

import matplotlib as mpl
import matplotlib.pyplot as plt
import random
        
plt.close('all')
math = Math()

# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                      'ONLY_FRICTION',
                      'ONLY_ACTUATION',
                      'FRICTION_AND_ACTUATION']

useVariableJacobian = False

''' parameters to be tuned'''
g = 9.81
total_mass = 85.
mu = 0.8
    
axisZ= array([[0.0], [0.0], [1.0]])

robot_name = 'anymal'
comp_dyn = ComputationalDynamics(robot_name)

number_of_tests = 100
tests3contacts = np.zeros((number_of_tests))
tests4contacts = np.zeros((number_of_tests))  

params = IterativeProjectionParameters(robot_name)

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
    randX = np.random.normal(0.3, sigma)
    randY = np.random.normal(0.2, sigma)
    randZ = np.random.normal(-0.5, sigma)
    LF_foot = np.array([randX, randY, randZ])
    randX = np.random.normal(0.3, sigma)
    randY = np.random.normal(-0.2, sigma)
    randZ = np.random.normal(-0.5, sigma)
    RF_foot = np.array([randX, randY, randZ])
    randX = np.random.normal(-0.3, sigma)
    randY = np.random.normal(0.2, sigma)
    randZ = np.random.normal(-0.5, sigma)
    LH_foot = np.array([randX, randY, randZ])
    randX = np.random.normal(-0.3, sigma)
    randY = np.random.normal(-0.2, sigma)
    randZ = np.random.normal(-0.5, sigma)
    RH_foot = np.array([randX, randY, randZ])
    contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
#    print contacts
#    contacts = contactsToStack[0:nc, :]

    comWF = np.array([0.0,0.0,0.0])
    
    ''' compute iterative projection '''
    stanceLegs = [1 ,1, 1, 1]
    randomSwingLeg = random.randint(0,3)
    #    print 'Swing leg', randomSwingLeg
    stanceLegs[randomSwingLeg] = 0
    
    params.setContactsPosWF(contacts)
    params.setCoMPosWF(comWF)
    params.setActiveContacts(stanceLegs)
    params.setConstraintModes(constraint_mode_IP)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setNumberOfFrictionConesEdges(4)
    params.setTotalMass(total_mass)
    #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
    comp_time = comp_time * 1000.0
    
    print comp_time
    tests3contacts[iter] = comp_time

nc = 4
comWF = np.array([0.0, 0.0, 0.0])

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
    randX = np.random.normal(0.3, sigma)
    randY = np.random.normal(0.2, sigma)
    randZ = np.random.normal(-0.5, sigma)
    LF_foot = np.array([randX, randY, randZ])
    randX = np.random.normal(0.3, sigma)
    randY = np.random.normal(-0.2, sigma)
    randZ = np.random.normal(-0.5, sigma)
    RF_foot = np.array([randX, randY, randZ])
    randX = np.random.normal(-0.3, sigma)
    randY = np.random.normal(0.2, sigma)
    randZ = np.random.normal(-0.5, sigma)
    LH_foot = np.array([randX, randY, randZ])
    randX = np.random.normal(-0.3, sigma)
    randY = np.random.normal(-0.2, sigma)
    randZ = np.random.normal(-0.5, sigma)
    RH_foot = np.array([randX, randY, randZ])
    contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
#    print contacts
    comWF = np.array([0.0,0.0,0.0])
        
    stanceLegs = [1 ,1, 1, 1]

    ''' compute iterative projection '''
    params.setContactsPosWF(contacts)
    params.setCoMPosWF(comWF)
    params.setActiveContacts(stanceLegs)
    params.setConstraintModes(constraint_mode_IP)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setNumberOfFrictionConesEdges(4)
    params.setTotalMass(total_mass)
    #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
#    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(constraint_mode_IP, stanceLegs, contacts, normals, trunk_mass, ng, mu, comWF, torque_limits)
    comp_time = comp_time * 1000.0
    tests4contacts[iter] = comp_time
    
''' plotting Iterative Projection points '''
plotter = Plotter()

''' 2D figure '''
    
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()

plt.plot([1,2,3])
plt.subplot(121)
#print tests
plt.grid()
plt.hist(tests3contacts,bins=np.arange(0,25,0.1))
plt.title("Histogram on 50 tests of the IP with 3 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")

plt.subplot(122)
#print tests
plt.grid()
plt.hist(tests4contacts,bins=np.arange(0,25,0.1))
plt.title("Histogram on 50 tests of the IP with 4 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")

plt.show()
mpl.rcParams.update({'font.size': 15})
#fig.savefig('../figs/initial_tests_IP/histogramIP.pdf')