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
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters

import matplotlib as mpl
import matplotlib.pyplot as plt
import random
        
plt.close('all')
math = Math()

# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode_IP = ['ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION']

useVariableJacobian = False
# number of decision variables of the problem
n = nc*6

''' parameters to be tuned'''
g = 9.81
trunk_mass = 85.
mu = 0.8
    
axisZ= array([[0.0], [0.0], [1.0]])

comp_dyn = ComputationalDynamics()

number_of_tests = 100
onlyFrictionTests3contacts = np.zeros((number_of_tests))
onlyFrictionTests4contacts = np.zeros((number_of_tests))  
onlyActuationTests3contacts = np.zeros((number_of_tests))
onlyActuationTests4contacts = np.zeros((number_of_tests))  
frictionAndActuation3contacts = np.zeros((number_of_tests))
frictionAndActuation4contacts = np.zeros((number_of_tests))  

params = IterativeProjectionParameters()


'''ONLY FRICTION'''
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

    LF_tau_lim = [80.0, 100.0, 100.0]
    RF_tau_lim = [80.0, 100.0, 100.0]
    LH_tau_lim = [80.0, 100.0, 100.0]
    RH_tau_lim = [80.0, 100.0, 100.0]
    torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
    comWF = np.array([0.0,0.0,0.0])
    
    ''' compute iterative projection '''
    stanceLegs = [1 ,1, 1, 1]
    randomSwingLeg = random.randint(0,3)
    #    print 'Swing leg', randomSwingLeg
    stanceLegs[randomSwingLeg] = 0
    
    params.setContactsPos(contacts)
    params.setCoMPos(comWF)
    params.setTorqueLims(torque_limits)
    params.setActiveContacts(stanceLegs)
    params.setConstraintModes(constraint_mode_IP)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setNumberOfFrictionConesEdges(ng)
    params.setTrunkMass(trunk_mass)
    #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
    comp_time = comp_time * 1000.0
    
    print comp_time
    onlyFrictionTests3contacts[iter] = comp_time

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

    LF_tau_lim = [80.0, 100.0, 100.0]
    RF_tau_lim = [80.0, 100.0, 100.0]
    LH_tau_lim = [80.0, 100.0, 100.0]
    RH_tau_lim = [80.0, 100.0, 100.0]
    torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
    comWF = np.array([0.0,0.0,0.0])
        
    stanceLegs = [1 ,1, 1, 1]

    ''' compute iterative projection '''
    params.setContactsPos(contacts)
    params.setCoMPos(comWF)
    params.setTorqueLims(torque_limits)
    params.setActiveContacts(stanceLegs)
    params.setConstraintModes(constraint_mode_IP)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setNumberOfFrictionConesEdges(ng)
    params.setTrunkMass(trunk_mass)
    #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
#    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(constraint_mode_IP, stanceLegs, contacts, normals, trunk_mass, ng, mu, comWF, torque_limits)
    comp_time = comp_time * 1000.0
    onlyFrictionTests4contacts[iter] = comp_time

'''ONLY ACTUATION'''

constraint_mode_IP = ['ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION']
params.setConstraintModes(constraint_mode_IP)

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

    LF_tau_lim = [80.0, 100.0, 100.0]
    RF_tau_lim = [80.0, 100.0, 100.0]
    LH_tau_lim = [80.0, 100.0, 100.0]
    RH_tau_lim = [80.0, 100.0, 100.0]
    torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
    comWF = np.array([0.0,0.0,0.0])
    
    ''' compute iterative projection '''
    stanceLegs = [1 ,1, 1, 1]
    randomSwingLeg = random.randint(0,3)
    #    print 'Swing leg', randomSwingLeg
    stanceLegs[randomSwingLeg] = 0
    
    params.setContactsPos(contacts)
    params.setCoMPos(comWF)
    params.setTorqueLims(torque_limits)
    params.setActiveContacts(stanceLegs)
    params.setConstraintModes(constraint_mode_IP)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setNumberOfFrictionConesEdges(ng)
    params.setTrunkMass(trunk_mass)
    #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
    comp_time = comp_time * 1000.0
    
    print comp_time
    onlyActuationTests3contacts[iter] = comp_time

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

    LF_tau_lim = [80.0, 100.0, 100.0]
    RF_tau_lim = [80.0, 100.0, 100.0]
    LH_tau_lim = [80.0, 100.0, 100.0]
    RH_tau_lim = [80.0, 100.0, 100.0]
    torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
    comWF = np.array([0.0,0.0,0.0])
        
    stanceLegs = [1 ,1, 1, 1]

    ''' compute iterative projection '''
    params.setContactsPos(contacts)
    params.setCoMPos(comWF)
    params.setTorqueLims(torque_limits)
    params.setActiveContacts(stanceLegs)
    params.setConstraintModes(constraint_mode_IP)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setNumberOfFrictionConesEdges(ng)
    params.setTrunkMass(trunk_mass)
    #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
#    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(constraint_mode_IP, stanceLegs, contacts, normals, trunk_mass, ng, mu, comWF, torque_limits)
    comp_time = comp_time * 1000.0
    onlyActuationTests4contacts[iter] = comp_time
    
    
'''FRICTION_AND_ACTUATION'''

constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION']
                      
params.setConstraintModes(constraint_mode_IP)

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

    LF_tau_lim = [80.0, 100.0, 100.0]
    RF_tau_lim = [80.0, 100.0, 100.0]
    LH_tau_lim = [80.0, 100.0, 100.0]
    RH_tau_lim = [80.0, 100.0, 100.0]
    torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
    comWF = np.array([0.0,0.0,0.0])
    
    ''' compute iterative projection '''
    stanceLegs = [1 ,1, 1, 1]
    randomSwingLeg = random.randint(0,3)
    #    print 'Swing leg', randomSwingLeg
    stanceLegs[randomSwingLeg] = 0
    
    params.setContactsPos(contacts)
    params.setCoMPos(comWF)
    params.setTorqueLims(torque_limits)
    params.setActiveContacts(stanceLegs)
    params.setConstraintModes(constraint_mode_IP)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setNumberOfFrictionConesEdges(ng)
    params.setTrunkMass(trunk_mass)
    #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
    comp_time = comp_time * 1000.0
    
    print comp_time
    frictionAndActuation3contacts[iter] = comp_time

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
    LF_tau_lim = [100.0, 100.0, 100.0]
    RF_tau_lim = [100.0, 100.0, 100.0]
    LH_tau_lim = [100.0, 100.0, 100.0]
    RH_tau_lim = [100.0, 100.0, 100.0]
    torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim, ])

    comWF = np.array([0.0,0.0,0.0])
        
    stanceLegs = [1 ,1, 1, 1]

    ''' compute iterative projection '''
    params.setContactsPos(contacts)
    params.setCoMPos(comWF)
    params.setTorqueLims(torque_limits)
    params.setActiveContacts(stanceLegs)
    params.setConstraintModes(constraint_mode_IP)
    params.setContactNormals(normals)
    params.setFrictionCoefficient(mu)
    params.setNumberOfFrictionConesEdges(ng)
    params.setTrunkMass(trunk_mass)
    #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
#    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(constraint_mode_IP, stanceLegs, contacts, normals, trunk_mass, ng, mu, comWF, torque_limits)
    comp_time = comp_time * 1000.0
    frictionAndActuation4contacts[iter] = comp_time
    
''' plotting Iterative Projection points '''
plotter = Plotter()

''' 2D figure '''
fig = plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()

plt.plot([1,2,3])
fig.suptitle('Computation times for ' + str(number_of_tests) + ' tests', fontsize=18)
plt.subplot(321)
#print tests
plt.grid()
plt.hist(onlyFrictionTests3contacts, color = "salmon", bins=np.arange(0,25,0.25))
plt.title("3 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")

subpl2 = plt.subplot(322)
#print tests
plt.grid()
plt.hist(onlyFrictionTests4contacts, color = "salmon", bins=np.arange(0,25,0.25))
plt.title("4 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")
subpl2.yaxis.set_label_position("right")

plt.subplot(323)
#print tests
plt.grid()
plt.hist(onlyActuationTests3contacts, color = "limegreen", bins=np.arange(0,25,0.25))
#plt.title("only friction, 3 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")

subpl4 = plt.subplot(324)
#print tests
plt.grid()
plt.hist(onlyActuationTests4contacts, color = "limegreen", bins=np.arange(0,25,0.25))
#plt.title("only friction, 4 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")
subpl4.yaxis.set_label_position("right")

plt.subplot(325)
#print tests
plt.grid()
plt.hist(frictionAndActuation3contacts, color = "skyblue", bins=np.arange(0,25,0.25))
#plt.title("only friction, 3 point contacts")
plt.xlabel("time [ms]")
plt.ylabel("count")

subpl6 = plt.subplot(326)
#print tests
plt.grid()
plt.hist(frictionAndActuation4contacts, color = "skyblue", bins=np.arange(0,25,0.25))
#plt.title("only friction, 4 point contacts")
plt.xlabel("time [ms]")
plt.ylabel("count")
subpl6.yaxis.set_label_position("right")
plt.show()
mpl.rcParams.update({'font.size': 14})
plt.savefig('../../figs/IP_bretl/statsIP.pdf')