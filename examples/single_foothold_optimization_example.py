# -*- coding: utf-8 -*-
"""
Created on Tue Dec 11 19:42:07 2018

@author: rorsolino
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Oct 18 11:05:06 2018

@author: rorsolino
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import time
import pylab
import pypoman
import numpy as np
from copy import deepcopy

from context import jet_leg 
import matplotlib.colors as colors
import matplotlib.cm as cmx

from numpy import array
from numpy.linalg import norm
from jet_leg.plotting_tools import Plotter
import random
from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.foothold_planning import FootHoldPlanning

import matplotlib.pyplot as plt
from jet_leg.arrow3D import Arrow3D
        
plt.close('all')
math = Math()
# number of contacts
#nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION

constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION']
useVariableJacobian = False
# number of decision variables of the problem
#n = nc*6
comWF = np.array([0.0, 0.0, 0.0])
# contact positions
""" contact points """

LF_foot = np.array([0.3, 0.2, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.3, 0.2, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.5])

feetPos = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts = deepcopy(feetPos)

#contacts = contactsToStack[0:nc+1, :]
#print contacts

''' parameters to be tuned'''
g = 9.81
trunk_mass = 85.
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

comWF = np.array([0.0,0.0,0.0])

nc = np.sum(stanceFeet)

comp_dyn = ComputationalDynamics()
params = IterativeProjectionParameters()
params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(trunk_mass)
params.com_position_to_validateW = [0.1, 0.1, 0]

footOption0 = [0., 0., 0.] + LF_foot
footOption1 = [0., 0.1, 0.] + LF_foot
footOption2 = [0.1, 0., 0.] + LF_foot
footOption3 = [0.1, 0.1, 0.] + LF_foot
footOption4 = [-0.1, 0.1, 0.] + LF_foot
footOption5 = [0.1, -0.1, 0.] + LF_foot
footOption6 = [-0.1, -0.1, 0.] + LF_foot
footOption7 = [0., -0.1, 0.] + LF_foot
footOption8 = [-0.1, 0., 0.] + LF_foot
params.footOptions = np.array([footOption0,
                                       footOption1,
                                       footOption2,
                                       footOption3,
                                       footOption4,
                                       footOption5,
                                       footOption6,
                                       footOption7,
                                       footOption8])

''' compute iterative projection '''
print 'contacts', feetPos
footHoldPlanning = FootHoldPlanning()
chosen_foothold, actuationRegions = footHoldPlanning.optimizeFootHold(params)
#IP_points, actuation_polygons, computation_time = comp_dyn.iterative_projection_bretl(params)
print 'foot option ',chosen_foothold

lowel_lim = -10
upper_lim = 10
numberOfOptions = np.size(params.footOptions,0)
scale = np.linspace(lowel_lim, upper_lim, numberOfOptions)
jet = cm = plt.get_cmap('seismic') 
cNorm  = colors.Normalize(vmin=-30, vmax=35)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
idx = 0
print 'contacts', feetPos
h0 = plt.plot(params.footOptions[chosen_foothold,0],params.footOptions[chosen_foothold,1],'^', color = 'r', markersize=20)
h1 = plt.plot(feetPos[0:nc,0],feetPos[0:nc,1],'ks',markersize=15, label='feet')

for polygon in actuationRegions:
    colorVal = scalarMap.to_rgba(scale[idx])
    x = polygon[:,0]
    y = polygon[:,1]
    h = plt.plot(x,y, color = colorVal, linewidth=5.)
    h2 = plt.plot(params.footOptions[idx,0],params.footOptions[idx,1],'o', color = colorVal, markersize=15)
    idx += 1

    
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.show()# -*- coding: utf-8 -*-
