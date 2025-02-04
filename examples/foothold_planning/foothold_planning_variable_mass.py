# -*- coding: utf-8 -*-
"""
Created on Thu Dec 20 09:57:43 2018

@author: rorsolino
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
from jet_leg.foothold_planning_interface import FootholdPlanningInterface
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
# contact positions
""" contact points """

LF_foot = np.array([0.3, 0.2, -0.0])
RF_foot = np.array([0.3, -0.2, -0.0])
LH_foot = np.array([-0.3, 0.2, -0.0])
RH_foot = np.array([-0.3, -0.2, -0.0])

feetPos = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts =deepcopy(feetPos)

#contacts = contactsToStack[0:nc+1, :]
#print contacts

''' parameters to be tuned'''
g = 9.81
trunk_mass = 65.
mu = 0.9

stanceFeet = [1,1,1,0]
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



nc = np.sum(stanceFeet)

comp_dyn = ComputationalDynamics()
params = IterativeProjectionParameters()
params.setContactsPosWF(contacts)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(trunk_mass)

#inputs for foothold planning
foothold_params = FootholdPlanningInterface()
foothold_params.com_position_to_validateW = [0.05, 0.05, 0.55]


maxCorrection = 0.2
#square
#predictedLF_foot = np.add(LF_foot,np.array([0.1,0.3,0.0]))
#footOption0 = [0., 0., 0.] + predictedLF_foot
#footOption1 = [0.,res, 0.] + predictedLF_foot
#footOption2 = [res, 0., 0.] + predictedLF_foot
#footOption3 = [res, res, 0.] + predictedLF_foot
#footOption4 = [-res, res, 0.] + predictedLF_foot
#footOption5 = [res, -res, 0.] + predictedLF_foot
#footOption6 = [-res, -res, 0.] + predictedLF_foot
#footOption7 = [0., -res, 0.] + predictedLF_foot
#footOption8 = [-res, 0., 0.] + predictedLF_foot

#y
#predictedLF_foot = np.add(LF_foot,np.array([0.1,0.3,0.0]))
#footOption4 = [0., 0., 0.] + predictedLF_foot
#footOption3 = [0, -bound*1/4, 0.] + predictedLF_foot
#footOption2 = [0, -bound*2/4, 0.] + predictedLF_foot
#footOption1 = [0., -bound*3/4, 0.] + predictedLF_foot
#footOption0 = [0, -bound*4/4., 0.] + predictedLF_foot
#footOption5 = [0.,bound*1/4, 0.] + predictedLF_foot
#footOption6 = [0, bound*2/4, 0.] + predictedLF_foot
#footOption7 = [0, bound*3/4, 0.] + predictedLF_foot
#footOption8 = [0, bound*4/4,  0.] + predictedLF_foot

#x
predictedLF_foot = np.add(LF_foot,np.array([0.3,0.15,0.0]))
footOption4 = [0., 0., 0.] + predictedLF_foot
footOption3 = [ -maxCorrection*1/4, 0., 0.] + predictedLF_foot
footOption2 = [ -maxCorrection*2/4, 0., 0.] + predictedLF_foot
footOption1 = [ -maxCorrection*3/4, 0., 0.] + predictedLF_foot
footOption0 = [ -maxCorrection*4/4., 0., 0.] + predictedLF_foot
footOption5 = [ maxCorrection*1/4, 0., 0.] + predictedLF_foot
footOption6 = [ maxCorrection*2/4,  0.,0.] + predictedLF_foot
footOption7 = [ maxCorrection*3/4,  0.,0.] + predictedLF_foot
footOption8 = [ maxCorrection*4/4,  0., 0.] + predictedLF_foot

foothold_params.minRadius = 0.2


foothold_params.footOptions = np.array([footOption0,
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
#chosen_foothold, actuationRegions = footHoldPlanning.selectMaximumFeasibleArea(footPlanningParams, params)

foothold_params.option_index, stackedResidualRadius, actuationRegions, mapFootHoldIdxToPolygonIdx = footHoldPlanning.selectMinumumRequiredFeasibleAreaResidualRadius( foothold_params, params)
print 'residual radius ', stackedResidualRadius
print 'feet options', foothold_params.footOptions
print 'final index', foothold_params.option_index
print 'list of indices', mapFootHoldIdxToPolygonIdx
#IP_points, actuation_polygons, computation_time = comp_dyn.iterative_projection_bretl(params)
#print 'foot option ',chosen_foothold

trunk_mass +=15
params.setTotalMass(trunk_mass)
foothold_params.option_index, stackedResidualRadius, actuationRegions, mapFootHoldIdxToPolygonIdx = footHoldPlanning.selectMinumumRequiredFeasibleAreaResidualRadius( foothold_params, params)
print 'residual radius ', stackedResidualRadius
#print 'feet options', foothold_params.footOptions
print 'final index', foothold_params.option_index
print 'list of indices', mapFootHoldIdxToPolygonIdx
#IP_points, actuation_polygons, computation_time = comp_dyn.iterative_projection_bretl(params)
#print 'foot option ',chosen_foothold

trunk_mass +=15
params.setTotalMass(trunk_mass)
foothold_params.option_index, stackedResidualRadius, actuationRegions, mapFootHoldIdxToPolygonIdx = footHoldPlanning.selectMinumumRequiredFeasibleAreaResidualRadius( foothold_params, params)
print 'residual radius ', stackedResidualRadius
#print 'feet options', foothold_params.footOptions
print 'final index', foothold_params.option_index
print 'list of indices', mapFootHoldIdxToPolygonIdx
#IP_points, actuation_polygons, computation_time = comp_dyn.iterative_projection_bretl(params)
#print 'foot option ',chosen_foothold


trunk_mass +=15
params.setTotalMass(trunk_mass)
foothold_params.option_index, stackedResidualRadius, actuationRegions, mapFootHoldIdxToPolygonIdx = footHoldPlanning.selectMinumumRequiredFeasibleAreaResidualRadius( foothold_params, params)
print 'residual radius ', stackedResidualRadius
#print 'feet options', foothold_params.footOptions
print 'final index', foothold_params.option_index
print 'list of indices', mapFootHoldIdxToPolygonIdx

lowel_lim = -10
upper_lim = 10

scale = np.linspace(lowel_lim, upper_lim, np.size(actuationRegions))
jet = cm = plt.get_cmap('seismic') 
cNorm  = colors.Normalize(vmin=-30, vmax=35)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
idx = 0
#print 'contacts', feetPos
h0 = plt.plot(foothold_params.footOptions[foothold_params.option_index][0],foothold_params.footOptions[foothold_params.option_index][1],'^', color = 'r', markersize=20)
h1 = plt.plot(feetPos[0:nc,0],feetPos[0:nc,1],'ks',markersize=15, label='feet')

for polygon in actuationRegions:
    colorVal = scalarMap.to_rgba(scale[idx])
    x = polygon[:,0]
    y = polygon[:,1]
    h = plt.plot(x,y, color = colorVal, linewidth=5.)
#    print foothold_params.footOptions[idx]
    h2 = plt.plot(foothold_params.footOptions[mapFootHoldIdxToPolygonIdx[idx]][0],foothold_params.footOptions[mapFootHoldIdxToPolygonIdx[idx]][1],'o', color = colorVal, markersize=15)
    idx += 1
h3 = plt.plot(params.getCoMPosWF()[0],params.getCoMPosWF()[1],'or',markersize=25)

plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.axis('equal')
plt.legend()
plt.show()# -*- coding: utf-8 -*-
