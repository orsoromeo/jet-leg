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
import matplotlib.colors as colors
import matplotlib.cm as cmx

from jet_leg.plotting.plotting_tools import Plotter
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.map.height_map import HeightMap

from jet_leg.optimization.path_sequential_iterative_projection import PathIterativeProjection
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
        
''' MAIN '''
start_t_IPVC = time.time()

math = Math()
compDyn = ComputationalDynamics()
pathIP = PathIterativeProjection()
# number of contacts
number_of_contacts = 4

# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode = ['FRICTION_AND_ACTUATION',
                   'FRICTION_AND_ACTUATION',
                   'FRICTION_AND_ACTUATION',
                   'FRICTION_AND_ACTUATION']
useVariableJacobian = True
total_mass = 100
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

axisZ= array([[0.0], [0.0], [1.0]])
math = Math()
n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
# %% Cell 2
normals = np.vstack([n1, n2, n3, n4])
ng = 4
tolerance = 0.01
max_iter = 10

params = IterativeProjectionParameters()
params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceLegs)
params.setConstraintModes(constraint_mode)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(total_mass)
CoMlist, stackedErrors, stacked_polygons = pathIP.find_vertex_along_path(params, desired_direction, tolerance, max_iter)
newLimitPoint = CoMlist[-1]
print 'Errors convergence: ', stackedErrors
print 'first', stacked_polygons[0]
first = stacked_polygons.pop(0)
print 'first', first
first = stacked_polygons.pop(0)
print 'second', first
print("Path Sequential Iterative Projection: --- %s seconds ---" % (time.time() - start_t_IPVC))


'''Plotting Fig 1'''
plt.close('all')
plotter = Plotter()

print stackedErrors
#dx = stackedErrors[:,0]
#dy = stackedErrors[:,1]
#d = np.linalg.norm(stackedErrors, axis=1)

plt.figure()
plt.grid()
plt.ylabel("Absolute error [m]")
plt.xlabel("Iterations number")
#plt.plot(np.abs(stackedErrors[:,0]), 'b', linewidth=5, label = 'X direction')
#plt.plot(np.abs(stackedErrors[:,1]), 'r', linewidth=2, label = 'Y direction')
plt.plot(np.abs(d), 'g', linewidth=2, label = 'd')
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

''' instantaneous plot actuation regions'''
lower_lim = -10
upper_lim = 10
scale = np.linspace(lower_lim, upper_lim, 10)
jet = cm = plt.get_cmap('seismic') 
cNorm  = colors.Normalize(vmin=lower_lim, vmax=upper_lim)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
index = 0
err = np.hstack(stackedErrors)
  
 
for polygon in stacked_polygons:
    point = np.vstack([polygon])
    x = np.hstack([point[:,0], point[0,0]])
    y = np.hstack([point[:,1], point[0,1]])
    colorVal = scalarMap.to_rgba(scale[index])
#    print index
    plt.plot(x,y, color = colorVal,  linewidth=5., label = 'error: '+str(round(d[index], 2)))
    plt.plot(CoMlist[index,0], CoMlist[index,1], color = colorVal, marker='o', markersize=15)
    index+=1
    
    
plt.xlim(-0.9, 0.5)
plt.ylim(-0.7, 0.7)
plt.legend()
plt.show()

