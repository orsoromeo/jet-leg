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

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix
from numpy.linalg import norm
from jet_leg.plotting_tools import Plotter

from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.vertex_based_projection import VertexBasedProjection
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters
import random
import matplotlib.pyplot as plt
from jet_leg.arrow3D import Arrow3D
        
plt.close('all')
math = Math()

''' number of generators, i.e. rays/edges used to linearize the friction cone '''
ng = 4

'''
possible constraints for each foot:
 ONLY_ACTUATION = only joint-torque limits are enforces
 ONLY_FRICTION = only friction cone constraints are enforced
 FRICTION_AND_ACTUATION = both friction cone constraints and joint-torque limits
'''
constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION']

# number of decision variables of the problem
#n = nc*6
comWF = np.array([0.0, 0.0, 0.0])

""" contact points in the World Frame"""
LF_foot = np.array([0.3, 0.2, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.3, 0.2, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.5])

contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

#contacts = contactsToStack[0:nc+1, :]
#print contacts

''' parameters to be tuned'''
g = 9.81
trunk_mass = 85.
mu = 0.5

''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1,1,1,1]

randomSwingLeg = random.randint(0,3)
tripleStance = True # if you want you can define a swing leg using this variable
if tripleStance:
    print 'Swing leg', randomSwingLeg
    stanceFeet[randomSwingLeg] = 0
print 'stanceLegs ' ,stanceFeet

''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
axisZ= array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # LF
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # RF
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # LH
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # RH
normals = np.vstack([n1, n2, n3, n4])

''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
HAA = Hip Abduction Adduction
HFE = Hip Flextion Extension
KFE = Knee Flextion Extension
'''
LF_tau_lim = [50.0, 100.0, 100.0] # HAA, HFE, KFE
RF_tau_lim = [50.0, 100.0, 100.0] # HAA, HFE, KFE
LH_tau_lim = [50.0, 100.0, 100.0] # HAA, HFE, KFE
RH_tau_lim = [50.0, 100.0, 100.0] # HAA, HFE, KFE
torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForceW = np.array([0.0, 0.0, 0.0]) # units are Nm

comp_dyn = ComputationalDynamics()

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
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
params.externalForceWF = extForceW  # params.externalForceWF is actually used anywhere at the moment

''' compute iterative projection 
Outputs of "iterative_projection_bretl" are:
IP_points = resulting 2D vertices
actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
computation_time = how long it took to compute the iterative projection
'''
IP_points, force_polytopes, IP_computation_time = comp_dyn.iterative_projection_bretl(params)

''' plotting Iterative Projection points '''

feasible, unfeasible, contact_forces = comp_dyn.LP_projection(params)
#print contact_forces
#for i in range(0, np.size(contact_forces,0)):
#    for j in range(0,nc):
#        a = Arrow3D([contacts[j,0], contacts[j,0]+contact_forces[i,j*3]/200], [contacts[j,1], contacts[j,1]+contact_forces[i,j*3+1]/200],[contacts[j,2], contacts[j,2]+contact_forces[i,j*3+2]/200], mutation_scale=20, lw=3, arrowstyle="-|>", color="g")
#        ax.add_artist(a)

#a1 = Arrow3D([0.0, 0.0],[ 0.0,0.0],[ 0.0, -trunk_mass*g/scaling_factor], mutation_scale=20, lw=3, arrowstyle="-|>", color="b")
#ax.add_artist(a1)

''' plotting LP test points '''
#if np.size(feasible,0) != 0:
#    ax.scatter(feasible[:,0], feasible[:,1], feasible[:,2],c='g',s=50)
#if np.size(unfeasible,0) != 0:
#    ax.scatter(unfeasible[:,0], unfeasible[:,1], unfeasible[:,2],c='r',s=50)

''' Vertex-based projection '''
vertexBasedProj = VertexBasedProjection()
vertices2d, simplices = vertexBasedProj.project(params)

#for simplex in simplices:
#    plt.plot(vertices2d[simplex, 0], vertices2d[simplex, 1], 'y-', linewidth=5.)

plt.show()

''' Add 2D figure '''
#plt.figure()
#h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')
#if np.size(feasible,0) != 0:
#        h2 = plt.scatter(feasible[:,0], feasible[:,1],c='g',s=50, label='LP feasible')
#if np.size(unfeasible,0) != 0:
#        h3 = plt.scatter(unfeasible[:,0], unfeasible[:,1],c='r',s=50, label='LP unfeasible')

feasiblePointsSize = np.size(feasible,0)
for j in range(0, feasiblePointsSize):
    if (feasible[j,2]<0.01)&(feasible[j,2]>-0.01):
        plt.scatter(feasible[j,0], feasible[j,1],c='g',s=50)
        lastFeasibleIndex = j
unfeasiblePointsSize = np.size(unfeasible,0)

for j in range(0, unfeasiblePointsSize):
    if (unfeasible[j,2]<0.01)&(unfeasible[j,2]>-0.01):
        plt.scatter(unfeasible[j,0], unfeasible[j,1],c='r',s=50)
        lastUnfeasibleIndex = j
h2 = plt.scatter(feasible[lastFeasibleIndex,0], feasible[lastFeasibleIndex,1],c='g',s=50, label='LP feasible')
h3 = plt.scatter(unfeasible[lastUnfeasibleIndex,0], unfeasible[lastUnfeasibleIndex,1],c='r',s=50, label='LP unfeasible')

        
h4 = plotter.plot_polygon(np.transpose(IP_points), '--b','Iterative Projection')

#i = 0
#for simplex in simplices:
#    if (i==0):
#        h5 = plt.plot(vertices2d[simplex, 0], vertices2d[simplex, 1], 'y-', linewidth=5., label = 'Vertex-based projection')
#    else:
#        plt.plot(vertices2d[simplex, 0], vertices2d[simplex, 1], 'y-', linewidth=5.)
#    i+=1
    
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
#plt.legend((h1,h2,h3,h4,h5),('feet','Iterative Projection', 'LP feasible', 'LP unfeasible','Vertex-based projection'))
plt.show()