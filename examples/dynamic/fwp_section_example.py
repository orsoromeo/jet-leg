# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""


import numpy as np

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix

from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.dynamics.vertex_based_projection import VertexBasedProjection
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.plotting.plotting_tools import Plotter
import random
import matplotlib.pyplot as plt
        
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
constraint_mode_IP = ['ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION']

comWF = np.array([0.0, 0.0, 0.0])

""" contact points in the World Frame"""
LF_foot = np.array([0.3, 0.2, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.3, 0.2, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.5])

contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

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

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForceW = np.array([0.0, 0.0, 0.0]) # units are Nm

robot_name = 'hyq'
comp_dyn = ComputationalDynamics(robot_name)

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters(robot_name)
params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
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

''' Vertex-based projection '''
vertexBasedProj = VertexBasedProjection()
vertices2d, simplices = vertexBasedProj.project(params, robot_name)

for simplex in simplices:
    plt.plot(vertices2d[simplex, 0], vertices2d[simplex, 1], 'y-', linewidth=5.)

plotter = Plotter()
h2 = plotter.plot_polygon(np.transpose(IP_points), '--b', 5, 'Feasible Region')
h1 = plt.plot(0.0, 0.0, 'y-', linewidth=5., label = 'FWP section')

plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
#plt.legend((h1,h2,h3,h4,h5),('feet','Iterative Projection', 'LP feasible', 'LP unfeasible','Vertex-based projection'))
plt.show()