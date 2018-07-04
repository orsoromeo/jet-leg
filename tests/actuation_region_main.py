# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import time
import pylab
import pypoman
import numpy as np

from context import legsthrust 

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix
from numpy.linalg import norm
from legsthrust.plotting_tools import Plotter
from legsthrust.constraints import Constraints
from legsthrust.kinematics import Kinematics
from legsthrust.math_tools import Math
from legsthrust.computational_dynamics import ComputationalDynamics
from legsthrust.vertex_based_projection import VertexBasedProjection

import matplotlib.pyplot as plt
from legsthrust.arrow3D import Arrow3D

plt.close('all')
math = Math()
# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

constraint_mode = 'ONLY_ACTUATION'
# number of decision variables of the problem
n = nc*6

# contact positions
""" contact points """
#LF_foot = np.array([0.3, 0.2, -.9])
#RF_foot = np.array([0.3, -0.2, -0.5])
#LH_foot = np.array([-0.3, 0.2, -0.5])
#RH_foot = np.array([-0.3, -0.2, -0.5])

LF_foot = np.array([0.3, 0.2, -0.35])
RF_foot = np.array([0.3, -0.2, -0.58])
LH_foot = np.array([-0.2, 0.2, -0.58])
RH_foot = np.array([-0.3, -0.2, -0.58])
contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts = contactsToStack[0:nc, :]


L_LF_foot = np.array([-0.2, 0.2, -0.5])
L_RF_foot = np.array([-0.2, 0.1, -0.5])
L_LH_foot = np.array([-0.1, 0.1, -0.5])
L_RH_foot = np.array([-0.1, 0.2, -0.5])

R_LF_foot = np.array([0.2, 0.2, -0.5])
R_RF_foot = np.array([0.3, 0.1, -0.5])
R_LH_foot = np.array([0.1, 0.1, -0.5])
R_RH_foot = np.array([0.1, 0.2, -0.5])

#contacts = np.vstack((L_LF_foot,L_RF_foot,L_LH_foot,L_RH_foot))

''' parameters to be tuned'''
g = 9.81
mass = 90.
mu = 0.8

axisZ= array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
# %% Cell 2

normals = np.vstack([n1, n2, n3, n4])

#
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.set_xlim(-0.6, 0.6)
#ax.set_ylim(-0.6, 0.6)
#ax.set_zlim(-0.2, 0.5)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)
for j in range(0,nc):
    ax.scatter(contacts[j,0], contacts[j,1], contacts[j,2],c='b',s=100)
    a = Arrow3D([contacts[j,0], contacts[j,0]+normals[j,0]/10], [contacts[j,1], contacts[j,1]+normals[j,1]/10],[contacts[j,2], contacts[j,2]+normals[j,2]/10], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
    ax.add_artist(a)

comp_dyn = ComputationalDynamics()
IP_points, actuation_LF, actuation_RF, actuation_LH, actuation_RH = comp_dyn.iterative_projection_bretl(constraint_mode, contacts, normals, mass, ng, mu)

''' plotting Iterative Projection points '''

plotter = Plotter()
scaling_factor = 2000
plotter.plot_polygon(np.transpose(IP_points))
plotter.plot_actuation_polygon(ax, actuation_LF, LF_foot, scaling_factor)
plotter.plot_actuation_polygon(ax, actuation_RF, RF_foot, scaling_factor)
plotter.plot_actuation_polygon(ax, actuation_LH, LH_foot, scaling_factor)
plotter.plot_actuation_polygon(ax, actuation_RH, RH_foot, scaling_factor)
feasible, unfeasible, contact_forces = comp_dyn.LP_projection(constraint_mode, contacts, normals, mass, mu, ng, nc, mu)
#print contact_forces
#for i in range(0, np.size(contact_forces,0)):
#    for j in range(0,nc):
#        a = Arrow3D([contacts[j,0], contacts[j,0]+contact_forces[i,j*3]/200], [contacts[j,1], contacts[j,1]+contact_forces[i,j*3+1]/200],[contacts[j,2], contacts[j,2]+contact_forces[i,j*3+2]/200], mutation_scale=20, lw=3, arrowstyle="-|>", color="g")
#        ax.add_artist(a)

a1 = Arrow3D([0.0, 0.0],[ 0.0,0.0],[ 0.0, -mass*g/scaling_factor], mutation_scale=20, lw=3, arrowstyle="-|>", color="b")
ax.add_artist(a1)

''' plotting LP test points '''
if np.size(feasible,0) != 0:
    ax.scatter(feasible[:,0], feasible[:,1], feasible[:,2],c='g',s=50)
if np.size(unfeasible,0) != 0:
    ax.scatter(unfeasible[:,0], unfeasible[:,1], unfeasible[:,2],c='r',s=50)

''' Vertex-based projection '''
vertexBasedProj = VertexBasedProjection()
vertices2d, simplices = vertexBasedProj.project(constraint_mode, contacts, normals, mass, ng, mu)

for simplex in simplices:
    plt.plot(vertices2d[simplex, 0], vertices2d[simplex, 1], 'y-', linewidth=5.)

plt.show()

''' Add 2D figure '''
plt.figure()
h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')
if np.size(feasible,0) != 0:
    h2 = plt.scatter(feasible[:,0], feasible[:,1],c='g',s=50, label='LP feasible')
if np.size(unfeasible,0) != 0:
    h3 = plt.scatter(unfeasible[:,0], unfeasible[:,1],c='r',s=50, label='LP unfeasible')
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