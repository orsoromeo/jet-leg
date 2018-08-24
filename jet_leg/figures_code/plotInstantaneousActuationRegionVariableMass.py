# -*- coding: utf-8 -*-
"""
Created on Fri Aug 10 16:08:43 2018

@author: romeoorsolino
"""

import numpy as np

from context import jet_leg 

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix
from numpy.linalg import norm

from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics

import matplotlib as mpl
import matplotlib.colors as colors
import matplotlib.cm as cmx
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge, Polygon
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from matplotlib.collections import PatchCollection

from jet_leg.arrow3D import Arrow3D

def set_axes_radius(ax, origin, radius):
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    set_axes_radius(ax, origin, radius)
    
plt.close('all')
math = Math()
# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION or ONLY_FRICTION
constraint_mode = 'ONLY_ACTUATION'
constraint_mode_IP = 'ONLY_ACTUATION'
useVariableJacobian = False
# number of decision variables of the problem
n = nc*6

# contact positions
""" contact points """
#LF_foot = np.array([0.3, 0.2, -.9])
#RF_foot = np.array([0.3, -0.2, -0.5])
#LH_foot = np.array([-0.3, 0.2, -0.5])
#RH_foot = np.array([-0.3, -0.2, -0.5])

LF_foot = np.array([0.3, 0.3, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.3, 0.3, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.3])

contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts = contactsToStack[0:nc, :]


''' parameters to be tuned'''
g = 9.81
mu = 0.8

axisZ= array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
# %% Cell 2

normals = np.vstack([n1, n2, n3, n4])
comp_dyn = ComputationalDynamics()

''' Add 2D figure '''
mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.unicode'] = True
fig = plt.figure(1)

scale = np.linspace(50, 150, 10)
jet = cm = plt.get_cmap('RdYlGn') 
cNorm  = colors.Normalize(vmin=50, vmax=150)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
idx = 0

for trunk_mass in range(140, 40, -10):
    IP_points, actuation_polygons = comp_dyn.iterative_projection_bretl(constraint_mode_IP, contacts, normals, trunk_mass, ng, mu)
    point = np.vstack([IP_points])
    colorVal = scalarMap.to_rgba(scale[idx])
    colorText = ('color: (%4.2f,%4.2f,%4.2f)'%(colorVal[0],colorVal[1],colorVal[2]))
    idx += 1
    #plotter.plot_polygon(np.transpose(IP_points), x[0],'trunk mass ' + str(trunk_mass*10) + ' N')    
    x = np.hstack([point[:,0], point[0,0]])
    y = np.hstack([point[:,1], point[0,1]])
    h = plt.plot(x,y, color = colorVal, linewidth=5., label = str(trunk_mass*10) + ' N')

h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')
    
plt.grid()
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()
plt.axis('equal')
plt.show()
fig.savefig('../../figs/IP_bretl/instantaneous_actuation_region_3contacts.pdf')
fig.savefig('../../figs/IP_bretl/instantaneous_actuation_region_3contacts.png')

''' Add 3D figure '''

fig = plt.figure(2)
ax = fig.add_subplot(111, projection='3d')

scale = np.linspace(50, 150, 10)
jet = cm = plt.get_cmap('RdYlGn') 
cNorm  = colors.Normalize(vmin=50, vmax=150)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
idx = 0

for trunk_mass in range(140, 40, -10):
    IP_points, actuation_polygons = comp_dyn.iterative_projection_bretl(constraint_mode_IP, contacts, normals, trunk_mass, ng, mu)
    point = np.vstack([IP_points])
    colorVal = scalarMap.to_rgba(scale[idx])
    colorText = ('color: (%4.2f,%4.2f,%4.2f)'%(colorVal[0],colorVal[1],colorVal[2]))
    idx += 1
    #plotter.plot_polygon(np.transpose(IP_points), x[0],'trunk mass ' + str(trunk_mass*10) + ' N')    
    x = np.hstack([point[:,0], point[0,0]])
    y = np.hstack([point[:,1], point[0,1]])
    h = plt.plot(x,y, color = colorVal, linewidth=5., label = str(trunk_mass*10) + ' N')

h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')

'''plot robot'''
r = [-1,1]

X, Y = np.meshgrid(r, r)
trunk_x_half_length = 0.45
trunk_y_half_length = 0.25
trunk_z_half_length = 0.1

comWF = np.array([0.0, 0.0, 0.58])
points = np.array([[-trunk_x_half_length, -trunk_y_half_length, -trunk_z_half_length],
                  [trunk_x_half_length, -trunk_y_half_length, -trunk_z_half_length ],
                  [trunk_x_half_length, trunk_y_half_length, -trunk_z_half_length],
                  [-trunk_x_half_length, trunk_y_half_length, -trunk_z_half_length],
                  [-trunk_x_half_length, -trunk_y_half_length, trunk_z_half_length],
                  [trunk_x_half_length, -trunk_y_half_length, trunk_z_half_length],
                  [trunk_x_half_length, trunk_y_half_length, trunk_z_half_length],
                  [-trunk_x_half_length, trunk_y_half_length, trunk_z_half_length]])
                  
points= points + comWF
                  

P = np.eye((3))

Z = np.zeros((8,3))
for i in range(8): Z[i,:] = np.dot(points[i,:],P)

#ax.scatter3D(Z[:, 0], Z[:, 1], Z[:, 2])


# list of sides' polygons of figure
verts = [[Z[0],Z[1],Z[2],Z[3]],
 [Z[4],Z[5],Z[6],Z[7]], 
 [Z[0],Z[1],Z[5],Z[4]], 
 [Z[2],Z[3],Z[7],Z[6]], 
 [Z[1],Z[2],Z[6],Z[5]],
 [Z[4],Z[7],Z[3],Z[0]], 
 [Z[2],Z[3],Z[7],Z[6]]]

# plot sides
#ax.add_collection3d(Poly3DCollection(verts, 
# facecolors='cyan', linewidths=1, edgecolors='k', alpha=.5))

    
plt.grid()
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_zlabel("z [m]")
ax.legend(loc='upper right')
ax.set_xlim3d([-0.6, 1.0])
ax.set_ylim3d([-0.5, 1.1])
ax.set_zlim3d([0.0, 1.6])
plt.show()
fig.savefig('../../figs/IP_bretl/instantaneous_actuation_region_3contacts_3D.pdf')
fig.savefig('../../figs/IP_bretl/instantaneous_actuation_region_3contacts_3D.png')
