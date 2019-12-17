# -*- coding: utf-8 -*-
"""
Created on Fri Aug 10 16:08:43 2018

@author: romeoorsolino
"""

import numpy as np

from context import jet_leg 

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix
from numpy.linalg import norm

from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters

import matplotlib as mpl
import matplotlib.colors as colors
import matplotlib.cm as cmx
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge, Polygon
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from matplotlib.collections import PatchCollection

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

# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION or ONLY_FRICTION or FRICTION_AND_ACTUATION
#constraint_mode = 'ONLY_ACTUATION'
constraint_mode = ['FRICTION_AND_ACTUATION',
                   'FRICTION_AND_ACTUATION',
                   'FRICTION_AND_ACTUATION',
                   'FRICTION_AND_ACTUATION']
                   
useVariableJacobian = False

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

contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
stanceLegs = [1,1,1,1]
nc = np.sum(stanceLegs)
stanceIndex = []
swingIndex = []
print 'stance', stanceLegs
for iter in range(0, 4):
    if stanceLegs[iter] == 1:
#               print 'new poly', stanceIndex, iter
        stanceIndex = np.hstack([stanceIndex, iter])
    else:
        swingIndex = iter
        
''' parameters to be tuned'''
g = 9.81
mu = 0.8
comWF = np.array([0.0, 0.0, 0.0])
axisZ= array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
# %% Cell 2

normals = np.vstack([n1, n2, n3, n4])
LF_tau_lim = [50.0, 100.0, 100.0]
RF_tau_lim = [50.0, 100.0, 100.0]
LH_tau_lim = [50.0, 100.0, 100.0]
RH_tau_lim = [50.0, 100.0, 100.0]
torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
comp_dyn = ComputationalDynamics('hyq')

''' Add 2D figure '''
mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.unicode'] = True
fig = plt.figure(1)

scale = np.linspace(50, 150, 10)
jet = cm = plt.get_cmap('RdYlGn') 
cNorm  = colors.Normalize(vmin=50, vmax=150)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
idx = 0

params = IterativeProjectionParameters()
params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceLegs)
params.setConstraintModes(constraint_mode)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)

legends_to_show = [0,1,-2,-1]

mass_range = range(140, 40, -10)
for total_mass in mass_range:
    params.setTotalMass(total_mass)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
    point = np.vstack([IP_points])
    colorVal = scalarMap.to_rgba(scale[idx])
    colorText = ('color: (%4.2f,%4.2f,%4.2f)'%(colorVal[0],colorVal[1],colorVal[2]))
    idx += 1
    #plotter.plot_polygon(np.transpose(IP_points), x[0],'trunk mass ' + str(trunk_mass*10) + ' N')    
    x = np.hstack([point[:,0], point[0,0]])
    y = np.hstack([point[:,1], point[0,1]])
    if total_mass is 140 or total_mass is 130 or total_mass is 50 or total_mass is 60:
        h = plt.plot(x,y, color = colorVal, linewidth=5., label = str(total_mass*10) + ' N')
    elif total_mass is 110 or total_mass is 80:
        h = plt.plot(x, y, color=colorVal, linewidth=5., label='$\dots$')
    else:
        h = plt.plot(x, y, color=colorVal, linewidth=5.)

h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')
constraint_mode = ['ONLY_FRICTION',
                   'ONLY_FRICTION',
                   'ONLY_FRICTION',
                   'ONLY_FRICTION']
params.setConstraintModes(constraint_mode)
IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
point = np.vstack([IP_points])
x = np.hstack([point[:,0], point[0,0]])
y = np.hstack([point[:,1], point[0,1]])
h2 = plt.plot(x,y, color = 'blue', linestyle='dashed', linewidth=5.,markersize=15, label = 'support region')

plt.rc('font', family='serif', size=20)
plt.grid()
plt.xlabel("x [m]",fontsize=25)
plt.ylabel("y [m]",fontsize=25)
plt.xticks(np.arange(-1, 1, step=0.1),fontsize=20)
plt.yticks(np.arange(-0.4, 1., step=0.1),fontsize=20)
plt.title("friction and actuation", fontsize = 25).set_position([.5, 1.01])
ax = plt.subplot(111)
chartBox = ax.get_position()
ax.set_position([chartBox.x0+0.05, chartBox.y0+0.045, chartBox.width, chartBox.height])
#plt.legend(loc='upper right', bbox_to_anchor=(1.0, 1.0))

plt.axis('equal')
#plt.axis([-0.4, 0.6, -0.4, 1.])
#ax.legend(loc='upper center',bbox_to_anchor=(0.75, 1.0))
#plt.show()
fig.savefig('../../figs/4contacts_friction_and_actuation.pdf')
#plt.savefig('../../figs/4contacts_only_actuation.png')

''' Add 3D figure '''

fig = plt.figure(2)
ax = fig.add_subplot(111, projection='3d')

scale = np.linspace(50, 150, 10)
jet = cm = plt.get_cmap('RdYlGn') 
cNorm  = colors.Normalize(vmin=50, vmax=150)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
idx = 0
constraint_mode = ['ONLY_ACTUATION',
                   'ONLY_ACTUATION',
                   'ONLY_ACTUATION',
                   'ONLY_ACTUATION']

params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceLegs)
params.setConstraintModes(constraint_mode)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)


for total_mass in range(140, 40, -10):
    params.setTotalMass(total_mass)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
    point = np.vstack([IP_points])
    colorVal = scalarMap.to_rgba(scale[idx])
    colorText = ('color: (%4.2f,%4.2f,%4.2f)'%(colorVal[0],colorVal[1],colorVal[2]))
    idx += 1
    #plotter.plot_polygon(np.transpose(IP_points), x[0],'trunk mass ' + str(trunk_mass*10) + ' N')    
    x = np.hstack([point[:,0], point[0,0]])
    y = np.hstack([point[:,1], point[0,1]])
    h = plt.plot(x,y, color = colorVal, linewidth=5., label = str(total_mass*10) + ' N')

h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')
constraint_mode = ['ONLY_FRICTION',
                   'ONLY_FRICTION',
                   'ONLY_FRICTION',
                   'ONLY_FRICTION']
params.setConstraintModes(constraint_mode)
IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
point = np.vstack([IP_points])
x = np.hstack([point[:,0], point[0,0]])
y = np.hstack([point[:,1], point[0,1]])
h2 = plt.plot(x,y, color = 'blue', linestyle='dashed', linewidth=5., label = 'only friction')

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
#ax.legend(loc='upper right')
ax.set_xlim3d([-0.6, 1.0])
ax.set_ylim3d([-0.5, 1.1])
ax.set_zlim3d([0.0, 1.6])
plt.show()
fig.savefig('../../figs/instantaneous_actuation_region_3contacts_3D.pdf')
