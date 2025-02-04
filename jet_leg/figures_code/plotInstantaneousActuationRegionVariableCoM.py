# -*- coding: utf-8 -*-
"""
Created on Fri Aug 10 16:08:43 2018

@author: romeoorsolino
"""

import numpy as np

from context import jet_leg 

from numpy import array
from numpy.linalg import norm
import matplotlib.path as mpath

from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters

import matplotlib as mpl
import matplotlib.colors as colors
import matplotlib.cm as cmx
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge, Polygon
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from matplotlib.collections import PatchCollection
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

from jet_leg.arrow3D import Arrow3D

def colorline(
    x, y, z=None, cmap=plt.get_cmap('copper'), norm=plt.Normalize(0.0, 1.0),
        linewidth=15, alpha=1.0):
    """
    http://nbviewer.ipython.org/github/dpsanders/matplotlib-examples/blob/master/colorline.ipynb
    http://matplotlib.org/examples/pylab_examples/multicolored_line.html
    Plot a colored line with coordinates x and y
    Optionally specify colors in the array z
    Optionally specify a colormap, a norm function and a line width
    """

    # Default colors equally spaced on [0,1]:
    if z is None:
        z = np.linspace(0.0, 1.0, len(x))

    # Special case if a single number:
    if not hasattr(z, "__iter__"):  # to check for numerical input -- this is a hack
        z = np.array([z])

    z = np.asarray(z)

    segments = make_segments(x, y)
    lc = mcoll.LineCollection(segments, array=z, cmap=cmap, norm=norm,
                              linewidth=linewidth, alpha=alpha)

    ax = plt.gca()
    ax.add_collection(lc)

    return lc


def make_segments(x, y):
    """
    Create list of line segments from x and y coordinates, in the correct format
    for LineCollection: an array of the form numlines x (points per line) x 2 (x
    and y) array
    """

    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    return segments
    
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
nc = 4
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION or ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode = ['FRICTION_AND_ACTUATION',
                   'FRICTION_AND_ACTUATION',
                   'FRICTION_AND_ACTUATION',
                   'FRICTION_AND_ACTUATION']
                   
useVariableJacobian = False
# number of decision variables of the problem
n = nc*6

# contact positions
""" contact points """
#LF_foot = np.array([0.3, 0.2, -.9])
#RF_foot = np.array([0.3, -0.2, -0.5])
#LH_foot = np.array([-0.3, 0.2, -0.5])
#RH_foot = np.array([-0.3, -0.2, -0.5])

LF_foot = np.array([0.3, 0.2, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.3, 0.2, -0.5])
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

axisZ= array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
# %% Cell 2

normals = np.vstack([n1, n2, n3, n4])
comp_dyn = ComputationalDynamics()
comWF = np.array([0.0, 0.0, 0.0])
trunk_mass = 90
idx = 0

LF_tau_lim = [50.0, 100.0, 100.0]
RF_tau_lim = [50.0, 100.0, 100.0]
LH_tau_lim = [50.0, 100.0, 100.0]
RH_tau_lim = [50.0, 100.0, 100.0]
torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])

params = IterativeProjectionParameters()
params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceLegs)
params.setConstraintModes(constraint_mode)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(trunk_mass)

lowel_lim = -30
upper_lim = 40
scale = np.linspace(lowel_lim, upper_lim, 7)
jet = cm = plt.get_cmap('seismic') 
cNorm  = colors.Normalize(vmin=-30, vmax=35)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
comToStackX = []
comToStackY = []
for com_x in range(lowel_lim, upper_lim, 10):
#    print com_x/100.0
    comWF = np.array([com_x/100.0, 0.0, 0.0])
#    print comWF, comToStack
    comToStackX = np.hstack([comToStackX, comWF[0]])
    comToStackY = np.hstack([comToStackY, comWF[1]])

#    params.setCoMPos(comWF)
    LF_foot_tmp = LF_foot - comWF
    RF_foot_tmp = RF_foot - comWF
    LH_foot_tmp = LH_foot - comWF
    RH_foot_tmp = RH_foot - comWF
    contactsBF = np.vstack((LF_foot_tmp, RF_foot_tmp, LH_foot_tmp, RH_foot_tmp))
    params.setContactsPosWF(contactsBF)
    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
    point = np.vstack([IP_points])
#    print idx, scale
#    print scale[idx]
    colorVal = scalarMap.to_rgba(scale[idx])
    colorText = ('color: (%4.2f,%4.2f,%4.2f)'%(colorVal[0],colorVal[1],colorVal[2]))
    idx += 1
    #plotter.plot_polygon(np.transpose(IP_points), x[0],'trunk mass ' + str(trunk_mass*10) + ' N')    
    x = np.hstack([point[:,0], point[0,0]])
    y = np.hstack([point[:,1], point[0,1]])
    h = plt.plot(x,y, color = colorVal, linewidth=5., label = str(com_x/100.0) + ' cm')
    

constraint_mode = ['ONLY_FRICTION',
                   'ONLY_FRICTION',
                   'ONLY_FRICTION',
                   'ONLY_FRICTION']
params.setConstraintModes(constraint_mode)
params.setContactsPosWF(contacts)
IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
point = np.vstack([IP_points])
x = np.hstack([point[:,0], point[0,0]])
y = np.hstack([point[:,1], point[0,1]])
h2 = plt.plot(x,y, color = 'gray', linestyle='dashed', linewidth=5., label = 'only friction')

idx = 0
for com_x in range(lowel_lim, upper_lim, 10):
    colorVal = scalarMap.to_rgba(scale[idx])
    colorText = ('color: (%4.2f,%4.2f,%4.2f)'%(colorVal[0],colorVal[1],colorVal[2]))
    idx += 1
    comWF = np.array([com_x/100.0, 0.0, 0.0])
    print idx, com_x
    if idx <=2 or idx >= 6:
        h2 = plt.plot(comWF[0], comWF[1], color = colorVal, marker='s', markersize=15)
    else:
        h2 = plt.plot(comWF[0], comWF[1], color = colorVal, marker='^', markersize=15)

path = mpath.Path(np.column_stack([comToStackX, comToStackY]))
verts = path.interpolated(steps=3).vertices
x, y = verts[:, 0], verts[:, 1]
z = np.linspace(0, 1, len(x))
#print x, y, z
#colorline(x, y, z, cmap=plt.get_cmap('seismic'), linewidth=5)

#print comToStack
''' Add 2D figure '''
mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.unicode'] = True


fig = plt.figure(1)
h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')
#h2 = plt.plot(comToStackX,comToStackY,'ko',markersize=15, label='path')
plt.grid()
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend()
plt.axis('equal')
plt.show()
#fig.savefig('../../figs/IP_bretl/instantaneous_actuation_region_variable_com.pdf')
#fig.savefig('../../figs/IP_bretl/instantaneous_actuation_region_variable_com.png')

''' Add 3D figure '''

#fig = plt.figure(2)
#ax = fig.add_subplot(111, projection='3d')
#
#scale = np.linspace(50, 150, 10)
#jet = cm = plt.get_cmap('RdYlGn') 
#cNorm  = colors.Normalize(vmin=50, vmax=150)
#scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)
#idx = 0
#
#comWF = np.array([com_x/10., 0.0, 0.58])
#
#for com_x in range(-10, 10, 5):
#    comWF = np.array([com_x/100.0, 0.0, 0.0])
#    params.setCoMPos(comWF)
#    IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
#    point = np.vstack([IP_points])
#    colorVal = scalarMap.to_rgba(scale[idx])
#    colorText = ('color: (%4.2f,%4.2f,%4.2f)'%(colorVal[0],colorVal[1],colorVal[2]))
#    idx += 1
#    #plotter.plot_polygon(np.transpose(IP_points), x[0],'trunk mass ' + str(trunk_mass*10) + ' N')    
#    x = np.hstack([point[:,0], point[0,0]])
#    y = np.hstack([point[:,1], point[0,1]])
#    h = plt.plot(x,y, color = colorVal, linewidth=5., label = str(trunk_mass*10) + ' N')
#
#h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')
#
#'''plot robot'''
#r = [-1,1]
#
#X, Y = np.meshgrid(r, r)
#trunk_x_half_length = 0.45
#trunk_y_half_length = 0.25
#trunk_z_half_length = 0.1
#
#points = np.array([[-trunk_x_half_length, -trunk_y_half_length, -trunk_z_half_length],
#                  [trunk_x_half_length, -trunk_y_half_length, -trunk_z_half_length ],
#                  [trunk_x_half_length, trunk_y_half_length, -trunk_z_half_length],
#                  [-trunk_x_half_length, trunk_y_half_length, -trunk_z_half_length],
#                  [-trunk_x_half_length, -trunk_y_half_length, trunk_z_half_length],
#                  [trunk_x_half_length, -trunk_y_half_length, trunk_z_half_length],
#                  [trunk_x_half_length, trunk_y_half_length, trunk_z_half_length],
#                  [-trunk_x_half_length, trunk_y_half_length, trunk_z_half_length]])
#                  
#points= points + comWF
#                  
#
#P = np.eye((3))
#
#Z = np.zeros((8,3))
#for i in range(8): Z[i,:] = np.dot(points[i,:],P)
#
##ax.scatter3D(Z[:, 0], Z[:, 1], Z[:, 2])
#
#
## list of sides' polygons of figure
#verts = [[Z[0],Z[1],Z[2],Z[3]],
# [Z[4],Z[5],Z[6],Z[7]], 
# [Z[0],Z[1],Z[5],Z[4]], 
# [Z[2],Z[3],Z[7],Z[6]], 
# [Z[1],Z[2],Z[6],Z[5]],
# [Z[4],Z[7],Z[3],Z[0]], 
# [Z[2],Z[3],Z[7],Z[6]]]

# plot sides
#ax.add_collection3d(Poly3DCollection(verts, 
# facecolors='cyan', linewidths=1, edgecolors='k', alpha=.5))

    
#plt.grid()
#ax.set_xlabel("x [m]")
#ax.set_ylabel("y [m]")
#ax.set_zlabel("z [m]")
#ax.legend(loc='upper right')
#ax.set_xlim3d([-0.6, 1.0])
#ax.set_ylim3d([-0.5, 1.1])
#ax.set_zlim3d([0.0, 1.6])
#plt.show()
#fig.savefig('../../figs/IP_bretl/instantaneous_actuation_region_variable_com_3D.pdf')
#fig.savefig('../../figs/IP_bretl/instantaneous_actuation_region_variable_com_3D.png')
