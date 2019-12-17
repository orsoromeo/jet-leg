# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from numpy import array
from jet_leg.plotting.plotting_tools import Plotter
import random
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters

import matplotlib.pyplot as plt
from jet_leg.plotting.arrow3D import Arrow3D

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
# n = nc*6
comWF = np.array([0.0, 0.0, 0.0])

""" contact points in the World Frame"""
LF_foot = np.array([0.4, 0.3, -0.5])
RF_foot = np.array([0.3, -0.2, -0.5])
LH_foot = np.array([-0.3, 0.2, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.5])

contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

# contacts = contactsToStack[0:nc+1, :]
# print contacts

''' parameters to be tuned'''
g = 9.81
trunk_mass = 85.
mu = 0.5

''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1, 1, 1, 1]

randomSwingLeg = random.randint(0, 3)
tripleStance = False  # if you want you can define a swing leg using this variable
if tripleStance:
    print 'Swing leg', randomSwingLeg
    stanceFeet[randomSwingLeg] = 0
print 'stanceLegs ', stanceFeet

''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
axisZ = array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LF
n2 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RF
n3 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LH
n4 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RH
normals = np.vstack([n1, n2, n3, n4])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForceW = np.array([0.0, 0.0, 0.0])  # units are Nm

comp_dyn = ComputationalDynamics('anymal')

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()
params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
params.setTorqueLims(comp_dyn.robotModel.robotModel.torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(trunk_mass)
params.externalForceWF = extForceW  # params.externalForceWF is actually used anywhere at the moment

# print "Inequalities", comp_dyn.ineq
# print "actuation polygons"
# print actuation_polygons

'''I now check whether the given CoM configuration is stable or not'''
C, d, isIKoutOfWorkSpace, forcePolytopes = comp_dyn.constr.getInequalities(params)

'''Plotting the contact points in the 3D figure'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)
force_scaling_factor = 1500
# plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)
fz_tot = 0.0
for j in range(0,
               nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ax.scatter(contacts[idx, 0], contacts[idx, 1], contacts[idx, 2], c='b', s=100)

    ''' draw 3D arrows corresponding to contact normals'''
    a = Arrow3D([contacts[idx, 0], contacts[idx, 0] + normals[idx, 0] / 10],
                [contacts[idx, 1], contacts[idx, 1] + normals[idx, 1] / 10],
                [contacts[idx, 2], contacts[idx, 2] + normals[idx, 2] / 10], mutation_scale=20, lw=3, arrowstyle="-|>",
                color="r")

    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    ax.scatter(contacts[idx, 0], contacts[idx, 1], 0.0, c='k', s=100)
    ax.add_artist(a)

print 'sum of vertical forces is', fz_tot

''' plotting Iterative Projection points '''
plotter = Plotter()
for j in range(0, nc):  # this will only show the force polytopes of the feet that are defined to be in stance
    idx = int(stanceID[j])
    if (constraint_mode_IP[idx] == 'ONLY_ACTUATION') or (constraint_mode_IP[idx] == 'FRICTION_AND_ACTUATION'):
        plotter.plot_actuation_polygon(ax, forcePolytopes[idx], contacts[idx, :], force_scaling_factor)

plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.show()