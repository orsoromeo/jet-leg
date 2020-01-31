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

''' Set the robot's name (either 'hyq', 'hyqreal' or 'anymal')'''
robot_name = 'anymal'

''' number of generators, i.e. rays/edges used to linearize the friction cone '''
ng = 4

'''
possible constraints for each foot:
 ONLY_ACTUATION = only joint-torque limits are enforces
 ONLY_FRICTION = only friction cone constraints are enforced
 FRICTION_AND_ACTUATION = both friction cone constraints and joint-torque limits
'''
constraint_mode_IP = ['ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION']

# number of decision variables of the problem
# n = nc*6
comWF = np.array([0., 0., 0.0])
comWF_lin_acc = np.array([.0, .0, .0])
comWF_ang_acc = np.array([.0, .0, .0])

""" contact points in the World Frame"""
LF_foot = np.array([0.3, 0.2, -0.4])
RF_foot = np.array([0.3, -0.2, -0.4])
LH_foot = np.array([-0.3, 0.2, -0.4])
RH_foot = np.array([-0.3, -0.2, -0.4])

contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

# contacts = contactsToStack[0:nc+1, :]
# print contacts

''' parameters to be tuned'''
trunk_mass = 45.
mu = 0.5

''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [0, 1, 1, 0]

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
extForce = np.array([0., 0.0, 0.0]) # units are N
extCentroidalTorque = np.array([.0, .0, .0]) # units are Nm
extCentroidalWrench = np.hstack([extForce, extCentroidalTorque])

comp_dyn = ComputationalDynamics(robot_name)

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()

params.setContactsPosWF(contacts)
params.useContactTorque = False
params.externalCentroidalWrench = extCentroidalWrench
params.setCoMPosWF(comWF)
params.setCoMLinAcc(comWF_lin_acc)
params.setTorqueLims(comp_dyn.robotModel.robotModel.joint_torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(comp_dyn.robotModel.robotModel.trunkMass)


# print "Inequalities", comp_dyn.ineq
# print "actuation polygons"
# print actuation_polygons

'''I now check whether the given CoM configuration is stable or not'''
isConfigurationStable, contactForces, forcePolytopes = comp_dyn.check_equilibrium(params)
print isConfigurationStable
print 'contact forces', contactForces

'''Plotting the contact points in the 3D figure'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)
force_scaling_factor = 1000
# plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)
fz_tot = 0.0
for j in range(0,
               nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ax.scatter(contacts[idx, 0], contacts[idx, 1], contacts[idx, 2], c='b', s=100)
    '''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region'''
    if isConfigurationStable:
        ax.scatter(comWF[0], comWF[1], comWF[2], c='g', s=100)
        if not params.useContactTorque:
            grf = contactForces[j * 3:j * 3 + 3]
            fz_tot += grf[2]
        else:
            grf = contactForces[j * 5:j * 5 + 3]
            fz_tot += grf[2]

        ''' draw the set contact forces that respects the constraints'''
        b = Arrow3D([contacts[idx, 0], contacts[idx, 0] + grf[0] / force_scaling_factor],
                    [contacts[idx, 1], contacts[idx, 1] + grf[1] / force_scaling_factor],
                    [contacts[idx, 2], contacts[idx, 2] + grf[2] / force_scaling_factor], mutation_scale=20, lw=3,
                    arrowstyle="-|>",
                    color="b")
        ax.add_artist(b)
    else:
        ax.scatter(comWF[0], comWF[1], comWF[2], c='r', s=100)

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
        plotter.plot_actuation_polygon(ax, forcePolytopes.getVertices()[idx], contacts[idx,:], force_scaling_factor)

''' 2D figure '''
plt.figure()
for j in range(0,
               nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    h1 = plt.plot(contacts[idx, 0], contacts[idx, 1], 'ko', markersize=15, label='stance feet')


'''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region'''
if isConfigurationStable:
    plt.plot(comWF[0], comWF[1], 'go', markersize=15, label='CoM')
else:
    plt.plot(comWF[0], comWF[1], 'ro', markersize=15, label='CoM')

inertialForce = comWF_lin_acc*params.getTotalMass()/force_scaling_factor
plt.arrow(comWF[0], comWF[1], inertialForce[0], inertialForce[1], head_width=0.01, head_length=0.01, fc='k',
              ec='orange', label='inertial acceleration')
plt.arrow(comWF[0] + inertialForce[0], comWF[1] + inertialForce[1], extForce[0]/force_scaling_factor, extForce[1]/force_scaling_factor, head_width=0.01,
              head_length=0.01, fc='blue', ec='blue', label='external force')

plt.scatter([comWF[0], comWF[0] + inertialForce[0]/force_scaling_factor], [comWF[1], comWF[1] + inertialForce[1]/force_scaling_factor], color='k', marker= '^', label='inertial acceleration')
plt.scatter([comWF[0], comWF[0] + inertialForce[0]/force_scaling_factor], [comWF[1], comWF[1] + inertialForce[1]/force_scaling_factor], color='b', marker= '^', label='external force')

plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.show()