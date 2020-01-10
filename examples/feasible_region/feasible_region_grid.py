# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np
import time
from numpy import array
from jet_leg.plotting.plotting_tools import Plotter
import random
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.dynamics.instantaneous_capture_point import InstantaneousCapturePoint
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization.lp_vertex_redundancy import LpVertexRedundnacy

import matplotlib.pyplot as plt
from jet_leg.plotting.arrow3D import Arrow3D

def computeFeasibleRegionBinaryMatrix(IP_points, resolutionX = 0.01, resolutionY = 0.01, windowSizeX = 0.9, windowSizeY = 0.7):

    binary_matrix = np.zeros((int(windowSizeY/resolutionY), int(windowSizeX/resolutionX)))
    feasible_points = np.zeros((0, 2))
    unfeasible_points = np.zeros((0, 2))

    compGeom = ComputationalGeometry()
    facets = compGeom.compute_halfspaces_convex_hull(IP_points)
    print "facets", facets

    idX = 0
    for point2checkX in np.arange(-windowSizeX/2.0, windowSizeX/2.0, resolutionX):
        idY = 0
        for point2checkY in np.arange(windowSizeY/2.0, -windowSizeY/2.0, -resolutionY):
            point2check = np.array([point2checkX, point2checkY])
            isPointFeasible = compGeom.isPointRedundant(facets, point2check)
            # LPparams.setCoMPosWF(com_WF)
            # isPointFeasible, x = lpCheck.isPointRedundant(IP_points.T, point2check)

            if isPointFeasible:
                binary_matrix[idY, idX] = 1
                feasible_points = np.vstack([feasible_points, point2check])
            else:
                binary_matrix[idY, idX] = 0
                unfeasible_points = np.vstack([unfeasible_points, point2check])
            idY +=1
        idX +=1
    return binary_matrix, feasible_points, unfeasible_points


plt.close('all')
math = Math()

''' Set the robot's name (either 'hyq', 'hyqreal' or 'anymal')'''
robot_name = 'anymal'

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
comWF = np.array([.0, -0.05, 0.0])
comWF_lin_acc = np.array([.0, .0, .0])
comWF_ang_acc = np.array([.0, .0, .0])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForce = np.array([0., .0, 0.0 * 9.81])  # units are N
extCentroidalTorque = np.array([.0, .0, .0])  # units are Nm
extCentroidalWrench = np.hstack([extForce, extCentroidalTorque])

""" contact points in the World Frame"""
LF_foot = np.array([0.3, 0.2, -0.4])
RF_foot = np.array([0.3, -0.2, -0.4])
LH_foot = np.array([-0.3, 0.2, -0.4])
RH_foot = np.array([-0.3, -0.2, -0.4])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

''' parameters to be tuned'''
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
extForceW = np.array([0.0, 0.0, 0.0])  # units are Nm

comp_dyn = ComputationalDynamics(robot_name)

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()

params.pointContacts = False
params.useInstantaneousCapturePoint = True
params.setContactsPosWF(contactsWF)
params.externalCentroidalWrench = extCentroidalWrench
params.setCoMPosWF(comWF)
params.comLinVel = [0.25, 0.0, 0.0]
params.setCoMLinAcc(comWF_lin_acc)
params.setTorqueLims(comp_dyn.robotModel.robotModel.joint_torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setTotalMass(comp_dyn.robotModel.robotModel.trunkMass)
params.externalForceWF = extForceW  # params.externalForceWF is actually used anywhere at the moment

''' compute iterative projection 
Outputs of "iterative_projection_bretl" are:
IP_points = resulting 2D vertices
actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
computation_time = how long it took to compute the iterative projection
'''
IP_points, force_polytopes, IP_computation_time = comp_dyn.iterative_projection_bretl(params)

# print "Inequalities", comp_dyn.ineq
# print "actuation polygons"
# print actuation_polygons

'''I now check whether the given CoM configuration is stable or not'''
isConfigurationStable, contactForces, forcePolytopes = comp_dyn.check_equilibrium(params)
print isConfigurationStable
print 'contact forces', contactForces

''' compute Instantaneous Capture Point (ICP) and check if it belongs to the feasible region '''
if params.useInstantaneousCapturePoint:
    ICP = InstantaneousCapturePoint()
    icp = ICP.compute(params)
    params.instantaneousCapturePoint = icp
    lpCheck = LpVertexRedundnacy()
    isIcpInsideFeasibleRegion, lambdas = lpCheck.isPointRedundant(IP_points.T, icp)


start_t_IP = time.time()
binaryMatrix, feasible_points, unfeasible_points = computeFeasibleRegionBinaryMatrix(IP_points)
print "time for computing the grid points", time.time() - start_t_IP

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
    ax.scatter(contactsWF[idx, 0], contactsWF[idx, 1], contactsWF[idx, 2], c='b', s=100)
    '''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region'''
    if isConfigurationStable:
        ax.scatter(comWF[0], comWF[1], comWF[2], c='g', s=100)
        grf = contactForces[j * 3:j * 3 + 3]
        fz_tot += grf[2]

        ''' draw the set contact forces that respects the constraints'''
        b = Arrow3D([contactsWF[idx, 0], contactsWF[idx, 0] + grf[0] / force_scaling_factor],
                    [contactsWF[idx, 1], contactsWF[idx, 1] + grf[1] / force_scaling_factor],
                    [contactsWF[idx, 2], contactsWF[idx, 2] + grf[2] / force_scaling_factor], mutation_scale=20, lw=3,
                    arrowstyle="-|>",
                    color="b")
        ax.add_artist(b)
    else:
        ax.scatter(comWF[0], comWF[1], comWF[2], c='r', s=100)

    ''' draw 3D arrows corresponding to contact normals'''
    a = Arrow3D([contactsWF[idx, 0], contactsWF[idx, 0] + normals[idx, 0] / 10],
                [contactsWF[idx, 1], contactsWF[idx, 1] + normals[idx, 1] / 10],
                [contactsWF[idx, 2], contactsWF[idx, 2] + normals[idx, 2] / 10], mutation_scale=20, lw=3,
                arrowstyle="-|>", color="r")

    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    ax.scatter(contactsWF[idx, 0], contactsWF[idx, 1], 0.0, c='k', s=100)
    ax.add_artist(a)

print 'sum of vertical forces is', fz_tot

''' plotting Iterative Projection points '''
plotter = Plotter()
for j in range(0, nc):  # this will only show the force polytopes of the feet that are defined to be in stance
    idx = int(stanceID[j])
    plotter.plot_polygon(np.transpose(IP_points))
    if (constraint_mode_IP[idx] == 'ONLY_ACTUATION') or (constraint_mode_IP[idx] == 'FRICTION_AND_ACTUATION'):
        plotter.plot_actuation_polygon(ax, forcePolytopes.getVertices()[idx], contactsWF[idx, :], force_scaling_factor)

''' 2D figure '''
plt.figure()
for j in range(0,
               nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    h1 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1], 'ko', markersize=15, label='stance feet')
h2 = plotter.plot_polygon(np.transpose(IP_points), '--b', 'Support Region')

'''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region)'''
if isConfigurationStable:
    plt.plot(comWF[0], comWF[1], 'go', markersize=15, label='CoM')
else:
    plt.plot(comWF[0], comWF[1], 'ro', markersize=15, label='CoM')

if params.useInstantaneousCapturePoint:
    if isIcpInsideFeasibleRegion:
        plt.plot(params.instantaneousCapturePoint[0], params.instantaneousCapturePoint[1], 'gs', markersize=15,
                 label='ICP')
    else:
        plt.plot(params.instantaneousCapturePoint[0], params.instantaneousCapturePoint[1], 'rs', markersize=15,
                 label='ICP')

''' plotting Iterative Projection points '''
feasiblePointsSize = np.size(feasible_points, 0)
#for j in range(0, feasiblePointsSize):
#    plt.scatter(feasible_points[j, 0], feasible_points[j, 1], c='g', s=50)
#    lastFeasibleIndex = j
#
unfeasiblePointsSize = np.size(unfeasible_points, 0)
#for j in range(0, unfeasiblePointsSize):
#    plt.scatter(unfeasible_points[j, 0], unfeasible_points[j, 1], c='r', s=50)
#    lastUnfeasibleIndex = j

print "number of feasible points", feasiblePointsSize
print "number of unfeasible points", unfeasiblePointsSize,

plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()

plt.figure()
plt.imshow(binaryMatrix, cmap='gray')
plt.show()
