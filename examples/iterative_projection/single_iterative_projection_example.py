# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from jet_leg.plotting.plotting_tools import Plotter
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.dynamics.instantaneous_capture_point import InstantaneousCapturePoint
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.optimization.lp_vertex_redundancy import LpVertexRedundnacy
from jet_leg.feasibility.find_feasible_trajectories import FeasibilityAnalysis

import matplotlib.pyplot as plt
from jet_leg.plotting.arrow3D import Arrow3D
from copy import copy
import sys

plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = sys.argv[1]
print('robot name:', robot)

params = IterativeProjectionParameters(robot)
print('after first')
f = FeasibilityAnalysis(params.pin)


urdf_hip_id_lf = 1
urdf_hip_id_rf = 4
urdf_hip_id_lh = 7
urdf_hip_id_rh = 10

body_length = 0.33
params.pin.model.jointPlacements[urdf_hip_id_lf].translation[0] = body_length
params.pin.model.jointPlacements[urdf_hip_id_rf].translation[0] = body_length
params.pin.model.jointPlacements[urdf_hip_id_lh].translation[0] = -body_length
params.pin.model.jointPlacements[urdf_hip_id_rh].translation[0] = -body_length

links_length = 0.3
#pin.model.jointPlacements[urdf_hip_id_lf+1].translation[0] = links_length

FL_foot_frame_id = params.pin.model.getFrameId('FL_foot')
HL_foot_frame_id = params.pin.model.getFrameId('HL_foot')
FR_foot_frame_id = params.pin.model.getFrameId('FR_foot')
HR_foot_frame_id = params.pin.model.getFrameId('HR_foot')
FL_foot_frame = params.pin.model.frames[FL_foot_frame_id]
HL_foot_frame = params.pin.model.frames[HL_foot_frame_id]
FR_foot_frame = params.pin.model.frames[FR_foot_frame_id]
HR_foot_frame = params.pin.model.frames[HR_foot_frame_id]
FL_foot_frame.placement.translation[2] = -links_length
HL_foot_frame.placement.translation[2] = -links_length
FR_foot_frame.placement.translation[2] = -links_length
HR_foot_frame.placement.translation[2] = -links_length
print(FL_foot_frame.placement.translation)
print(HL_foot_frame.placement.translation)
print(FR_foot_frame.placement.translation)
print(HR_foot_frame.placement.translation)

FL_calf_frame = params.pin.model.jointPlacements[urdf_hip_id_lf+2]
HL_calf_frame = params.pin.model.jointPlacements[urdf_hip_id_rf+2]
FR_calf_frame = params.pin.model.jointPlacements[urdf_hip_id_lh+2]
HR_calf_frame = params.pin.model.jointPlacements[urdf_hip_id_rh+2]
FL_calf_frame.translation[2] = -links_length
HL_calf_frame.translation[2] = -links_length
FR_calf_frame.translation[2] = -links_length
HR_calf_frame.translation[2] = -links_length
print(FL_calf_frame.translation)
print(HL_calf_frame.translation)
print(FR_calf_frame.translation)
print(HR_calf_frame.translation)

params.setDefaultValuesWrtWorld(params.pin)

''' compute iterative projection
Outputs of "iterative_projection_bretl" are:
IP_points = resulting 2D vertices
actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
computation_time = how long it took to compute the iterative projection
'''

IP_points, force_polytopes, IP_computation_time, joints_pos, knee_pos, hips_pos = params.compDyn.iterative_projection_bretl(
    params)

# print "Inequalities", params.compDyn.ineq
# print "actuation polygons"
# print actuation_polygons

'''I now check whether the given CoM configuration is stable or not'''
# isCoMStable, contactForces, forcePolytopes = params.compDyn.check_equilibrium(params)
# print "is CoM stable?", isCoMStable
# print 'Contact forces:', contactForces

comp_geom = ComputationalGeometry()
facets = comp_geom.compute_halfspaces_convex_hull(IP_points)
point2check = params.compDyn.getReferencePoint(params, "ZMP")
isPointFeasible, margin = comp_geom.isPointRedundant(facets, point2check)
print("isPointFeasible: ", isPointFeasible)
print("Margin is: ", margin)

''' compute Instantaneous Capture Point (ICP) and check if it belongs to the feasible region '''
if params.useInstantaneousCapturePoint:
    ICP = InstantaneousCapturePoint()
    icp = ICP.compute(params)
    params.instantaneousCapturePoint = icp
    lpCheck = LpVertexRedundnacy()
    isIcpInsideFeasibleRegion, lambdas = lpCheck.isPointRedundant(
        IP_points.T, icp)

'''Plotting the contact points in the 3D figure'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
comWF = params.getCoMPosWF()
ax.set_xlim(comWF[0]-0.5, comWF[0]+0.5)
ax.set_ylim(comWF[1]-0.5, comWF[1]+0.5)
ax.set_zlim(comWF[2]-0.8, comWF[2]+0.2)

stanceFeet = params.getStanceFeet()
nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)
force_scaling_factor = 1500

fz_tot = 0.0
shoulder_position_WF = np.zeros((4, 3))
contactsWF = params.getContactsPosWF()
normals = params.getNormals()
contactsBF = params.getContactsPosBF()
rpy_base = params.getOrientation()
rot = Rot.from_euler(
    'xyz', [rpy_base[0], rpy_base[1], rpy_base[2]], degrees=False)
W_R_B = rot.as_matrix()
for j in range(0, nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ax.scatter(contactsWF[idx, 0], contactsWF[idx, 1],
               contactsWF[idx, 2], c='b', s=100)
    '''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region'''
    if isPointFeasible:
        ax.scatter(comWF[0], comWF[1], comWF[2], c='g', s=100)

    else:
        ax.scatter(comWF[0], comWF[1], comWF[2], c='r', s=100)

    ''' draw 3D arrows corresponding to contact normals'''
    a = Arrow3D([contactsWF[idx, 0], contactsWF[idx, 0]+normals[idx, 0]/10], [contactsWF[idx, 1], contactsWF[idx, 1]+normals[idx, 1]/10],
                [contactsWF[idx, 2], contactsWF[idx, 2]+normals[idx, 2]/10], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")

    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    print('hip pos', hips_pos[0])
    shoulder_position_BF = [
        float(hips_pos[idx][0]), float(hips_pos[idx][1]), float(hips_pos[idx][2])]
    rpy = params.getOrientation()
    shoulder_position_WF[j, :] = W_R_B.dot(shoulder_position_BF) + comWF
    ax.scatter(shoulder_position_WF[j, 0], shoulder_position_WF[j,
                                                                1], shoulder_position_WF[j, 2], c='k', s=100)
    ax.add_artist(a)

''' plotting Iterative Projection points '''
plotter = Plotter()
for j in range(0, nc):  # this will only show the force polytopes of the feet that are defined to be in stance
    idx = int(stanceID[j])
    plotter.plot_polygon(np.transpose(IP_points))
    if (params.getConstraintModes()[idx] == 'ONLY_ACTUATION') or (params.getConstraintModes()[idx] == 'FRICTION_AND_ACTUATION'):
        plotter.plot_actuation_polygon(ax, force_polytopes.getVertices()[
            idx], contactsWF[idx, :], force_scaling_factor)

for j in range(0, nc):
    idx = int(stanceID[j])
    knee_BF = [float(knee_pos[idx][0]), float(knee_pos[idx][1]),
               float(knee_pos[idx][2])]
    knee_pos_WF = W_R_B.dot(knee_BF) + comWF
    legs = np.vstack([shoulder_position_WF[j, :],
                      knee_pos_WF, contactsWF[idx, :]])
    ax.plot(legs[:, 0], legs[:, 1], legs[:, 2], '-k')

    step_distance = 1.0
    step_height = 0.5
    if f.kneeStepCollision(step_distance, step_height, knee_pos_WF):
        print('Knee collision detected! Knee pos in WF is', knee_pos_WF)

for j in range(0, 4):
    shoulder_position_BF = [
        float(hips_pos[j][0]), float(hips_pos[j][1]), float(hips_pos[j][2])]
    rpy = params.getOrientation()
    shoulder_position_WF[j, :] = W_R_B.dot(shoulder_position_BF) + comWF
tmp = copy(shoulder_position_WF[2, :])
shoulder_position_WF[2, :] = shoulder_position_WF[3, :]
shoulder_position_WF[3, :] = tmp
base_polygon = np.vstack([shoulder_position_WF, shoulder_position_WF[0, :]])
ax.plot(base_polygon[:, 0], base_polygon[:, 1], base_polygon[:, 2], '--k')

''' 2D figure '''
fig = plt.figure()
for j in range(0, nc):  # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    shoulder_position_BF = [contactsBF[idx, 0], contactsBF[idx, 1], 0.4]
    shoulder_position_WF[j, :] = np.dot(W_R_B, shoulder_position_BF) + comWF
    idx = int(stanceID[j])
    print('stance indices', idx)
    h1 = plt.plot(contactsWF[idx, 0], contactsWF[idx, 1],
                  'ko', markersize=15, label='stance feet')

    h2 = plotter.plot_polygon(np.transpose(
        IP_points), '--b', 5, 'Feasible Region')

    '''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region)'''
    if isPointFeasible:
        plt.plot(point2check[0], point2check[1], 'go',
                 markersize=15, label='ground reference point')
    else:
        plt.plot(point2check[0], point2check[1], 'ro',
                 markersize=15, label='ground reference point')

plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.show()
