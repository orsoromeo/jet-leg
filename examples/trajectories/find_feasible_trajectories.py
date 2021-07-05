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

import matplotlib.pyplot as plt
from jet_leg.plotting.arrow3D import Arrow3D
from copy import copy
        
plt.close('all')
math = Math()

''' Set the robot's name (current options: 'hyq', 'hyqreal', 'anymal_boxy', 'anymal_coyote' or 'lemo_EP0')'''
robot = "lemo_EP0"
params = IterativeProjectionParameters(robot)
comp_dyn = ComputationalDynamics(robot)
params.setDefaultValuesWrtWorld()


''' Generate trajectory of footholds'''
step_height = 0.2
step_distance = 0.5

des_height = 0.45
start_point = [0.0, 0.0, des_height]
dist_from_goal = 2.0*step_distance
goal_point = [dist_from_goal, 0.0, des_height + step_height]

avg_pitch = np.arcsin(des_height/dist_from_goal)
print 'avg pitch', avg_pitch

N = 200
T = 0.1
base_lin_traj = np.linspace(start_point , goal_point, num=N)

start_orient = [ 0.0, 0.0, 0.0]
mid_orient = [0.0, -avg_pitch, 0.0]
goal_orient = start_orient
base_orient_traj = np.vstack([np.linspace(start_orient , mid_orient, num=N/2), np.linspace(mid_orient , goal_orient, num=N/2)])
current_footholds = copy(params.getContactsPosWF())
step_lenght = [0.1, 0.0, 0.0]
in_stance = [True, True, True, True]

phase_offsets = np.array([0.0, 0.5, 0.5, 0.])
duty_factor = 0.75
swing_duration = 0.4
gait_duration = swing_duration/(1.0 - duty_factor)
tot_time = N*T
avg_speed = dist_from_goal/tot_time
print 'gait duration', gait_duration
print 'avg speed', avg_speed
step_lenght = avg_speed*gait_duration
print 'step length', step_lenght
step_idx = [0, 0, 0, 0]
next_liftoff_time = phase_offsets*gait_duration
next_touchdown_time = next_liftoff_time+swing_duration

for i in range(0, N):
    t = i*T
    print 'iter',i, 'time', t
    com_des = np.array([base_lin_traj[i, 0], base_lin_traj[i, 1], base_lin_traj[i, 2]])
    for leg in range(0, 4):
        if(t > next_liftoff_time[leg]):
            current_footholds[leg][0] += step_lenght
            step_idx[leg] += 1
            next_touchdown_time[leg] = next_liftoff_time[leg] + swing_duration
            next_liftoff_time[leg] += gait_duration
            in_stance[leg] = False

            if(current_footholds[leg][0] >= step_distance):
                current_footholds[leg][2] = step_height

        if (t >= next_touchdown_time[leg]):
            in_stance[leg] = True

    # print 'stance legs number:',np.sum(np.array(in_stance))
    # print 'step times', next_liftoff_time
    # print 'touch down times', next_touchdown_time
    # print 'step_idx', step_idx
    # print 'current footholds', current_footholds
    # print 'in stance', in_stance

    ''' Set current parameters'''
    params.setCoMPosWF(com_des)
    print 'current CoM pos', com_des

    euler_angles = np.array([base_orient_traj[i, 0], base_orient_traj[i, 1], base_orient_traj[i, 2]])
    params.setEulerAngles(euler_angles)
    print 'current base orient', euler_angles

    LF_foot = current_footholds[0]
    RF_foot = current_footholds[1]
    LH_foot = current_footholds[2]
    RH_foot = current_footholds[3]

    contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
    print 'current contacts wrt WF', contactsWF
    params.setContactsPosWF(contactsWF)
    ''' compute iterative projection
    Outputs of "iterative_projection_bretl" are:
    IP_points = resulting 2D vertices
    actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
    computation_time = how long it took to compute the iterative projection
    '''
    IP_points, force_polytopes, IP_computation_time, joints_pos, knee_pos, hips_pos = comp_dyn.try_iterative_projection_bretl(params)

    comp_geom = ComputationalGeometry()
    facets = comp_geom.compute_halfspaces_convex_hull(IP_points)
    point2check = comp_dyn.getReferencePoint(params, "ZMP")
    isPointFeasible, margin = comp_geom.isPointRedundant(facets, point2check)
    print "isPointFeasible: ", isPointFeasible
    print "Margin is: ", margin

    ''' compute Instantaneous Capture Point (ICP) and check if it belongs to the feasible region '''
    if params.useInstantaneousCapturePoint:
        ICP = InstantaneousCapturePoint()
        icp = ICP.compute(params)
        params.instantaneousCapturePoint = icp
        lpCheck = LpVertexRedundnacy()
        isIcpInsideFeasibleRegion, lambdas = lpCheck.isPointRedundant(IP_points.T, icp)

# '''Plotting the contact points in the 3D figure'''
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlabel('X axis')
# ax.set_ylabel('Y axis')
# ax.set_zlabel('Z axis')
# comWF = params.getCoMPosWF()
# ax.set_xlim(comWF[0]-0.5,comWF[0]+0.5)
# ax.set_ylim(comWF[1]-0.5,comWF[1]+0.5)
# ax.set_zlim(comWF[2]-0.8,comWF[2]+0.2)
#
# stanceFeet = params.getStanceFeet()
# nc = np.sum(stanceFeet)
# stanceID = params.getStanceIndex(stanceFeet)
# force_scaling_factor = 1500
#
# fz_tot = 0.0
# shoulder_position_WF = np.zeros((4,3))
# contactsWF = params.getContactsPosWF()
# normals = params.getNormals()
# contactsBF = params.getContactsPosBF()
# rpy_base = params.getOrientation()
# rot = Rot.from_euler('xyz', [rpy_base[0], rpy_base[1], rpy_base[2]], degrees=False)
# W_R_B = rot.as_dcm()
# for j in range(0,nc): # this will only show the contact positions and normals of the feet that are defined to be in stance
#     idx = int(stanceID[j])
#     ax.scatter(contactsWF[idx,0], contactsWF[idx,1], contactsWF[idx,2],c='b',s=100)
#     '''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region'''
#     if isPointFeasible:
#         ax.scatter(comWF[0], comWF[1], comWF[2],c='g',s=100)
#
#     else:
#         ax.scatter(comWF[0], comWF[1], comWF[2],c='r',s=100)
#
#     ''' draw 3D arrows corresponding to contact normals'''
#     a = Arrow3D([contactsWF[idx,0], contactsWF[idx,0]+normals[idx,0]/10], [contactsWF[idx,1], contactsWF[idx,1]+normals[idx,1]/10],[contactsWF[idx,2], contactsWF[idx,2]+normals[idx,2]/10], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
#
#     ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
#     shoulder_position_BF = [float(hips_pos[3*idx]),float(hips_pos[3*idx+1]),float(hips_pos[3*idx+2])]
#     rpy = params.getOrientation()
#     shoulder_position_WF[j,:] = W_R_B.dot(shoulder_position_BF) + comWF
#     ax.scatter(shoulder_position_WF[j,0], shoulder_position_WF[j,1], shoulder_position_WF[j,2], c='k', s=100)
#     ax.add_artist(a)
#
# print 'sum of vertical forces is', fz_tot
#
# ''' plotting Iterative Projection points '''
# plotter = Plotter()
# for j in range(0,nc): # this will only show the force polytopes of the feet that are defined to be in stance
#     idx = int(stanceID[j])
#     plotter.plot_polygon(np.transpose(IP_points))
#     if (params.getConstraintModes()[idx] == 'ONLY_ACTUATION') or (params.getConstraintModes()[idx] == 'FRICTION_AND_ACTUATION'):
#         plotter.plot_actuation_polygon(ax, force_polytopes.getVertices()[idx], contactsWF[idx,:], force_scaling_factor)
#
# for j in range(0, nc):
#     idx = int(stanceID[j])
#     knee_BF = [float(knee_pos[3*idx]), float(knee_pos[3*idx+1]), float(knee_pos[3*idx+2])]
#     knee_pos_WF = W_R_B.dot(knee_BF) + comWF
#     legs = np.vstack([shoulder_position_WF[j,:], knee_pos_WF, contactsWF[idx,:]])
#     ax.plot(legs[:,0],legs[:,1],legs[:,2], '-k')
#
# for j in range(0, 4):
#     shoulder_position_BF = [float(hips_pos[3 * j]), float(hips_pos[3 * j + 1]), float(hips_pos[3 * j + 2])]
#     rpy = params.getOrientation()
#     shoulder_position_WF[j, :] = W_R_B.dot(shoulder_position_BF) + comWF
# tmp = copy(shoulder_position_WF[2, :])
# shoulder_position_WF[2, :] = shoulder_position_WF[3, :]
# shoulder_position_WF[3, :] = tmp
# base_polygon = np.vstack([shoulder_position_WF, shoulder_position_WF[0, :]])
# ax.plot(base_polygon[:, 0], base_polygon[:, 1], base_polygon[:, 2], '--k')
#
# ''' 2D figure '''
# fig = plt.figure()
# for j in range(0,nc): # this will only show the contact positions and normals of the feet that are defined to be in stance
#     idx = int(stanceID[j])
#     ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
#     shoulder_position_BF = [contactsBF[idx,0],contactsBF[idx,1],0.4]
#     shoulder_position_WF[j,:] = np.dot(W_R_B,shoulder_position_BF) + comWF
#     idx = int(stanceID[j])
#     h1 = plt.plot(contactsWF[idx,0],contactsWF[idx,1],'ko',markersize=15, label='stance feet')
#
# h2 = plotter.plot_polygon(np.transpose(IP_points), '--b', 5, 'Feasible Region')
#
# '''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region)'''
# if isPointFeasible:
#     plt.plot(point2check[0],point2check[1],'go',markersize=15, label='ground reference point')
# else:
#     plt.plot(point2check[0],point2check[1],'ro',markersize=15, label='ground reference point')
#
#
# plt.grid()
# plt.xlabel("X [m]")
# plt.ylabel("Y [m]")
# plt.legend()
# plt.show()
