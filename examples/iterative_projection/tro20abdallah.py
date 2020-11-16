# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np
import copy
from numpy import array
from scipy.spatial.transform import Rotation as Rot
from jet_leg.plotting.plotting_tools import Plotter
import random
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.dynamics.instantaneous_capture_point import InstantaneousCapturePoint
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.optimization.lp_vertex_redundancy import LpVertexRedundnacy

import matplotlib.pyplot as plt
from jet_leg.plotting.arrow3D import Arrow3D
        
plt.close('all')
math = Math()

''' Set the robot's name (either 'hyq', 'hyqreal' or 'anymal')'''
robot_name = 'anymal_boxy'

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
#n = nc*6
comWF = np.array([.0, 0.0, 1.0])
comWF_lin_acc = np.array([2.5, -.35, .0])
comWF_ang_acc = np.array([.0, .0, .0])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForce = np.array([0.0, 0.0, 0.0*9.81]) # units are N
extCentroidalTorque = np.array([.0, .0, .0]) # units are Nm
extCentroidalWrench = np.hstack([extForce, extCentroidalTorque])

''' Roll Pitch Yaw angles of the base link'''
rpy_base = np.array([0.0, 0.0, 0.0]) # units are rads
rot = Rot.from_euler('xyz', [rpy_base[0], rpy_base[1], rpy_base[2]], degrees=False)
W_R_B = rot.as_dcm()

""" contact points in the World Frame"""
LF_foot = np.array([0.36, 0.16, -0.55])
RF_foot = np.array([0.4, -0.2, -0.55])
LH_foot = np.array([-0.4, 0.2, -0.55])
RH_foot = np.array([-0.44, -0.24, -0.55])
#LF_foot = np.array([0.270093,  0.163428, -0.479816])
#RF_foot = np.array([0.270093,  0.163428, -0.479816])
#LH_foot = np.array([0.36,      0.21,     -0.47])
#RH_foot = np.array([-0.254592, -0.162951, -0.325419])

contactsBF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
contactsWF = copy.copy(contactsBF);
for j in np.arange(0,4):
    contactsWF[j,:] = np.add(np.dot(W_R_B,copy.copy(contactsBF[j, :])), comWF)

''' parameters to be tuned'''
mu = 0.8

''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1,0,0,1]

randomSwingLeg = random.randint(0,3)
tripleStance = False # if you want you can define a swing leg using this variable
if tripleStance:
    print 'Swing leg', randomSwingLeg
    stanceFeet[randomSwingLeg] = 0
print 'stanceLegs ' ,stanceFeet

''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
axisZ= array([[0.0], [0.0], [1.0]])
n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # LF
r, p, y = math.normalToRpy(n1) # this can be used for training the RL
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # RF
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # LH
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # RH
normals = np.vstack([n1, n2, n3, n4])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForceW = np.array([0.0, 0.0, 0.0]) # units are Nm

comp_dyn = ComputationalDynamics(robot_name)

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters(robot_name)

params.setEulerAngles(rpy_base)
params.setContactsPosWF(contactsWF)
params.externalCentroidalWrench = extCentroidalWrench
params.setCoMPosWF(comWF)
params.comLinVel = [0.0, 0.0, 0.0]
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

#print "Inequalities", comp_dyn.ineq
#print "actuation polygons"
#print actuation_polygons

'''I now check whether the given CoM configuration is stable or not'''
#isCoMStable, contactForces, forcePolytopes = comp_dyn.check_equilibrium(params)
#print "is CoM stable?", isCoMStable
#print 'Contact forces:', contactForces

comp_geom = ComputationalGeometry()
facets = comp_geom.compute_halfspaces_convex_hull(IP_points)
point2check = comp_dyn.getReferencePoint(params, "COM")
zmp = comp_dyn.getReferencePoint(params, "ZMP")
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

'''Plotting the contact points in the 3D figure'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_xlim(comWF[0]-0.5,comWF[0]+0.5)
ax.set_ylim(comWF[1]-0.5,comWF[1]+0.5)
ax.set_zlim(comWF[2]-0.8,comWF[2]+0.2)

nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)
force_scaling_factor = 1500
#plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)
fz_tot = 0.0
shoulder_position_WF = np.zeros((4,3))

for j in range(0,nc): # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ax.scatter(contactsWF[idx,0], contactsWF[idx,1], contactsWF[idx,2],c='b',s=100)
    '''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region'''
    if isPointFeasible:
        ax.scatter(comWF[0], comWF[1], comWF[2],c='g',s=100)

    else:
        ax.scatter(comWF[0], comWF[1], comWF[2],c='r',s=100)

    ''' draw 3D arrows corresponding to contact normals'''
    a = Arrow3D([contactsWF[idx,0], contactsWF[idx,0]+normals[idx,0]/10], [contactsWF[idx,1], contactsWF[idx,1]+normals[idx,1]/10],[contactsWF[idx,2], contactsWF[idx,2]+normals[idx,2]/10], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")

    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    shoulder_position_BF = [contactsBF[idx,0],contactsBF[idx,1],0.0]
    rpy = params.getOrientation()
    shoulder_position_WF[j,:] = W_R_B.dot(shoulder_position_BF) + comWF
    ax.scatter(shoulder_position_WF[j,0], shoulder_position_WF[j,1], shoulder_position_WF[j,2], c='k', s=100)
    ax.add_artist(a)

print 'sum of vertical forces is', fz_tot

''' plotting Iterative Projection points '''
plotter = Plotter()
for j in range(0,nc): # this will only show the force polytopes of the feet that are defined to be in stance
    idx = int(stanceID[j])
    plotter.plot_polygon(np.transpose(IP_points))
    if (constraint_mode_IP[idx] == 'ONLY_ACTUATION') or (constraint_mode_IP[idx] == 'FRICTION_AND_ACTUATION'):
        plotter.plot_actuation_polygon(ax, force_polytopes.getVertices()[idx], contactsWF[idx,:], force_scaling_factor)

base_polygon = np.vstack([shoulder_position_WF, shoulder_position_WF[0,:]])
ax.plot(base_polygon[:,0],base_polygon[:,1],base_polygon[:,2], '--k')

''' 2D figure '''
fig = plt.figure()
plt.grid()
for j in range(0,nc): # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    shoulder_position_BF = [contactsBF[idx,0],contactsBF[idx,1],0.4]
    shoulder_position_WF[j,:] = np.dot(W_R_B,shoulder_position_BF) + comWF


plt.scatter(shoulder_position_WF[0,0],shoulder_position_WF[0,1], s=250, c = 'k')
plt.scatter(shoulder_position_WF[1,0],shoulder_position_WF[1,1], s=250, c = 'k')
plt.plot(shoulder_position_WF[0,0],shoulder_position_WF[0,1],'ko',markersize=15, label='stance feet')
h2 = plotter.plot_polygon(np.transpose(IP_points), '--b', 5,'Improved Feasible Region')
plt.plot([shoulder_position_WF[0,0], shoulder_position_WF[1,0]],[shoulder_position_WF[0,1], shoulder_position_WF[1,1]],'k--',markersize=15,
         alpha = 0.5, label='convex hull of contact points')
acc_scaling_factor = 10.0
Width = 0.01

'''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region)'''
if isPointFeasible:
    plt.plot(point2check[0],point2check[1],'go',markersize=15,  markeredgecolor = 'k', label='CoM projection', zorder=10)
    Color = 'g'
else:
    plt.plot(point2check[0],point2check[1],'ro',markersize=15,  markeredgecolor = 'k', label='CoM projection', zorder=10)
    Color = 'r'

plt.plot(zmp[0], zmp[1],'go',markersize=15,  markeredgecolor = 'k', label='ZMP', zorder=10)

plt.arrow(point2check[0],point2check[1], point2check[0] + comWF_lin_acc[0]/acc_scaling_factor, point2check[1] + comWF_lin_acc[1]/acc_scaling_factor,
          width = Width, head_width = 2*Width, edgecolor = 'k', facecolor = Color, zorder=9)

plt.annotate("CoM acceleration",
            xy=(point2check[0] + comWF_lin_acc[0]/acc_scaling_factor - 0.1, point2check[1] + comWF_lin_acc[1]/acc_scaling_factor - 0.01), xycoords='data',
            xytext=(point2check[0] + comWF_lin_acc[0]/acc_scaling_factor - 0.1, point2check[1] + comWF_lin_acc[1]/acc_scaling_factor - 0.01),
            arrowprops=dict(facecolor='black', shrink=1.5),
            horizontalalignment='left',
            verticalalignment='top',
            fontsize = 12,
            )

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend(prop={'size': 12})
fig.savefig('../../figs/double_stance_region.pdf')
plt.show()

