# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from numpy import array
from jet_leg.plotting.plotting_tools import Plotter
import random
from jet_leg.maths.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.dynamics.vertex_based_projection import VertexBasedProjection
from jet_leg.maths.iterative_projection_parameters import IterativeProjectionParameters

import matplotlib.pyplot as plt
from jet_leg.plotting.arrow3D import Arrow3D
        
plt.close('all')
math = Math()
class FeasibleWrenchPolytope():
    def __init__(self):
        self.vProj = VertexBasedProjection()

    def computeAngularPart(self, forcePolygonsVertices):
        numberOfForcePolygons = np.size(forcePolygonsVertices, 0)
        contactsBF = params.computeContactsBF().T
        wrenchPolytopes = []
        for i in np.arange(0, numberOfForcePolygons):
            footPos = contactsBF[:,i]
            currentPolytope = forcePolygonsVertices[i]
            angularPart = np.zeros((3,8))
            for j in np.arange(0,8):
                linear = currentPolytope[:,j]
                print "linear", linear
                print "foot pos", footPos
                angularPart[:,j] = np.cross(footPos,linear)
                print "angular", angularPart
            sixDpoly = np.vstack([currentPolytope, angularPart])
            wrenchPolytopes.append(sixDpoly)

        return wrenchPolytopes

    def compute_feasible_wrench_polytope_v_rep(self, params, forcePolygonsVertices):

        print "contacts BF",params.computeContactsBF()
        wrenchPolytopes = self.computeAngularPart(forcePolygonsVertices)
        numberOfForcePolygons = np.size(wrenchPolytopes, 0)
        print "number of force polytopes", numberOfForcePolygons
        tmpSum = wrenchPolytopes[0]
        print "dimensionality of the polytopes:", np.size(tmpSum[:,0])
        i = 0
        for j in np.arange(0,numberOfForcePolygons-1):
            nextPolygon = wrenchPolytopes[j+1]
            tmpSum = self.vProj.minksum(tmpSum, nextPolygon)
            print "tmpsm", np.shape(tmpSum)

        currentPolygonSum = self.vProj.convex_hull(tmpSum)
        print "tmpsm", np.shape(currentPolygonSum)
        return  currentPolygonSum

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
constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION']

# number of decision variables of the problem
#n = nc*6
comWF = np.array([.0, .0, .0])

""" contact points in the World Frame"""
LF_foot = np.array([0.3, 0.2, -0.4])
RF_foot = np.array([0.3, -0.2, -0.4])
LH_foot = np.array([-0.3, 0.2, -0.4])
RH_foot = np.array([-0.3, -0.2, -0.4])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

''' parameters to be tuned'''
mu = 0.5

''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1,1,1,1]

randomSwingLeg = random.randint(0,3)
tripleStance = False # if you want you can define a swing leg using this variable
if tripleStance:
    print 'Swing leg', randomSwingLeg
    stanceFeet[randomSwingLeg] = 0
print 'stanceLegs ' ,stanceFeet

''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
axisZ= array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # LF
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # RF
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # LH
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))  # RH
normals = np.vstack([n1, n2, n3, n4])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForceW = np.array([0.0, 0.0, 0.0]) # units are Nm

comp_dyn = ComputationalDynamics(robot_name)

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()

params.setContactsPosWF(contactsWF)
params.setCoMPosWF(comWF)
params.setTorqueLims(comp_dyn.robotModel.robotModel.torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(comp_dyn.robotModel.robotModel.trunkMass)
params.externalForceWF = extForceW  # params.externalForceWF is actually used anywhere at the moment

'''I now check whether the given CoM configuration is stable or not'''
C, d, isIKoutOfWorkSpace, forcePolytopes = comp_dyn.constr.getInequalities(params)

print isIKoutOfWorkSpace
print 'vertices', forcePolytopes

fwp = FeasibleWrenchPolytope()
fwp.compute_feasible_wrench_polytope_v_rep(params, forcePolytopes)


'''Plotting the contact points in the 3D figure'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)
force_scaling_factor = 1500
#plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15)
fz_tot = 0.0
for j in range(0,nc): # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ax.scatter(contactsWF[idx,0], contactsWF[idx,1], contactsWF[idx,2],c='b',s=100)
    '''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region'''
    if isConfigurationStable:
        ax.scatter(comWF[0], comWF[1], comWF[2],c='g',s=100)
        grf = contactForces[j*3:j*3+3]
        fz_tot += grf[2]

        ''' draw the set contact forces that respects the constraints'''
        b = Arrow3D([contactsWF[idx, 0], contactsWF[idx, 0] + grf[0] / force_scaling_factor],
                    [contactsWF[idx, 1], contactsWF[idx, 1] + grf[1] / force_scaling_factor],
                    [contactsWF[idx, 2], contactsWF[idx, 2] + grf[2] / force_scaling_factor], mutation_scale=20, lw=3,
                    arrowstyle="-|>",
                    color="b")
        ax.add_artist(b)
    else:
        ax.scatter(comWF[0], comWF[1], comWF[2],c='r',s=100)

    ''' draw 3D arrows corresponding to contact normals'''
    a = Arrow3D([contactsWF[idx,0], contactsWF[idx,0]+normals[idx,0]/10], [contactsWF[idx,1], contactsWF[idx,1]+normals[idx,1]/10],[contactsWF[idx,2], contactsWF[idx,2]+normals[idx,2]/10], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")

    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    ax.scatter(contactsWF[idx, 0], contactsWF[idx, 1], 0.0, c='k', s=100)
    ax.add_artist(a)

print 'sum of vertical forces is', fz_tot

''' plotting Iterative Projection points '''
plotter = Plotter()
for j in range(0,nc): # this will only show the force polytopes of the feet that are defined to be in stance
    idx = int(stanceID[j])
    plotter.plot_polygon(np.transpose(IP_points))
    if (constraint_mode_IP[idx] == 'ONLY_ACTUATION') or (constraint_mode_IP[idx] == 'FRICTION_AND_ACTUATION'):
        plotter.plot_actuation_polygon(ax, forcePolytopes[idx], contactsWF[idx,:], force_scaling_factor)

''' 2D figure '''
plt.figure()
for j in range(0,nc): # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    h1 = plt.plot(contactsWF[idx,0],contactsWF[idx,1],'ko',markersize=15, label='stance feet')
h2 = plotter.plot_polygon(np.transpose(IP_points), '--b','Support Region')

'''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region)'''
if isConfigurationStable:
    plt.plot(comWF[0],comWF[1],'go',markersize=15, label='CoM')
else:
    plt.plot(comWF[0],comWF[1],'ro',markersize=15, label='CoM')
    
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.show()