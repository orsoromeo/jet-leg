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
from jet_leg.optimization.lp_vertex_redundancy import LpVertexRedundnacy
from jet_leg.maths.iterative_projection_parameters import IterativeProjectionParameters

import matplotlib.pyplot as plt
from jet_leg.plotting.arrow3D import Arrow3D
        
plt.close('all')
math = Math()
class FeasibleWrenchPolytope():
    def __init__(self):
        self.vProj = VertexBasedProjection()

    def checkDynamicStability(self, FWP, w_gi):
        lp = LpVertexRedundnacy()
        isPointRedundant, lambdas = lp.isPointRedundant(FWP, w_gi)
        #print "is point redundant? ", isPointRedundant, lambdas
        if isPointRedundant:
            isStateStable = True
        else:
            isStateStable = False
        return isStateStable

    def compute_aggregated_centroidal_wrench(self, fwp_params):
        linear = np.array([0.0, 0.0, fwp_params.getTotalMass()*9.81])
        com_pos_WF = fwp_params.getCoMPosWF()
        angular = np.cross(com_pos_WF, linear)
        w_gi = np.hstack([linear, angular])
        return w_gi

    def computeAngularPart(self, fwp_params, forcePolygonsVertices):
        numberOfForcePolygons = np.size(forcePolygonsVertices, 0)
        contactsBF = fwp_params.computeContactsBF().T
        wrenchPolytopes = []
        stanceLegs = fwp_params.getStanceFeet()
        stanceIndex = fwp_params.getStanceIndex(stanceLegs)
        contactsNumber = np.sum(stanceFeet)
        for i in range(0, contactsNumber):
            #r = contactsWF[int(stanceIndex[j]), :]
            footPos = contactsBF[:,int(stanceIndex[i])]
            currentPolytope = forcePolygonsVertices[i]
            angularPart = np.zeros((3,8))
            for j in np.arange(0,8):
                linear = currentPolytope[:,j]
                angularPart[:,j] = np.cross(footPos,linear)

            sixDpoly = np.vstack([currentPolytope, angularPart])
            wrenchPolytopes.append(sixDpoly)

        return wrenchPolytopes

    def compute_feasible_wrench_polytope_v_rep(self, fwp_params, forcePolygonsVertices):

        wrenchPolytopes = self.computeAngularPart(fwp_params, forcePolygonsVertices)
        numberOfForcePolygons = np.size(wrenchPolytopes, 0)
        tmpSum = wrenchPolytopes[0]
        i = 0
        for j in np.arange(0,numberOfForcePolygons-1):
            nextPolygon = wrenchPolytopes[j+1]
            tmpSum = self.vProj.minksum(tmpSum, nextPolygon)

        currentPolygonSum = self.vProj.convex_hull(tmpSum)
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
comWF = np.array([.31, .0, .0])

""" contact points in the World Frame"""
LF_foot = np.array([0.3, 0.2, -0.4])
RF_foot = np.array([0.3, -0.2, -0.4])
LH_foot = np.array([-0.3, 0.2, -0.4])
RH_foot = np.array([-0.3, -0.2, -0.4])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

''' parameters to be tuned'''
mu = 0.5

''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1,1,0,1]

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

fwp = FeasibleWrenchPolytope()
FWP = fwp.compute_feasible_wrench_polytope_v_rep(params, forcePolytopes)
w_gi = fwp.compute_aggregated_centroidal_wrench(params)
isFWPStable = fwp.checkDynamicStability(FWP, w_gi)

'''I now check whether the given CoM configuration is stable or not'''
isStaticallyStable, contactForces, forcePolytopes = comp_dyn.check_equilibrium(params)



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
    if isFWPStable:
        ax.scatter(comWF[0], comWF[1], comWF[2],c='g',s=100)
    else:
        ax.scatter(comWF[0], comWF[1], comWF[2],c='r',s=100)

    if isStaticallyStable:
        ax.scatter(comWF[0], comWF[1], comWF[2], c='g', s=100, marker='^')
    else:
        ax.scatter(comWF[0], comWF[1], comWF[2], c='r', s=100, marker='^')

    ''' draw 3D arrows corresponding to contact normals'''
    a = Arrow3D([contactsWF[idx,0], contactsWF[idx,0]+normals[idx,0]/10], [contactsWF[idx,1], contactsWF[idx,1]+normals[idx,1]/10],[contactsWF[idx,2], contactsWF[idx,2]+normals[idx,2]/10], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")

    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    ax.scatter(contactsWF[idx, 0], contactsWF[idx, 1], 0.0, c='k', s=100)
    ax.add_artist(a)


''' 2D figure '''
plotter = Plotter()
plt.figure()
for j in range(0,nc): # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    h1 = plt.plot(contactsWF[idx,0],contactsWF[idx,1],'ko',markersize=15, label='stance feet')


'''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region)'''
if isFWPStable:
    plt.plot(comWF[0],comWF[1],'go',markersize=15, label='CoM (dynamic check)')
else:
    plt.plot(comWF[0],comWF[1],'ro',markersize=15, label='CoM (dynamic check)')

if isStaticallyStable:
    plt.plot(comWF[0],comWF[1],'g',markersize=20, marker= '^', label='CoM (static check)')
else:
    plt.plot(comWF[0], comWF[1], 'r', markersize=20, marker='^', label='CoM (static check)')
    
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.show()