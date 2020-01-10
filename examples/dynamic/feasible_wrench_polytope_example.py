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
from jet_leg.dynamics.feasible_wrench_polytope import FeasibleWrenchPolytope
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
import time
import matplotlib.pyplot as plt


plt.close('all')
math = Math()

''' Set the robot's name (either 'hyq', 'hyqreal' or 'anymal')'''
robot_name = 'hyq'

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
comWF = np.array([0., 0., 0.0])
comWF_lin_acc = np.array([1.0, 1.0, .0])
comWF_ang_acc = np.array([.0, .0, .0])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForce = np.array([0., .0, .0]) # units are N
extCentroidalTorque = np.array([.0, .0, .0]) # units are Nm
extCentroidalWrench = np.hstack([extForce, extCentroidalTorque])

""" contact points in the World Frame"""
LF_foot = np.array([0.3, 0.2, -0.4])
RF_foot = np.array([0.3, -0.2, -0.4])
LH_foot = np.array([-0.3, 0.15, -0.4])
RH_foot = np.array([-0.3, -0.2, -0.4])

contactsWF = np.vstack((LF_foot+comWF, RF_foot+comWF, LH_foot+comWF, RH_foot+comWF))

''' parameters to be tuned'''
mu = 0.5

''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [0,1,1,1]

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

comp_dyn = ComputationalDynamics(robot_name)

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters()

params.setContactsPosWF(contactsWF)
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

'''I now check whether the given CoM configuration is stable or not'''
C, d, isIKoutOfWorkSpace, forcePolytopes = comp_dyn.constr.getInequalities(params)

rbd = RigidBodyDynamics()

if not isIKoutOfWorkSpace:
    fwp = FeasibleWrenchPolytope(params)
    FWP = fwp.computeFeasibleWrenchPolytopeVRep(params, forcePolytopes)
    w_gi = rbd.computeCentroidalWrench(params.getTotalMass(), comWF, params.externalCentroidalWrench, comWF_lin_acc)
    '''I now check whether the given CoM configuration is dynamically stable or not (see "Feasible Wrench Polytope")'''
    start = time.time()
    isFWPStable = fwp.checkDynamicStability(FWP, w_gi)
    print "dynamic stability check time", time.time() - start

    '''I now check whether the given CoM configuration is statically stable or not (see "Feasible Region")'''
    start = time.time()
    isStaticallyStable, contactForces, forcePolytopes = comp_dyn.check_equilibrium(params)
    print "static stability check time", time.time() - start
else:
    isFWPStable = False
    isStaticallyStable = False

'''Plotting the contact points in the 3D figure'''
nc = np.sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)
force_scaling_factor = 1500

''' 2D figure '''
plotter = Plotter()
plt.figure()
for j in range(0,nc): # this will only show the contact positions and normals of the feet that are defined to be in stance
    idx = int(stanceID[j])
    ''' The black spheres represent the projection of the contact points on the same plane of the feasible region'''
    h1 = plt.plot(contactsWF[idx,0],contactsWF[idx,1],'ko',markersize=15, label='stance feet')


'''CoM will be plotted in green if it is stable (i.e., if it is inside the feasible region)'''
inertialForce = comWF_lin_acc*params.getTotalMass()/force_scaling_factor
extForce = extForce/100
if isFWPStable:
    plt.plot(comWF[0],comWF[1],'go',markersize=15, label='CoM (dynamic check)')
else:
    plt.plot(comWF[0],comWF[1],'ro',markersize=15, label='CoM (dynamic check)')

plt.arrow(comWF[0], comWF[1], inertialForce[0], inertialForce[1], head_width=0.01, head_length=0.01, fc='k',
              ec='orange', label='inertial acceleration')
plt.arrow(comWF[0] + inertialForce[0], comWF[1] + inertialForce[1], extForce[0]/force_scaling_factor, extForce[1]/force_scaling_factor, head_width=0.01,
              head_length=0.01, fc='blue', ec='blue', label='external force')

plt.scatter([comWF[0], comWF[0]+inertialForce[0]], [comWF[1], comWF[1]+inertialForce[1]], color='k', marker= '^', label='inertial acceleration')
plt.scatter([comWF[0], comWF[0]+inertialForce[0]], [comWF[1], comWF[1]+inertialForce[1]], color='b', marker= '^', label='external force')

if isStaticallyStable:
    plt.plot(comWF[0],comWF[1],'g',markersize=20, marker= '^', label='CoM (static check)')
else:
    plt.plot(comWF[0], comWF[1], 'r', markersize=20, marker='^', label='CoM (static check)')
    
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.show()