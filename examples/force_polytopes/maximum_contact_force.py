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
comWF = np.array([0.0, -0.0, 0.0])

''' Set the robot's name (either 'hyq', 'hyqreal' or 'anymal')'''
robot_name = 'anymal'
comp_dyn = ComputationalDynamics(robot_name)

stackedForcePolytopesLF = np.zeros((3,8))
stackedFootPosLF = []
for lf_x in np.arange(0.0, 0.7, 0.05):

    """ contact points in the World Frame"""
    LF_foot = np.array([lf_x, 0.2, -0.4])
    RF_foot = np.array([0.3, -0.2, -0.4])
    LH_foot = np.array([-0.3, 0.2, -0.4])
    RH_foot = np.array([-0.3, -0.2, -0.4])
    print LF_foot
    contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

    # contacts = contactsToStack[0:nc+1, :]
    # print contacts

    ''' parameters to be tuned'''
    g = 9.81
    trunk_mass = 45.
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

    ''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
    HAA = Hip Abduction Adduction
    HFE = Hip Flextion Extension
    KFE = Knee Flextion Extension
    '''
    LF_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
    RF_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
    LH_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
    RH_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
    torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])

    ''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
    extForceW = np.array([0.0, 0.0, 0.0])  # units are Nm


    '''You now need to fill the 'params' object with all the relevant 
        informations needed for the computation of the IP'''
    params = IterativeProjectionParameters()
    params.setContactsPosWF(contacts)
    params.setCoMPosWF(comWF)
    params.setTorqueLims(torque_limits)
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

    if isIKoutOfWorkSpace:
        break
    else:
        forcePolytopeLF = forcePolytopes[0]
        print("force polytope LS", forcePolytopeLF)
        stackedForcePolytopesLF = np.hstack([stackedForcePolytopesLF, forcePolytopeLF])
        stackedFootPosLF = np.hstack([stackedFootPosLF, lf_x])

''' 2D figure '''
number_of_samples = np.shape(stackedForcePolytopesLF)[1]/8 -1
print number_of_samples
plt.figure()

amplitude = []
for j in np.arange(0,number_of_samples):
    tmpVX = stackedForcePolytopesLF[0]
    tmpVY = stackedForcePolytopesLF[1]
    tmpVZ = stackedForcePolytopesLF[2]
    vx = []
    vy = []
    vz = []
    amp = 0.0
    for i in np.arange(0,8):
        new_vx = tmpVX[8 + j * 8 + i]
        vx = np.hstack([vx, new_vx])
        vy = np.hstack([vy, tmpVY[8 + j * 8 + i]])
        vz = np.hstack([vz, tmpVZ[8 + j * 8 + i]])
        tmp_amp = np.sqrt(np.power(vx[-1],2) + np.power(vy[-1],2) + np.power(vz[-1],2))
        print tmp_amp
        if tmp_amp>amp:
            amp = tmp_amp
    amplitude = np.hstack([amplitude, amp])
print "amplitude", amplitude

plt.plot(stackedFootPosLF, amplitude, '-o', markersize=15, label='vertices')

tmpVX = stackedForcePolytopesLF[0]
tmpVY = stackedForcePolytopesLF[1]
tmpVZ = stackedForcePolytopesLF[2]
for i in np.arange(0,8):
    vx = []
    vy = []
    vz = []
    for j in np.arange(0,number_of_samples):
        print np.shape(tmpVX)
        print j
        vx = np.hstack([vx, tmpVX[8 + j * 8 + i]])
        vy = np.hstack([vy, tmpVY[8 + j * 8 + i]])
        vz = np.hstack([vz, tmpVZ[8 + j * 8 + i]])
        amp = np.sqrt(np.power(vx,2) + np.power(vy,2) + np.power(vz,2))
        amplitude = np.hstack([amplitude, amp])
     #plt.plot(stackedFootPosLF, vz, '-o', markersize=15, label='vertices')

plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend()
plt.show()

