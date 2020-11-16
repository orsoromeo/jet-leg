"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from numpy import array
from copy import deepcopy
import random
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.optimization.jacobians import Jacobians
from jet_leg.variables.com import CoM
import time

import matplotlib.pyplot as plt

plt.close('all')
math = Math()

''' Set the robot's name (either 'hyq', 'hyqreal', 'anymal_boxy' or 'anymal_coyote')'''
robot_name = 'anymal_coyote'

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

comWF = np.array([0.0, 0.0, 0.0])
comWF_lin_acc = np.array([.0, .0, .0])
comWF_ang_acc = np.array([.0, .0, .0])

''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
extForce = np.array([0., .0, 0.0 * 9.81])  # units are N
extCentroidalTorque = np.array([.0, .0, .0])  # units are Nm
extCentroidalWrench = np.hstack([extForce, extCentroidalTorque])

''' parameters to be tuned'''
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

comp_dyn = ComputationalDynamics(robot_name)

'''You now need to fill the 'params' object with all the relevant 
    informations needed for the computation of the IP'''
params = IterativeProjectionParameters(robot_name)
""" contact points in the World Frame"""
LF_foot = np.array([0.3, 0.2, -0.4])
RF_foot = np.array([0.3, -0.2, -0.4])
LH_foot = np.array([-0.3, 0.2, -0.4])
RH_foot = np.array([-0.3, -0.2, -0.4])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
params.setContactsPosWF(contactsWF)


start = time.time()

params.useContactTorque = True
params.useInstantaneousCapturePoint = True
params.externalCentroidalWrench = extCentroidalWrench
params.setCoMPosWF(comWF)
params.comLinVel = [0., 0.0, 0.0]
params.setCoMLinAcc(comWF_lin_acc)
params.setTorqueLims(comp_dyn.robotModel.robotModel.joint_torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setTotalMass(comp_dyn.robotModel.robotModel.trunkMass)
params.externalForceWF = extForceW  # params.externalForceWF is actually used anywhere at the moment

params_com_x = deepcopy(params)
params_com_y = deepcopy(params)
params_com_z = deepcopy(params)
params_com_vel_x = deepcopy(params)
params_com_vel_y = deepcopy(params)
params_com_vel_z = deepcopy(params)
params_com_acc_x = deepcopy(params)
params_com_acc_y = deepcopy(params)
params_com_acc_z = deepcopy(params)
params_base_roll = deepcopy(params)
params_base_pitch = deepcopy(params)

jac = Jacobians(robot_name)
comp_geom = ComputationalGeometry()
com = CoM(params)

delta_pos_range = 0.2
num_of_tests = 25
delta_pos_range_vec = np.linspace(-delta_pos_range/2.0, delta_pos_range/2.0, num_of_tests)
print "number of tests", num_of_tests

pos_margin_x, jac_com_pos_x = jac.plotMarginAndJacobianWrtComPosition(params_com_x, delta_pos_range_vec, 0) # dm / dx
pos_margin_y, jac_com_pos_y = jac.plotMarginAndJacobianWrtComPosition(params_com_y, delta_pos_range_vec, 1) # dm / dy
pos_margin_z, jac_com_pos_z = jac.plotMarginAndJacobianWrtComPosition(params_com_z, delta_pos_range_vec, 2) # dm / dz

delta_vel_range = 3.0
delta_vel_range_vec = np.linspace(-delta_vel_range/2.0, delta_vel_range/2.0, num_of_tests)
print num_of_tests, np.shape(delta_vel_range_vec)

""" contact points in the World Frame"""
LF_foot = np.array([0.3, 0.2, -0.4])
RF_foot = np.array([0.3, -0.2, -0.4])
LH_foot = np.array([-0.3, 0.2, -0.4])
RH_foot = np.array([-0.3, -0.2, -0.4])

contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
params.setContactsPosWF(contactsWF)

vel_margin_x, jac_com_lin_vel_x = jac.plotMarginAndJacobianOfMarginWrtComVelocity(params_com_vel_x, delta_vel_range_vec, 0) # dm / d x_d
vel_margin_y, jac_com_lin_vel_y = jac.plotMarginAndJacobianOfMarginWrtComVelocity(params_com_vel_y, delta_vel_range_vec, 1) # dm / d y_d
vel_margin_z, jac_com_lin_vel_z = jac.plotMarginAndJacobianOfMarginWrtComVelocity(params_com_vel_z, delta_vel_range_vec, 2) # dm / d z_d

delta_acc_range = 8.0
delta_acc_range_vec = np.linspace(-delta_acc_range/2.0, delta_acc_range/2.0, num_of_tests)
acc_margin_x, jac_com_lin_acc_x = jac.plotMarginAndJacobianOfMarginWrtComLinAcceleration(params_com_acc_x, delta_acc_range_vec, 0) # dm/d x_dd
acc_margin_y, jac_com_lin_acc_y = jac.plotMarginAndJacobianOfMarginWrtComLinAcceleration(params_com_acc_y, delta_acc_range_vec, 1) # dm/d y_dd
acc_margin_z, jac_com_lin_acc_z = jac.plotMarginAndJacobianOfMarginWrtComLinAcceleration(params_com_acc_z, delta_acc_range_vec, 2) # dm/d z_dd

delta_orientation_range = np.pi/4
delta_orientation_range_vec = np.linspace(-delta_orientation_range/2.0, delta_orientation_range/2.0, num_of_tests)
roll_margin, jac_base_roll = jac.plotMarginAndJacobianWrtBaseOrientation(params_base_roll, delta_orientation_range_vec, 0) # roll
pitch_margin, jac_base_pitch = jac.plotMarginAndJacobianWrtBaseOrientation(params_base_pitch, delta_orientation_range_vec, 1) # pitch


### Plotting

### X axis
fig1 = plt.figure(1)
fig1.suptitle("Analytic stability margin")
plt.subplot(231)
plt.plot(delta_pos_range_vec, pos_margin_x, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$c_x$ [m]")
plt.ylabel("m [m]")
plt.title("CoM X pos margin")

plt.subplot(234)
plt.plot(delta_pos_range_vec, jac_com_pos_x[0,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$c_x$ [m]")
plt.ylabel(" $ \delta m/  \delta c_x$")
plt.title("CoM X pos jacobian")

plt.subplot(232)
plt.plot(delta_vel_range_vec, pos_margin_y, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\dot{c}_x$ [m/s]")
plt.ylabel("m [m]")
plt.title("CoM X vel margin")

plt.subplot(235)
plt.plot(delta_vel_range_vec, jac_com_lin_vel_x[0,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\dot{c}_x$ [m/s]")
plt.ylabel("$ \delta m/  \delta \dot{c}_x$")
plt.title("CoM X vel jacobian")

plt.subplot(233)
plt.plot(delta_acc_range_vec, acc_margin_x, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\ddot{c}_x$ [m/s^2]")
plt.ylabel("m [m]")
plt.title("CoM X acc margin")

plt.subplot(236)
plt.plot(delta_acc_range_vec, jac_com_lin_acc_x[0,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\ddot{c}_x$ [m/s^2]")
plt.ylabel("$ \delta m/  \delta \ddot{c}_x$")
plt.title("CoM X acc jacobian")

### Y axis
fig2 = plt.figure(2)
fig2.suptitle("Analytic stability margin")
plt.subplot(231)
plt.plot(delta_pos_range_vec, pos_margin_y, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$c_y$ [m]")
plt.ylabel("m [m]")
plt.title("CoM Y pos margin")

plt.subplot(234)
plt.plot(delta_pos_range_vec, jac_com_pos_y[1,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$c_y$ [m]")
plt.ylabel(" $ \delta m/  \delta c_y$")
plt.title("CoM Y pos jacobian")

plt.subplot(232)
plt.plot(delta_vel_range_vec, vel_margin_y, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\dot{c}_y$ [m/s]")
plt.ylabel("m [m]")
plt.title("CoM Y vel margin")

plt.subplot(235)
plt.plot(delta_vel_range_vec, jac_com_lin_vel_y[1,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\dot{c}_y$ [m/s]")
plt.ylabel("$ \delta m/  \delta \dot{c}_y$")
plt.title("CoM Y vel jacobian")

plt.subplot(233)
plt.plot(delta_acc_range_vec, acc_margin_y, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\ddot{c}_y$ [m/s^2]")
plt.ylabel("m [m]")
plt.title("CoM Y acc margin")

plt.subplot(236)
plt.plot(delta_acc_range_vec, jac_com_lin_acc_y[1,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\ddot{c}_y$ [m/s^2]")
plt.ylabel("$ \delta m/  \delta \ddot{c}_y$")
plt.title("CoM Y acc jacobian")


### Z axis
fig3 = plt.figure(3)
fig3.suptitle("Analytic stability margin")
plt.subplot(231)
plt.plot(delta_pos_range_vec, pos_margin_z, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$c_z$ [m]")
plt.ylabel("m [m]")
plt.title("CoM Z pos margin")

plt.subplot(234)
plt.plot(delta_pos_range_vec, jac_com_pos_z[2,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$c_z$ [m]")
plt.ylabel(" $ \delta m/  \delta c_z$")
plt.title("CoM Z pos jacobian")

plt.subplot(232)
plt.plot(delta_vel_range_vec, vel_margin_z, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\dot{c}_z$ [m/s]")
plt.ylabel("m [m]")
plt.title("CoM Z vel margin")

plt.subplot(235)
plt.plot(delta_vel_range_vec, jac_com_lin_vel_z[2,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\dot{c}_z$ [m/s]")
plt.ylabel("$ \delta m/  \delta \dot{c}_z$")
plt.title("CoM Z vel jacobian")

plt.subplot(233)
plt.plot(delta_acc_range_vec, acc_margin_z, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\ddot{c}_z$ [m/s^2]")
plt.ylabel("m [m]")
plt.title("CoM Z acc margin")

sub = plt.subplot(236)
plt.plot(delta_acc_range_vec, jac_com_lin_acc_z[2,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$\ddot{c}_z$ [m/s^2]")
plt.ylabel("$ \delta m/  \delta \ddot{c}_z$")
#sub.set_xlim([-4, 4])
#sub.set_ylim([-0.5, 0.5])
plt.title("CoM Z acc jacobian")

### Orientation
fig4 = plt.figure(4)
fig4.suptitle("Analytic stability margin")
plt.subplot(221)
plt.plot(delta_orientation_range_vec, roll_margin, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("roll $\theta$ [rad]")
plt.ylabel("m [m]")
plt.title("Base roll margin")

plt.subplot(223)
plt.plot(delta_orientation_range_vec, jac_base_roll[2,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$roll $\theta$ [rad]")
plt.ylabel(" $ \delta m/  \delta \theta $")
plt.title("CoM Z pos jacobian")

plt.subplot(222)
plt.plot(delta_orientation_range_vec, pitch_margin, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("pitch [rad]")
plt.ylabel("m [m]")
plt.title("Base pitch margin")

plt.subplot(224)
plt.plot(delta_orientation_range_vec, jac_base_pitch[2,:], 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("")
plt.ylabel("")
plt.title("Basep pitch jacobian")

plt.show()