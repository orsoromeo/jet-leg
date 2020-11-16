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

delta_pos_range = 0.39
dx = 0.01
num_of_tests = delta_pos_range/dx
delta_pos_range_vec = np.linspace(-delta_pos_range/2.0, delta_pos_range/2.0, num_of_tests)
print "number of tests", num_of_tests

pos_margin_y, jac_com_pos_y = jac.plotMarginAndJacobianWrtComPosition(params_com_y, delta_pos_range_vec, 1) # dm / dyz

delta_vel_range = 3.0
delta_vel_range_vec = np.linspace(-delta_vel_range/2.0, delta_vel_range/2.0, num_of_tests)
print num_of_tests, np.shape(delta_vel_range_vec)

### Plotting

### X axis
fig1 = plt.figure(1)
fig1.suptitle("Analytic vs. Learned stability margin")

plt.subplot(411)
plt.plot(delta_pos_range_vec, pos_margin_y, 'r-o', markersize=2, label='analytic')

plt.subplot(412)
plt.plot(delta_pos_range_vec, jac_com_pos_y[0,:], 'r-o', markersize=2, label='analytic (finite-diff)')

plt.subplot(413)
plt.plot(delta_pos_range_vec, jac_com_pos_y[1,:], 'r-o', markersize=2, label='analytic (finite-diff)')

plt.subplot(4,1,4)
plt.plot(delta_pos_range_vec, jac_com_pos_y[2,:], 'r-o', markersize=2, label='analytic (finite-diff)')


iter, ee_offset, mx, my, mz, \
ee_jac_xx, ee_jac_xx_fin_diff, ee_jac_xy, ee_jac_xy_fin_diff, ee_jac_xz, ee_jac_xz_fin_diff, \
ee_jac_yx, ee_jac_yx_fin_diff, ee_jac_yy, ee_jac_yy_fin_diff, ee_jac_yz, ee_jac_yz_fin_diff, \
ee_jac_zx, ee_jac_zx_fin_diff, ee_jac_zy, ee_jac_zy_fin_diff, ee_jac_zz, ee_jac_zz_fin_diff \
= np.loadtxt('/home/rorsolino/catkin_ws/src/towr/towr/dependencies/jet_leg_learn_ported/build/devel/lib/jet_leg_learn_ported/com_jacobian.txt', delimiter=',', unpack=True)

plt.subplot(411)
plt.plot(ee_offset, my, 'b-o',markersize=2 , label='forward pass')
#plt.plot(delta_pos_range_vec, pos_margin_x, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$c_{y}$ [m]")
plt.ylabel("margin [m]")
plt.title("CoM, y")
plt.legend()

plt.subplot(412)
plt.plot(ee_offset, ee_jac_yx, 'b-o',markersize=2, label='learned (backprop)')
#plt.plot(ee_offset, ee_jac_yx_fin_diff, 'bo',markersize=2, label='finite diff')
#plt.plot(delta_pos_range_vec, pos_margin_x, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$c_{y}$ [m]")
plt.ylabel("$\delta m/  \delta c_{y}$ [m]")
plt.legend()

plt.subplot(413)
plt.plot(ee_offset, ee_jac_yy, 'b-o',markersize=2, label='learned (backprop)')
#plt.plot(ee_offset, ee_jac_yy_fin_diff, 'bo',markersize=2, label='finite diff')
#plt.plot(delta_pos_range_vec, pos_margin_x, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$c_{y}$ [m]")
plt.ylabel("$\delta m/  \delta c_{y}$ [m]")
plt.legend()

plt.subplot(414)
plt.plot(ee_offset, ee_jac_yz, 'b-o',markersize=2, label='learned (backprop)')
#plt.plot(ee_offset, ee_jac_zy_fin_diff, 'bo',markersize=2, label='finite diff')
#plt.plot(delta_pos_range_vec, pos_margin_x, 'g', markersize=15, label='CoM')
plt.grid()
plt.xlabel("$c_{z}$ [m]")
plt.ylabel("$\delta m/  \delta c_{z}$ [m]")
plt.legend()

plt.show()