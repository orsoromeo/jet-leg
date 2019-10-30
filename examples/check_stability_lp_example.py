"""
@author: romeo orsolino
"""

import numpy as np
import random
from jet_leg.math_tools import Math
from jet_leg.plotting_tools import Plotter
import matplotlib.pyplot as plt
from jet_leg.arrow3D import Arrow3D
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters

math = Math()
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_FRICTION
# ONLY_ACTUATION
constraint_mode_IP = ['ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION']

useVariableJacobian = True

comWF = np.array([1.25, 0.1, 0.0])

""" contact points in the World Frame"""
LF_foot = np.array([1.3, 0.2, -0.6])
RF_foot = np.array([1.3, -0.2, -0.5])
LH_foot = np.array([0.7, 0.2, -0.45])
RH_foot = np.array([0.7, -0.2, -0.5])

contactsToStack = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
contacts = contactsToStack[0:4, :]

''' parameters to be tuned'''
g = 9.81
trunk_mass = 50.
mu = 0.8

axisZ = np.array([[0.0], [0.0], [1.0]])

n1 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
# %% Cell 2
''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
stanceFeet = [1, 1, 1, 1]

randomSwingLeg = random.randint(0, 3)
tripleStance = False  # if you want you can define a swing leg using this variable
if tripleStance:
    print 'Swing leg', randomSwingLeg
    stanceFeet[0] = 0
print 'stanceLegs ', stanceFeet

normals = np.vstack([n1, n2, n3, n4])
comp_dyn = ComputationalDynamics()
params = IterativeProjectionParameters()
params.setContactsPosWF(contacts)
params.setCoMPosWF(comWF)
#       params.setTorqueLims(torque_limits)
params.setActiveContacts(stanceFeet)
params.setConstraintModes(constraint_mode_IP)
params.setContactNormals(normals)
params.setFrictionCoefficient(mu)
params.setNumberOfFrictionConesEdges(ng)
params.setTotalMass(trunk_mass)
isConfigurationStable, x, force_polytopes = comp_dyn.check_equilibrium(params)
print isConfigurationStable, x

'''Plotting the contact points in the 3D figure'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

''' plotting Iterative Projection points '''
plotter = Plotter()
scaling_factor = 2000
nc = sum(stanceFeet)
stanceID = params.getStanceIndex(stanceFeet)
for j in range(0, nc):  # this will only show the force polytopes of the feet that are defined to be in stance
    idx = int(stanceID[j])
    if (constraint_mode_IP[idx] == 'ONLY_ACTUATION') or (constraint_mode_IP[idx] == 'FRICTION_AND_ACTUATION'):
        plotter.plot_actuation_polygon(ax, force_polytopes[idx], contacts[idx, :], scaling_factor)
        plt.show()