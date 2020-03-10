"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np

from context import jet_leg

from numpy import array
from numpy.linalg import norm
from jet_leg.plotting.plotting_tools import Plotter

from jet_leg.computational_geometry.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters

import matplotlib as mpl
import matplotlib.pyplot as plt
import random

plt.close('all')
math = Math()

print ("''' ONLY_FRICTION, 3 CONTACTS'''")

# number of generators, i.e. rays used to linearize the friction cone
ng = 4

useVariableJacobian = False

''' parameters to be tuned'''
g = 9.81
total_mass = 85.
mu = 0.8

axisZ = array([[0.0], [0.0], [1.0]])

robot_name = 'anymal'
comp_dyn = ComputationalDynamics(robot_name)

number_of_tests = 1000
onlyFrictionTests3contacts = np.zeros((number_of_tests))
onlyFrictionTests4contacts = np.zeros((number_of_tests))
onlyActuationTests3contacts = np.zeros((number_of_tests))
onlyActuationTests4contacts = np.zeros((number_of_tests))
frictionAndActuation3contacts = np.zeros((number_of_tests))
frictionAndActuation4contacts = np.zeros((number_of_tests))

params = IterativeProjectionParameters(robot_name)


def perform_statistics(number_of_tests, number_of_contacts, _constraint_mode):
    computation_times = np.zeros((number_of_tests))
    params.setConstraintModes(constraint_mode_IP)
    for iter in range(0, number_of_tests):

        ''' random normals '''
        randRoll = np.random.normal(0.0, 0.2)
        randPitch = np.random.normal(0.0, 0.2)
        randYaw = np.random.normal(0.0, 0.2)
        n1 = np.transpose(np.transpose(math.rpyToRot(randRoll, randPitch, randYaw)).dot(axisZ))
        randRoll = np.random.normal(0.0, 0.2)
        randPitch = np.random.normal(0.0, 0.2)
        randYaw = np.random.normal(0.0, 0.2)
        n2 = np.transpose(np.transpose(math.rpyToRot(randRoll, randPitch, randYaw)).dot(axisZ))
        randRoll = np.random.normal(0.0, 0.2)
        randPitch = np.random.normal(0.0, 0.2)
        randYaw = np.random.normal(0.0, 0.2)
        n3 = np.transpose(np.transpose(math.rpyToRot(randRoll, randPitch, randYaw)).dot(axisZ))
        randRoll = np.random.normal(0.0, 0.2)
        randPitch = np.random.normal(0.0, 0.2)
        randYaw = np.random.normal(0.0, 0.2)
        n4 = np.transpose(np.transpose(math.rpyToRot(randRoll, randPitch, randYaw)).dot(axisZ))
        normals = np.vstack([n1, n2, n3, n4])

        """ contact points """
        sigma = 0.05  # mean and standard deviation
        randX = np.random.normal(0.3, sigma)
        randY = np.random.normal(0.2, sigma)
        randZ = np.random.normal(-0.5, sigma)
        LF_foot = np.array([randX, randY, randZ])
        randX = np.random.normal(0.3, sigma)
        randY = np.random.normal(-0.2, sigma)
        randZ = np.random.normal(-0.5, sigma)
        RF_foot = np.array([randX, randY, randZ])
        randX = np.random.normal(-0.3, sigma)
        randY = np.random.normal(0.2, sigma)
        randZ = np.random.normal(-0.5, sigma)
        LH_foot = np.array([randX, randY, randZ])
        randX = np.random.normal(-0.3, sigma)
        randY = np.random.normal(-0.2, sigma)
        randZ = np.random.normal(-0.5, sigma)
        RH_foot = np.array([randX, randY, randZ])
        contacts = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
        #    print contacts
        #    contacts = contactsToStack[0:nc, :]

        LF_tau_lim = [80.0, 100.0, 100.0]
        RF_tau_lim = [80.0, 100.0, 100.0]
        LH_tau_lim = [80.0, 100.0, 100.0]
        RH_tau_lim = [80.0, 100.0, 100.0]
        torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
        comWF = np.array([0.0, 0.0, 0.0])

        ''' compute iterative projection '''
        stanceLegs = [1, 1, 1, 1]
        if number_of_contacts is 3:
            randomSwingLeg = random.randint(0, 3)
            stanceLegs[randomSwingLeg] = 0
            print('swing leg is', randomSwingLeg)

        params.setContactsPosWF(contacts)
        params.setCoMPosWF(comWF)
        params.setTorqueLims(torque_limits)
        params.setActiveContacts(stanceLegs)
        params.setConstraintModes(constraint_mode_IP)
        params.setContactNormals(normals)
        params.setFrictionCoefficient(mu)
        params.setNumberOfFrictionConesEdges(ng)
        params.setTotalMass(total_mass)
        #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
        IP_points, actuation_polygons, comp_time = comp_dyn.iterative_projection_bretl(params)
        comp_time = comp_time * 1000.0

        print comp_time
        computation_times[iter] = comp_time
    return computation_times


'''ONLY FRICTION'''
# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode_IP = ['ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION']
contacts_number = 3
onlyFrictionTests3contacts = perform_statistics(number_of_tests, contacts_number, constraint_mode_IP)

print ("''' ONLY_FRICTION, 4 CONTACTS'''")
# ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
constraint_mode_IP = ['ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION',
                      'ONLY_FRICTION']
contacts_number = 4
onlyFrictionTests4contacts = perform_statistics(number_of_tests, contacts_number, constraint_mode_IP)

print ("''' ONLY ACTUATION, 3 CONTACTS'''")

constraint_mode_IP = ['ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION']
contacts_number = 3
onlyActuationTests3contacts = perform_statistics(number_of_tests, contacts_number, constraint_mode_IP)

print ("''' ONLY ACTUATION, 4 CONTACTS'''")
constraint_mode_IP = ['ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION',
                      'ONLY_ACTUATION']
contacts_number = 4
onlyActuationTests4contacts = perform_statistics(number_of_tests, contacts_number, constraint_mode_IP)

print ("''' FRICTION AND ACTUATION, 3 CONTACTS'''")

constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION']
contacts_number = 3
frictionAndActuation3contacts = perform_statistics(number_of_tests, contacts_number, constraint_mode_IP)

print ("''' FRICTION AND ACTUATION, 4 CONTACTS'''")
constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION']
contacts_number = 4
frictionAndActuation4contacts = perform_statistics(number_of_tests, contacts_number, constraint_mode_IP)

maximum_time = np.zeros((6,1))
maximum_time[0] = max(onlyFrictionTests3contacts)
maximum_time[1] = max(onlyFrictionTests4contacts)
maximum_time[2] = max(onlyActuationTests3contacts)
maximum_time[3] = max(onlyActuationTests4contacts)
maximum_time[4] = max(frictionAndActuation3contacts)
maximum_time[5] = max(frictionAndActuation4contacts)
plot_time_range = max(maximum_time)


print('max is', maximum_time)
''' plotting Iterative Projection points '''
print 'producing plot'
plotter = Plotter()

''' 2D figure '''
fig = plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")

plt.plot([1,2,3])
fig.suptitle('Computation times for ' + str(number_of_tests) + ' tests', fontsize=18)
plt.subplot(321)
#print tests
plt.hist(onlyFrictionTests3contacts, color = "salmon", bins=np.arange(0,plot_time_range, plot_time_range/50))
plt.xticks(np.arange(0, plot_time_range, plot_time_range/5))
plt.yticks(np.arange(0, 400, 50))
plt.grid()
plt.title("3 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")

subpl2 = plt.subplot(322)
#print tests
plt.hist(onlyFrictionTests4contacts, color = "salmon", bins=np.arange(0,plot_time_range, plot_time_range/50))
plt.xticks(np.arange(0, plot_time_range, plot_time_range/5))
plt.yticks(np.arange(0, 400, 50))
plt.grid()
plt.title("4 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")
subpl2.yaxis.set_label_position("right")

plt.subplot(323)
#print tests
plt.hist(onlyActuationTests3contacts, color = "limegreen", bins=np.arange(0,plot_time_range, plot_time_range/50))
plt.xticks(np.arange(0, plot_time_range, plot_time_range/5))
plt.yticks(np.arange(0, 400, 50))
plt.grid()
#plt.title("only friction, 3 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")

subpl4 = plt.subplot(324)
#print tests
plt.hist(onlyActuationTests4contacts, color = "limegreen", bins=np.arange(0,plot_time_range, plot_time_range/50))
plt.xticks(np.arange(0, plot_time_range, plot_time_range/5))
plt.yticks(np.arange(0, 400, 50))
plt.grid()
#plt.title("only friction, 4 point contacts")
plt.xlabel("times [ms]")
plt.ylabel("count")
subpl4.yaxis.set_label_position("right")

plt.subplot(325)
#print tests
plt.grid()
plt.hist(frictionAndActuation3contacts, color = "skyblue", bins=np.arange(0,maximum_time[4], maximum_time[4]/50))
plt.xticks(np.arange(0, maximum_time[4] , maximum_time[4]/5))
plt.yticks(np.arange(0, 400, 50))
#plt.title("only friction, 3 point contacts")
plt.xlabel("time [ms]")
plt.ylabel("count")

subpl6 = plt.subplot(326)
plt.grid()
plt.hist(frictionAndActuation4contacts, color = "skyblue", bins=np.arange(0,maximum_time[5] , maximum_time[5] /50))
plt.xlabel("time [ms]")
plt.ylabel("count")
plt.xticks(np.arange(0, maximum_time[5] , maximum_time[5] /5))
plt.yticks(np.arange(0, 400, 50))
subpl6.yaxis.set_label_position("right")
#plt.legend()
plt.show()
mpl.rcParams.update({'font.size': 14})

#plt.savefig('../figs/IP_bretl/statsIP.pdf')