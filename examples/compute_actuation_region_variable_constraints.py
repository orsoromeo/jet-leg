# -*- coding: utf-8 -*-
"""
Created on Tue Aug 14 14:07:15 2018

@author: romeoorsolino
"""
for name in dir():
    if not name.startswith('_'):
        del globals()[name]
        
import numpy as np

from context import jet_leg 

import time

import random
import cvxopt
from cvxopt import matrix, solvers

from numpy import array, cos, cross, pi, sin
from numpy.random import random
from scipy.linalg import norm
from scipy.linalg import block_diag

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix

import matplotlib.pyplot as plt
from jet_leg.plotting_tools import Plotter
from jet_leg.constraints import Constraints
from jet_leg.hyq_kinematics import HyQKinematics
from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.height_map import HeightMap
from jet_leg.sequential_iterative_projection import SequentialIterativeProjection
        
class IterativeProjection:
        
    def setup_iterative_projection(self, contacts, comWF, trunk_mass, mu, normals):
        ''' parameters to be tuned'''
        g = 9.81
        isOutOfWorkspace = False;

        grav = array([0., 0., -g])
        contactsNumber = np.size(contacts,0)
        # Unprojected state is:
        #
        #     x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
        Ex = np.zeros((0)) 
        Ey = np.zeros((0))        
        G = np.zeros((6,0))   
        for j in range(0,contactsNumber):
            r = contacts[j,:]
            graspMatrix = compDyn.getGraspMatrix(r)[:,0:3]
            Ex = hstack([Ex, -graspMatrix[4]])
            Ey = hstack([Ey, graspMatrix[3]])
            G = hstack([G, graspMatrix])            
            
        E = vstack((Ex, Ey)) / (g)
        f = zeros(2)
        proj = (E, f)  # y = E * x + f
        
        # number of the equality constraints
        m_eq = 6
        
        # see Equation (52) in "ZMP Support Areas for Multicontact..."
        A_f_and_tauz = array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1]])
        A = dot(A_f_and_tauz, G)
        t = hstack([0, 0, g, 0])
        #print A,t
        eq = (A, t)  # A * x == t
        
        act_LF = np.zeros((0,1))
        act_RF = np.zeros((0,1))
        act_LH = np.zeros((0,1))
        act_RH = np.zeros((0,1))
        actuation_polygons = np.zeros((0,1))
        # Inequality matrix for a contact force in local contact frame:
        constr = Constraints()
        #C_force = constr.linearized_cone_halfspaces(ng, mu)
        # Inequality matrix for stacked contact forces in world frame:
        if constraint_mode == 'ONLY_FRICTION':
            C, d = constr.linearized_cone_halfspaces_world(contactsNumber, ng, mu, normals)
            
        elif constraint_mode == 'ONLY_ACTUATION':
            #kin = Kinematics()
            kin = HyQKinematics()
            foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
            contactsFourLegs = np.vstack([contacts, np.zeros((4-contactsNumber,3))])
            q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = kin.inverse_kin(np.transpose(contactsFourLegs[:,0] - comWF[0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contactsFourLegs[:,1] - comWF[1]),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contactsFourLegs[:,2] - comWF[2]),
                                                    np.transpose(foot_vel[:,2]))
            J_LF, J_RF, J_LH, J_RH = kin.update_jacobians(q)

            if isOutOfWorkspace:
                C = np.zeros((0,0))
                d = np.zeros((1,0))
            else:
                act_LF = constr.computeActuationPolygon(J_LF)
                act_RF = constr.computeActuationPolygon(J_RF)
                act_LH = constr.computeActuationPolygon(J_LH)
                act_RH = constr.computeActuationPolygon(J_RH)            
                ''' in the case of the IP alg. the contact force limits must be divided by the mass
                because the gravito inertial wrench is normalized'''
                
                C = np.zeros((0,0))
                d = np.zeros((1,0))
                actuation_polygons = np.array([act_LF,act_RF,act_LH,act_RH])
                for j in range (0,contactsNumber):
                    hexahedronHalfSpaceConstraints, knownTerm = constr.hexahedron(actuation_polygons[j]/trunk_mass)
                    C = block_diag(C, hexahedronHalfSpaceConstraints)
                    d = hstack([d, knownTerm.T])
                    
                d = d.reshape(6*contactsNumber)    
                #C = block_diag(c1, c2, c3, c4)
                #d = np.vstack([e1, e2, e3, e4]).reshape(6*4)
                #print C, d
        
        ineq = (C, d)  # C * x <= d
        
        if isOutOfWorkspace:
            lp = 0
        else:
            max_radius=1e5
            (E, f), (A, b), (C, d) = proj, ineq, eq
            assert E.shape[0] == f.shape[0] == 2
            # Inequality constraints: A_ext * [ x  u  v ] <= b_ext iff
            # (1) A * x <= b and (2) |u|, |v| <= max_radius
            A_ext = zeros((A.shape[0] + 4, A.shape[1] + 2))
            A_ext[:-4, :-2] = A
            A_ext[-4, -2] = 1
            A_ext[-3, -2] = -1
            A_ext[-2, -1] = 1
            A_ext[-1, -1] = -1
            A_ext = cvxopt.matrix(A_ext)
            
            b_ext = zeros(b.shape[0] + 4)
            b_ext[:-4] = b
            b_ext[-4:] = array([max_radius] * 4)
            b_ext = cvxopt.matrix(b_ext)
            
            # Equality constraints: C_ext * [ x  u  v ] == d_ext iff
            # (1) C * x == d and (2) [ u  v ] == E * x + f
            C_ext = zeros((C.shape[0] + 2, C.shape[1] + 2))
            C_ext[:-2, :-2] = C
            C_ext[-2:, :-2] = E[:2]
            C_ext[-2:, -2:] = array([[-1, 0], [0, -1]])
            C_ext = cvxopt.matrix(C_ext)
            
            d_ext = zeros(d.shape[0] + 2)
            d_ext[:-2] = d
            d_ext[-2:] = -f[:2]
            d_ext = cvxopt.matrix(d_ext)
            
            lp_obj = cvxopt.matrix(zeros(A.shape[1] + 2))
            lp = lp_obj, A_ext, b_ext, C_ext, d_ext
        
        return lp, actuation_polygons/trunk_mass, isOutOfWorkspace
    

''' MAIN '''
start_t_IPVC = time.time()
math = Math()
compDyn = ComputationalDynamics()
sequentialIP = SequentialIterativeProjection()
# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION or ONLY_FRICTION
constraint_mode = 'ONLY_ACTUATION'
useVariableJacobian = True
trunk_mass = 100
mu = 0.8
# number of decision variables of the problem
n = nc*6
i = 0
comTrajectoriesToStack = np.zeros((0,3))
terrain = HeightMap()

comWF = np.array([0.0, 0.15, 0.0])
optimizedVariablesToStack = np.zeros((0,3))
iterProj = IterativeProjection()        
for_iter = 0
rand = random()
step = pi/4
iterations_number = 8#np.random.randint(2,5)
for angle in np.arange(rand,rand+step*iterations_number,step):
    print "angle is: ", angle
    dir_x = np.cos(angle)
    dir_y = np.sin(angle)
    desired_dir = np.array([dir_x, dir_y, 0.0])
    #print "direction: ", desired_direction
    """ contact points """
    LF_foot = np.array([0.4, 0.3, -0.5])
    RF_foot = np.array([0.4, -0.3, -0.5])
    LH_foot = np.array([-0.4, 0.3, -0.5])
    #print "Terrain height: ", LH_foot        
    RH_foot = np.array([-0.4, -0.3, -0.5])

    contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
    contacts = contactsToStack[0:nc, :]
    
    comToStack = sequentialIP.optimize_direction_variable_constraints(constraint_mode, desired_dir, contacts, comWF)
        
    for_iter += 1
    #print "for ",for_iter
    comTrajectoriesToStack = np.vstack([comTrajectoriesToStack, comToStack[-1]])
    optimizedVariablesToStack = np.vstack([optimizedVariablesToStack, np.array([-0.4, 0.3, dir_y])])

print "Final CoM points ", comTrajectoriesToStack
print "Tested combinations: ", optimizedVariablesToStack

max_motion_indices = np.unravel_index(np.argsort(comTrajectoriesToStack[:,0], axis=None), comTrajectoriesToStack[:,0].shape)
max_motin_index = max_motion_indices[0][0]
print("Directed Iterative Projection: --- %s seconds ---" % (time.time() - start_t_IPVC))


''' plotting Iterative Projection points '''

axisZ= array([[0.0], [0.0], [1.0]])
n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2
        
normals = np.vstack([n1, n2, n3, n4])
IP_points, actuation_polygons = compDyn.iterative_projection_bretl(constraint_mode, contacts, normals, trunk_mass, ng, mu)

feasible, unfeasible, contact_forces = compDyn.LP_projection(constraint_mode, contacts, normals, trunk_mass, mu, ng, nc, mu, useVariableJacobian, 0.05, 0.05)

#IP_points_saturated_friction, actuation_polygons = compDyn.iterative_projection_bretl('ONLY_FRICTION', contacts, normals, trunk_mass, ng, mu, saturate_normal_force = True)

'''Plotting'''
plt.close('all')
plotter = Plotter()

plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
#plt.plot(intersection[:,0], intersection[:,1], 'ro', markersize=15)
#plt.plot(final_points[:,0], final_points[:,1], 'r^', markersize=20)
#plt.plot(comToStack[:,0], comToStack[:,1], 'g^', markersize=20)
#plt.plot(comToStack[-1,0], comToStack[-1,1], 'bo', markersize=20)

com_limit_x = np.hstack([comTrajectoriesToStack[:,0],comTrajectoriesToStack[0,0]])
com_limit_y = np.hstack([comTrajectoriesToStack[:,1],comTrajectoriesToStack[0,1]])
plt.plot(comTrajectoriesToStack[:,0], comTrajectoriesToStack[:,1], 'g^', markersize=20)
plt.plot(com_limit_x, com_limit_y, 'g--', linewidth=5)
#plt.plot(comTrajectoriesToStack[max_motin_index,0], comTrajectoriesToStack[max_motin_index,1], 'y^', markersize=25)
h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')

#plotter.plot_polygon(np.asanyarray(vertices1), color = 'y')

#plotter.plot_polygon(np.asanyarray(vertices2), color = 'r')

#plotter.plot_polygon(np.asanyarray(vertices3), color = 'b')

#plotter.plot_polygon(np.asanyarray(vertices4), color = 'g')

#plotter.plot_polygon(np.asanyarray(vertices5), color = 'k')

#plotter.plot_polygon(np.transpose(IP_points))

#plotter.plot_polygon(np.transpose(IP_points_saturated_friction), color = '--r')

feasiblePointsSize = np.size(feasible,0)
for j in range(0, feasiblePointsSize):
    if (feasible[j,2]<0.01)&(feasible[j,2]>-0.01):
        plt.scatter(feasible[j,0], feasible[j,1],c='g',s=50)
        lastFeasibleIndex = j
unfeasiblePointsSize = np.size(unfeasible,0)

for j in range(0, unfeasiblePointsSize):
    if (unfeasible[j,2]<0.01)&(unfeasible[j,2]>-0.01):
        plt.scatter(unfeasible[j,0], unfeasible[j,1],c='r',s=50)
        lastUnfeasibleIndex = j
h2 = plt.scatter(feasible[lastFeasibleIndex,0], feasible[lastFeasibleIndex,1],c='g',s=50, label='LP feasible')
h3 = plt.scatter(unfeasible[lastUnfeasibleIndex,0], unfeasible[lastUnfeasibleIndex,1],c='r',s=50, label='LP unfeasible')

plt.plot(optimizedVariablesToStack[:,0], optimizedVariablesToStack[:,1], 'ko', markersize = 15)
#plt.plot(optimizedVariablesToStack[max_motin_index,0], optimizedVariablesToStack[max_motin_index,1], 'yo', markersize=25)
plt.xlim(-0.9, 0.5)
plt.ylim(-0.7, 0.7)
plt.legend()
plt.show()

