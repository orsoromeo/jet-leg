# -*- coding: utf-8 -*-
"""
Created on Sat May 20 15:09:24 2018

@author: Romeo Orsolino
"""
import time
import cvxopt
from cvxopt import matrix, solvers
import numpy as np
from hyq_kinematics import HyQKinematics
from constraints import Constraints
from math_tools import Math
# for plotting
from plotting_tools import Plotter
from arrow3D import Arrow3D
import matplotlib.pyplot as plt
from computational_dynamics import ComputationalDynamics

"""
    In this file we can test whether, given a predefined position of the feet, there exist a set of contact forces
    that can ensure static stability and/or actuation consistency. This is done by solving a Linear Program where
    the contact forces are the optimization variables.
    
    General settings that can be changed by the user:
 
    nc = number of contacts
    mass = mass of the trunk of the robot
    constraint_mode = type of constraint to be enforced in the LP (three possible types only)
    LF_foot, .... RH_foot = positions of the feet in the base frame (the names recall the convention used for the HyQ robot)
    friction_coeff = friction coefficient in the contacts (for now the same values is used for all contacts)
    tau_lim_HAA, tau_lim_HFE, tau_lim_KFE = torque limits of the HAA, HFE and KFE joints of a single leg of HyQ

"""

constraint_mode = 'only_actuation'
nc = 3; # Number of contacts. This has been tested mainly for 3 and 4 contact points (keeping in mind a quadruped robot, but this is not mandatory)
mass = 10 # Kg
friction_coeff = 1.0
    
def getGraspMatrix(r):
    G = np.block([[np.eye(3), np.zeros((3,3))],
                   [math.skew(r), np.eye(3)]])
    return G
    
def computeActuationRegionAnalytical(force_polygons, mass, contacts):
    
    points_number = np.size(force_polygons,1)
    A1 = np.zeros((3,points_number))
    A2 = np.zeros((3,points_number))
    A3 = np.zeros((3,points_number))
    A4 = np.zeros((3,points_number))    
    p1 = contacts[:,0]    
    p2 = contacts[:,1]
    p3 = contacts[:,2]
    d = p2[0]-p3[0]
    a = (p1[1]-p3[1])/(p1[0]-p3[0])
    b = (p2[1]-p3[1]) - a * (p2[0]-p3[0])
    e = (p3[1] + p3[0] * a) * d
    g = 9.81    
    f = mass*g/(p1[0]-p3[0]) 

    for j in range(0,points_number):
        fz_lim = force_polygons[2,j]
        A1[0,j] = - a * mass * g / b
        A1[1,j] = - mass * g / b
        tmp = -mass * g / b *(p3[1] + a*p3[0])
        A1[2,j] = tmp - fz_lim
        A2[0,j] = -A1[0,j]
        A2[1,j] = -A1[1,j]
        A2[2,j] = - tmp - fz_lim
       
    for j in range(0,points_number):
        fz_lim = force_polygons[2,j]
        A3[0,j] = f
        A3[1,j] = - f * d
        tmp = f* (e / b - a * d / b)
        A3[2,j] = tmp - fz_lim
        A4[0,j] = - A3[0,j]
        A4[1,j] = - A3[1,j]
        A4[2,j] = - tmp - fz_lim
        
    A = np.hstack([A1, A2, A3, A4])
    print A
 
    return A

g = 9.81
grav = np.array([[0.], [0.], [-g*mass]])

""" contact points """
LF_foot = np.array([0.3, 0.2, -.5])
RF_foot = np.array([0.3, -0.2, -.5])
LH_foot = np.array([-0.3, 0.2, -.5])
RH_foot = np.array([-0.3, -0.2, -.5])
contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))

""" normals of the surface in the contact points """
math = Math()
LF_normal = np.array([[0.0], [0.0], [1.0]])
RF_normal = np.array([[0.0], [0.0], [1.0]])
LH_normal = np.array([[0.0], [0.0], [1.0]])
RH_normal = np.array([[0.0], [0.0], [1.0]])
LF_normal, RF_normal, LH_normal, RH_normal = (math.normalize(n) for n in [LF_normal, RF_normal, LH_normal, RH_normal])
normals = np.hstack((LF_normal, RF_normal, LH_normal, RH_normal))

#p = matrix(np.ones((3*nc,1)))
#
#cons1 = np.zeros((0,0))
#h_vec1 = np.zeros((0,1))
#cons2 = np.zeros((0,0))
#h_vec2 = np.zeros((0,1))
#
#kin = Kinematics()
#foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
#q, q_dot, J_LF, J_RF, J_LH, J_RH = kin.compute_xy_IK(np.transpose(contacts[:,0]),
#                                          np.transpose(foot_vel[:,0]),
#                                            np.transpose(contacts[:,2]),
#                                            np.transpose(foot_vel[:,2]))
#constraint = Constraints()                              
#actuation_polygon_LF = constraint.computeActuationPolygon(J_LF)
#actuation_polygon_RF = constraint.computeActuationPolygon(J_RF)
#actuation_polygon_RF = actuation_polygon_LF
#actuation_polygon_LH = constraint.computeActuationPolygon(J_LH)
#actuation_polygon_RH = constraint.computeActuationPolygon(J_RH)
#actuation_polygon_RH = actuation_polygon_LH
#print 'actuation polygon LF: ',actuation_polygon_LF
#print 'actuation polygon RF: ',actuation_polygon_RF
#print 'actuation polygon LH: ',actuation_polygon_LH
#print 'actuation polygon RH: ',actuation_polygon_RH
#
#""" construct the equations needed for the inequality constraints of the LP """   
#for j in range(0,nc):
#    c, h_term = constraint.linear_cone(normals[:,j],friction_coeff)
#    cons1 = np.block([[cons1, np.zeros((np.size(cons1,0),np.size(c,1)))],
#                  [np.zeros((np.size(c,0),np.size(cons1,1))), c]])
#    h_vec1 = np.vstack([h_vec1, h_term])
#    c, h_term = constraint.hexahedron(actuation_polygon_LF)
#    #c, h_term = constraint.zonotope()    
#    cons2 = np.block([[cons2, np.zeros((np.size(cons2,0),np.size(c,1)))],
#                  [np.zeros((np.size(c,0),np.size(cons2,1))), c]])    
#    h_vec2 = np.vstack([h_vec2, h_term])
#
#if constraint_mode == 'only_friction':
#    cons = cons1
#    h_vec = h_vec1
#elif constraint_mode == 'only_actuation':
#    cons = cons2
#    h_vec = h_vec2
#elif constraint_mode == 'friction_and_actuation':
#    cons = np.vstack([cons1, cons2])
#    h_vec = np.vstack([h_vec1, h_vec2])
#
#"""Definition of the inequality constraints"""
#m_ineq = np.size(cons,0)
##A=A.astype(double) 
##cons = cons.astype(np.double)
#G = matrix(cons) #matrix([[-1.0,0.0],[0.0,-1.0]])
#h = matrix(h_vec.reshape(m_ineq)) #matrix([0.0,0.0])
#print G, h
#print np.size(G,0), np.size(G,1)
#
#feasible_points = np.zeros((0,3))
#unfeasible_points = np.zeros((0,3))
#
#""" Defining the equality constraints """
#for com_x in np.arange(-0.6,0.7,0.5):
#    for com_y in np.arange(-0.6,0.5,0.5):
#        com = np.array([com_x, com_y, 0.0])
#        torque = -np.cross(com, np.transpose(grav))
#        A = np.zeros((6,0))
#        for j in range(0,nc):
#            r = contacts[j,:]
#            GraspMat = getGraspMatrix(r)
#            A = np.hstack((A, GraspMat[:,0:3]))
#        A = matrix(A)
#        b = matrix(np.vstack([-grav, np.transpose(torque)]).reshape((6)))
#        #A = matrix([1.0, 1.0], (1,2))
#        #b = matrix(1.0)
#
#        sol=solvers.lp(p, G, h, A, b)
#        x = sol['x']
#        status = sol['status']
#        #print status
#        if status == 'optimal':
#            feasible_points = np.vstack([feasible_points,com])
#        else:
#            unfeasible_points = np.vstack([unfeasible_points,com])
#
#
#print("--- %s seconds ---" % (time.time() - start_time))
#
#""" Plotting the results """
#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
#for j in range(0,nc):
#    ax.scatter(contacts[j,0], contacts[j,1], contacts[j,2],c='b',s=100)
#    a = Arrow3D([contacts[j,0], contacts[j,0]+normals[0,j]/3], [contacts[j,1], contacts[j,1]+normals[1,j]/3],[contacts[j,2], contacts[j,2]+normals[2,j]/3], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
#    ax.add_artist(a)
#    
#if np.size(feasible_points,0) != 0:
#    ax.scatter(feasible_points[:,0], feasible_points[:,1], feasible_points[:,2],c='g',s=50)
#if np.size(unfeasible_points,0) != 0:
#    ax.scatter(unfeasible_points[:,0], unfeasible_points[:,1], unfeasible_points[:,2],c='r',s=50)
#
#plotter = Plotter()                    
#plotter.plot_actuation_polygon(ax, actuation_polygon_LF, LF_foot)
#plotter.plot_actuation_polygon(ax, actuation_polygon_RF, RF_foot)
#plotter.plot_actuation_polygon(ax, actuation_polygon_LH, LH_foot)
#if nc == 4: plotter.plot_actuation_polygon(ax, actuation_polygon_RH, RH_foot)
#
#'''test the analytic computation of the actuation region'''
##dx = tau_lim_HAA
##dy = tau_lim_HFE
##dz = tau_lim_KFE
##vertices = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
##                     [dy, -dy, -dy, dy, dy, -dy, -dy, dy],
##                     [dz, dz, dz, dz, -dz, -dz, -dz, -dz]])
##edges = computeActuationRegionAnalytical(vertices, mass, contacts)
##plotter.plot_actuation_region(ax,edges)
#
#ax.set_xlabel('X Label')
#ax.set_ylabel('Y Label')
#ax.set_zlabel('Z Label')
#plt.draw()
#plt.show()
ng = 4
cd = ComputationalDynamics()
#cd.LP_projection(constraint_mode, contacts, normals, mass, friction_coeff, ng, nc)
print "bye bye" 
