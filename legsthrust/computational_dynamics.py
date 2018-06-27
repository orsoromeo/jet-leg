# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 15:57:17 2018

@author: Romeo Orsolino
"""

import pylab
import pypoman
import numpy as np
from numpy import array, cross, dot, eye, hstack, vstack, zeros
from numpy.linalg import norm
import scipy
from scipy.linalg import block_diag
from plotting_tools import Plotter
from constraints import Constraints
from kinematics import Kinematics
from math_tools import Math
from cvxopt import matrix, solvers
import time
import matplotlib.pyplot as plt
from arrow3D import Arrow3D

class ComputationalDynamics():
    def getGraspMatrix(self, r):
        math = Math()
        G = np.vstack([np.hstack([eye(3), zeros((3, 3))]),np.hstack([math.skew(r), eye(3)])])
        return G
    
    def iterative_projection_bretl(self, constraint_mode, contacts, normals, mass, ng, mu):
        start_t_IP = time.time()
        g = 9.81
        grav = array([0., 0., -g])
        # Unprojected state is:
        #
        #     x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
        math = Math()
        r1 = contacts[0,:]
        r2 = contacts[1,:]
        r3 = contacts[2,:]
        G1 = self.getGraspMatrix(r1)[:, 0:3]
        G2 = self.getGraspMatrix(r2)[:, 0:3]
        G3 = self.getGraspMatrix(r3)[:, 0:3]
        
        # Projection matrix
        Ex = -hstack((G1[4], G2[4], G3[4]))
        Ey = +hstack((G1[3], G2[3], G3[3]))
        E = vstack((Ex, Ey)) / (g)
        f = zeros(2)
        proj = (E, f)  # y = E * x + f
        
        # number of the equality constraints
        m_eq = 6
        
        # grasp matrix for the stacked vector of contact forces
        G = hstack((G1, G2, G3))
        
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
        
        # Contact surface normals
        
        n1 = normals[0,:]
        n2 = normals[1,:]
        n3 = normals[2,:]
        n1, n2, n3 = (math.normalize(n) for n in [n1, n2, n3])
        #print n1, n2, n3
        R1, R2, R3 = (math.rotation_matrix_from_normal(n) for n in [n1, n2, n3])
        
        # Inequality matrix for a contact force in local contact frame:
        constr = Constraints()
        #C_force = constr.linearized_cone_halfspaces(ng, mu)
        # Inequality matrix for stacked contact forces in world frame:
        if constraint_mode == 'ONLY_FRICTION':
            #C = block_diag(
            #dot(C_force, R1.T),
            #dot(C_force, R2.T),
            #dot(C_force, R3.T))
            #d = zeros(C.shape[0])
            C, d = constr.linearized_cone_halfspaces_world(ng, mu, n1, n2, n3)
            
        elif constraint_mode == 'ONLY_ACTUATION':
            kin = Kinematics()
            foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
            q, q_dot, J_LF, J_RF, J_LH, J_RH = kin.compute_xy_IK(np.transpose(contacts[:,0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contacts[:,2]),
                                                    np.transpose(foot_vel[:,2]))

            act_LF = constr.computeActuationPolygon(J_LF)
            act_LH = constr.computeActuationPolygon(J_LF)
            act_RF = constr.computeActuationPolygon(J_LF)
            ''' in the case of the IP alg. the contact force limits must be divided by the mass
            because the gravito inertial wrench is normalized'''
            c1, e1 = constr.hexahedron(act_LF/mass)
            c2, e2 = constr.hexahedron(act_LF/mass)
            c3, e3 = constr.hexahedron(act_LF/mass)
            C = block_diag(c1, c2, c3)
            d = np.vstack([e1, e2, e3]).reshape(18)
            #print C, d
        
        ineq = (C, d)  # C * x <= d
        
        vertices = pypoman.project_polytope(proj, ineq, eq, method='bretl')

        print("Iterative Projection (Bretl): --- %s seconds ---" % (time.time() - start_t_IP))
        return vertices

    def LP_projection(self, constraint_mode, contacts, normals, mass, friction_coeff, ng, nc, mu):
        start_t_LP = time.time()
        g = 9.81    
        grav = np.array([[0.], [0.], [-g*mass]])
        p = matrix(np.ones((3*nc,1)))    
        cons1 = np.zeros((0,0))
        h_vec1 = np.zeros((0,1))
        cons2 = np.zeros((0,0))
        h_vec2 = np.zeros((0,1))
        
        kin = Kinematics()
        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
        q, q_dot, J_LF, J_RF, J_LH, J_RH = kin.compute_xy_IK(np.transpose(contacts[:,0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contacts[:,2]),
                                                    np.transpose(foot_vel[:,2]))
        constraint = Constraints()                              
        actuation_polygon_LF = constraint.computeActuationPolygon(J_LF)
        actuation_polygon_RF = constraint.computeActuationPolygon(J_RF)
        actuation_polygon_RF = actuation_polygon_LF
        actuation_polygon_LH = constraint.computeActuationPolygon(J_LH)
        actuation_polygon_RH = constraint.computeActuationPolygon(J_RH)
        actuation_polygon_RH = actuation_polygon_LF
        #print 'actuation polygon LF: ',actuation_polygon_LF
        #print 'actuation polygon RF: ',actuation_polygon_RF
        #print 'actuation polygon LH: ',actuation_polygon_LH
        #print 'actuation polygon RH: ',actuation_polygon_RH
        
        """ construct the equations needed for the inequality constraints of the LP """   
        for j in range(0,nc):
            c, h_term = constraint.linear_cone(normals[j,:],friction_coeff)
            cons1 = np.vstack([np.hstack([cons1, np.zeros((np.size(cons1,0),np.size(c,1)))]),
                                          np.hstack([np.zeros((np.size(c,0),np.size(cons1,1))), c])])

            h_vec1 = np.vstack([h_vec1, h_term])
            c, h_term = constraint.hexahedron(actuation_polygon_LF)
            #c, h_term = constraint.zonotope()    
            cons2 = np.vstack([np.hstack([cons2, np.zeros((np.size(cons2,0),np.size(c,1)))]),
                          np.hstack([np.zeros((np.size(c,0),np.size(cons2,1))), c])])    
            h_vec2 = np.vstack([h_vec2, h_term])
        
        n1 = normals[0,:]
        n2 = normals[1,:]
        n3 = normals[2,:]
        math_lp = Math()
        n1, n2, n3 = (math_lp.normalize(n) for n in [n1, n2, n3])
        #R1, R2, R3 = (math_lp.rotation_matrix_from_normal(n) for n in [n1, n2, n3])
        # Inequality matrix for a contact force in local contact frame:
        constr = Constraints()
        #C_force = constr.linearized_cone_halfspaces(ng, mu)
        
        if constraint_mode == 'ONLY_FRICTION':
            #cons = cons1
            #h_vec = h_vec1
            cons, h_vec = constr.linearized_cone_halfspaces_world(ng, mu, n1, n2, n3)
            #cons = block_diag(
            #dot(C_force, R1.T),
            #dot(C_force, R2.T),
            #dot(C_force, R3.T))
            #h_vec = zeros(cons.shape[0])
            
        elif constraint_mode == 'ONLY_ACTUATION':
            cons = cons2
            h_vec = h_vec2
        elif constraint_mode == 'friction_and_actuation':
            cons = np.vstack([cons1, cons2])
            h_vec = np.vstack([h_vec1, h_vec2])
        
        """Definition of the inequality constraints"""
        m_ineq = np.size(cons,0)
        #A=A.astype(double) 
        #cons = cons.astype(np.double)
        G = matrix(cons) #matrix([[-1.0,0.0],[0.0,-1.0]])
        h = matrix(h_vec.reshape(m_ineq)) #matrix([0.0,0.0])
        #print G, h
        #print np.size(G,0), np.size(G,1)
        
        feasible_points = np.zeros((0,3))
        unfeasible_points = np.zeros((0,3))
        contact_forces = np.zeros((0,9))
        """ Defining the equality constraints """

        for com_x in np.arange(-0.6,0.7,0.025):
            for com_y in np.arange(-0.6,0.5,0.025):
#        for com_x in np.arange(-0.05,-0.04,0.001):
#            for com_y in np.arange(-0.02,-0.01,0.001):
                com = np.array([com_x, com_y, 0.0])
                torque = -np.cross(com, np.transpose(grav))
                A = np.zeros((6,0))
                for j in range(0,nc):
                    r = contacts[j,:]
                    GraspMat = self.getGraspMatrix(r)
                    A = np.hstack((A, GraspMat[:,0:3]))
                A = matrix(A)
                b = matrix(np.vstack([-grav, np.transpose(torque)]).reshape((6)))
                #A = matrix([1.0, 1.0], (1,2))
                #b = matrix(1.0)
        
                sol=solvers.lp(p, G, h, A, b)
                x = sol['x']
                status = sol['status']
                #print x
                if status == 'optimal':
                    feasible_points = np.vstack([feasible_points,com])
                    contact_forces = np.vstack([contact_forces, np.transpose(x)])
                else:
                    unfeasible_points = np.vstack([unfeasible_points,com])
        
        
        print("LP test: --- %s seconds ---" % (time.time() - start_t_LP))
        
        return feasible_points, unfeasible_points, contact_forces
