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
from hyq_kinematics import HyQKinematics
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
        contactsNumber = np.size(contacts,0)

        # Unprojected state is:
        #
        #     x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
        math = Math()

        Ex = np.zeros((0)) 
        Ey = np.zeros((0))        
        G = np.zeros((6,0))   
        for j in range(0,contactsNumber):
            r = contacts[j,:]
            graspMatrix = self.getGraspMatrix(r)[:,0:3]
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
        # Inequality matrix for a contact force in local contact frame:
        constr = Constraints()
        #C_force = constr.linearized_cone_halfspaces(ng, mu)
        # Inequality matrix for stacked contact forces in world frame:
        if constraint_mode == 'ONLY_FRICTION':
            C, d = constr.linearized_cone_halfspaces_world(contactsNumber, ng, mu, normals)
            
        elif constraint_mode == 'ONLY_ACTUATION':
            kin = Kinematics()
            foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
            contactsFourLegs = np.vstack([contacts, np.zeros((4-contactsNumber,3))])
            q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = kin.compute_xy_IK(np.transpose(contactsFourLegs[:,0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contactsFourLegs[:,2]),
                                                    np.transpose(foot_vel[:,2]))

            act_LF = constr.computeActuationPolygon(J_LF)
            act_RF = constr.computeActuationPolygon(J_LF)
            act_LH = constr.computeActuationPolygon(J_LF)
            act_RH = constr.computeActuationPolygon(J_LF)            
            ''' in the case of the IP alg. the contact force limits must be divided by the mass
            because the gravito inertial wrench is normalized'''
            c1, e1 = constr.hexahedron(act_LF/mass)
            c2, e2 = constr.hexahedron(act_LF/mass)
            c3, e3 = constr.hexahedron(act_LF/mass)
            c4, e4 = constr.hexahedron(act_LF/mass)
            C = np.zeros((0,0))
            d = np.zeros((1,0))
            
            for j in range (0,contactsNumber):
                hexahedronHalfSpaceConstraints, knownTerm = constr.hexahedron(act_LF/mass)
                C = block_diag(C, hexahedronHalfSpaceConstraints)
                d = hstack([d, knownTerm.T])
                
            d = d.reshape(6*contactsNumber)    
            #C = block_diag(c1, c2, c3, c4)
            #d = np.vstack([e1, e2, e3, e4]).reshape(6*4)
            #print C, d
        
        ineq = (C, d)  # C * x <= d
        
        vertices = pypoman.project_polytope(proj, ineq, eq, method='bretl')

        print("Iterative Projection (Bretl): --- %s seconds ---" % (time.time() - start_t_IP))

        return vertices, act_LF, act_RF, act_RH, act_LH
                
                

    def LP_projection(self, constraint_mode, contacts, normals, mass, friction_coeff, ng, nc, mu, useVariableJacobian = False, stepX = 0.05, stepY = 0.05, stepZ = 0.05):
        start_t_LP = time.time()
        g = 9.81    
        grav = np.array([[0.], [0.], [-g*mass]])
        p = matrix(np.ones((3*nc,1)))    
        
        #kin = Kinematics()
        kin = HyQKinematics()
        constraint = Constraints()                              
                
        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
        contactsFourLegs = np.vstack([contacts, np.zeros((4-nc,3))])
        q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = kin.inverse_kin(np.transpose(contactsFourLegs[:,0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contactsFourLegs[:,1]),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contactsFourLegs[:,2]),
                                                    np.transpose(foot_vel[:,2]))
        
        G, h, isConstraintOk = constraint.inequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
        #print G, h
        #print np.size(G,0), np.size(G,1)
        
        feasible_points = np.zeros((0,3))
        unfeasible_points = np.zeros((0,3))
        contact_forces = np.zeros((0,nc*3))
        
        """ Defining the equality constraints """
        for com_x in np.arange(-0.5,0.5,stepX):
            for com_y in np.arange(-0.4,0.4,stepY):
                for com_z in np.arange(-0.,0.05,stepZ):
                    com = np.array([com_x, com_y, com_z])
                    torque = -np.cross(com, np.transpose(grav))
                    A = np.zeros((6,0))
                    for j in range(0,nc):
                        r = contacts[j,:]
                        GraspMat = self.getGraspMatrix(r)
                        A = np.hstack((A, GraspMat[:,0:3]))
                        A = matrix(A)
                        b = matrix(np.vstack([-grav, np.transpose(torque)]).reshape((6)))
                        
                    #contactsFourLegs = np.vstack([contacts, np.zeros((4-nc,3))])\
                    if (useVariableJacobian):
                        contacts_new_x = contactsFourLegs[:,0] + com_x
                        q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = kin.inverse_kin(np.transpose(contacts_new_x),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contactsFourLegs[:,1] + com_y),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contactsFourLegs[:,2] + com_z),
                                                    np.transpose(foot_vel[:,2]))
                    if (not isOutOfWorkspace):
                        kin.update_jacobians(q)
                        #print J_LF
                        G, h, isConstraintOk = constraint.inequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
                        print G, h
                        if not isConstraintOk:
                            print 'something is wrong in the inequalities'
                        else:
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
