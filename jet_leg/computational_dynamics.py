# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 15:57:17 2018

@author: Romeo Orsolino
"""

import pylab
import pypoman
import numpy as np
from numpy import array, dot, eye, hstack, vstack, zeros
from numpy.linalg import norm
import scipy
from scipy.linalg import block_diag

from constraints import Constraints

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

    ''' 
    This function is used to prepare all the variables that will be later used 
    for the computation of the Iterative Projection algorithm 
    
    The arguments are: 
    constraint_mode = either 'ONLY_ACTUATION' or 'ONLY_FRICTION'
    comWF = position of the CoM
    contacts = location of the nc contacts
    normals = [3x nc] matrix containing the normals of each contact point on its rows
    trunk_mass = mass of the robots trunk (excluding the legs mass) in Kg 
    nc = number of point contacts
    ng = number of edges to be used to linearize the friction cones (in case that the friction constraint is considered)
    mu = friction coefficient (we assume here the same coeff for all the contact points)
    saturate_normal_force = if True this sets a max constant value on the normal force of the friction cones
    '''
    def setup_iterative_projection(self, constraint_mode, comWF, contacts, normals, trunk_mass, ng, mu, saturate_normal_force):
        ''' parameters to be tuned'''
        g = 9.81
        isOutOfWorkspace = False;

        grav = array([0., 0., -g])
        contactsNumber = np.size(contacts,0)
        
        compDyn = ComputationalDynamics()
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
                
        return proj, eq, ineq, actuation_polygons
        
    def iterative_projection_bretl(self, constraint_mode, contacts, normals, trunk_mass, ng, mu, comWF = np.array([0.0,0.0,0.0]), saturate_normal_force = False):

        start_t_IP = time.time()
        
        proj, eq, ineq, actuation_polygons = self.setup_iterative_projection(constraint_mode, comWF, contacts, normals, trunk_mass, ng, mu, saturate_normal_force)
        vertices_WF = pypoman.project_polytope(proj, ineq, eq, method='bretl')
        #vertices_WF = vertices_BF + np.transpose(comWF[0:2])
        computation_time = (time.time() - start_t_IP)
        print("Iterative Projection (Bretl): --- %s seconds ---" % computation_time)

        return vertices_WF, actuation_polygons, computation_time
        
        
    def instantaneous_actuation_region_bretl(self, contacts, normals, trunk_mass, comWF = np.array([0.0,0.0,0.0])):
        constraint_mode = 'ONLY_ACTUATION'
        number_of_generators = 4
        mu = 1.0
        IP_points, actuation_polygons, computation_time = self.iterative_projection_bretl(constraint_mode, contacts, normals, trunk_mass, number_of_generators, mu, comWF)

        return IP_points, actuation_polygons, computation_time
                
                
    ''' 
    This function is used to check every single CoM position and see if there is a feasible set of contact forces for that configuration.
    This function can consider either friction constraints or actuation constraints separately (depending on the value of the constraint_mode variable).
    
    The arguments are: 
    constraint_mode = either 'ONLY_ACTUATION' or 'ONLY_FRICTION'
    contacts = location of the nc contacts
    normals = [3x nc] matrix containing the normals of each contact point on its rows
    mass = mass of the robots trunk (excluding the legs mass) in Kg 
    friction_coeff = friction coefficient (we assume here the same coeff for all the contact points)
    nc = number of point contacts
    ng = number of edges to be used to linearize the friction cones (in case that the friction constraint is considered)
    '''
    def LP_projection(self, constraint_mode, contacts, normals, mass, friction_coeff, ng, nc, useVariableJacobian = False, stepX = 0.05, stepY = 0.05, stepZ = 0.05):
        start_t_LP = time.time()
        feasible_points = np.zeros((0,3))
        unfeasible_points = np.zeros((0,3))
        contact_forces = np.zeros((0,nc*3))  
        verbose = False
        com_WF = np.array([0.0, 0.0, 0.0])
        default_com_WF = com_WF
        p, G, h, A, b, isConstraintOk, LP_actuation_polygons = self.setup_lp(mass, contacts, nc, ng, normals, com_WF, constraint_mode, friction_coeff)
        
        """ Defining the equality constraints """
        for com_x in np.arange(-0.5,0.5,stepX):
            for com_y in np.arange(-0.4,0.4,stepY):
                for com_z in np.arange(-0.2,0.25,stepZ):
                    com_WF = np.array([com_x, com_y, com_z])

                    if useVariableJacobian:
                         p, G, h, A, b, isConstraintOk, LP_actuation_polygons = self.setup_lp(mass, contacts, nc, ng, normals, com_WF, constraint_mode, friction_coeff)      
                    else:
                         p, G, h, A, b, isConstraintOk, LP_actuation_polygons = self.setup_lp(mass, contacts, nc, ng, normals, default_com_WF, constraint_mode, friction_coeff)      

                    if not isConstraintOk:
                        unfeasible_points = np.vstack([unfeasible_points, com_WF])
                        if verbose:
                            print 'something is wrong in the inequalities or the point is out of workspace'
                    else:
                        
                        sol=solvers.lp(p, G, h, A, b)
                        x = sol['x']
                        status = sol['status']
                        #print x
                        if status == 'optimal':
                            feasible_points = np.vstack([feasible_points, com_WF])
                            contact_forces = np.vstack([contact_forces, np.transpose(x)])
                        else:
                            unfeasible_points = np.vstack([unfeasible_points, com_WF])
                    
        print("LP test: --- %s seconds ---" % (time.time() - start_t_LP))
        
        return feasible_points, unfeasible_points, contact_forces

    ''' 
    This function is used to prepare all the variables that will be later used 
    for the computation of the LP ground truth. 
    
    The arguments are: 
    mass = mass of the robots trunk (excluding the legs mass) in Kg 
    nc = number of point contacts
    contacts = location of the nc contacts
    numberOfGenerators = number of edges to be used to linearize the friction cones (in case that the friction constraint is considered)
    normals = [3x nc] matrix containing the normals of each contact point on its rows
    comWorldFrame = position of the CoM
    constraint_mode = either 'ONLY_ACTUATION' or 'ONLY_FRICTION'
    friction_coeff = friction coefficient (we assume here the same coeff for all the contact points)
    '''
    def setup_lp(self, mass, contacts, nc, numberOfGenerators, normals, comWorldFrame, constraint_mode, friction_coeff):
        g = 9.81    
        grav = np.array([[0.], [0.], [-g*mass]])

        p = matrix(np.zeros((3*nc,1)))                        
    
        kin = HyQKinematics()
        constraint = Constraints()                              
        
        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
        contactsFourLegsWF = np.vstack([contacts, np.zeros((4-nc,3))])
        #kin.inverse_kin(np.transpose(contactsFourLegsWF[:,0])- comWorldFrame[0],
        #                np.transpose(foot_vel[:,0]),
        #                np.transpose(contactsFourLegsWF[:,1]- comWorldFrame[1]),
        #               np.transpose(foot_vel[:,1]),
        #               np.transpose(contactsFourLegsWF[:,2]- comWorldFrame[2]),
        #               np.transpose(foot_vel[:,2]))
        
        torque = -np.cross(comWorldFrame, np.transpose(grav)) 
        A = np.zeros((6,0))
        for j in range(0,nc):
            r = contacts[j,:]
            GraspMat = self.getGraspMatrix(r)
            A = np.hstack((A, GraspMat[:,0:3]))
            A = matrix(A)
            b = matrix(np.vstack([-grav, np.transpose(torque)]).reshape((6)))
            
        #contactsFourLegs = np.vstack([contacts, np.zeros((4-nc,3))])\
        #if (useVariableJacobian):
            #contacts_new_x = contactsFourLegs[:,0] + com_x
        q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = kin.inverse_kin(np.transpose(contactsFourLegsWF[:,0] - comWorldFrame[0]),
                                              np.transpose(foot_vel[:,0]),
                                                    np.transpose(contactsFourLegsWF[:,1] - comWorldFrame[1]),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contactsFourLegsWF[:,2] - comWorldFrame[2]),
                                                    np.transpose(foot_vel[:,2]))
        if (not isOutOfWorkspace):
            #kin.update_jacobians(q)
            J_LF, J_RF, J_LH, J_RH = kin.update_jacobians(q)
            #print J_LF
            G, h, isLpOK, LP_actuation_polygons = constraint.getInequalities(constraint_mode, nc, numberOfGenerators, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
        
        else:
            LP_actuation_polygons = [None]
            isLpOK = False
            G= [None]
            h = [None]
        
        lp = p, G, h, A, b
        return p, G, h, A, b, isLpOK, LP_actuation_polygons