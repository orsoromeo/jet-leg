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
from scipy.spatial import ConvexHull

from constraints import Constraints
from hyq_kinematics import HyQKinematics
from math_tools import Math
from geometry import Geometry
from cvxopt import matrix, solvers
import time
import matplotlib.pyplot as plt
from arrow3D import Arrow3D

class ComputationalDynamics():
    def __init__(self):
        self.geom = Geometry()
        self.math = Math()
        self.constr = Constraints()
        self.kin = HyQKinematics()
        
    def getGraspMatrix(self, r):

        G = np.vstack([np.hstack([eye(3), zeros((3, 3))]),np.hstack([self.math.skew(r), eye(3)])])
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
    def setup_iterative_projection(self, iterative_projection_params, saturate_normal_force):
        
        stanceLegs = iterative_projection_params.getStanceFeet()
        contacts = iterative_projection_params.getContactsPos()
        constraint_mode = iterative_projection_params.getConstraintModes()
        constraint_mode = constraint_mode[0]
        comWF = iterative_projection_params.getCoMPos()
        torque_limits = iterative_projection_params.getTorqueLims()
        trunk_mass = iterative_projection_params.getTrunkMass()
        ng = iterative_projection_params.getNumberOfFrictionConesEdges()
        mu = iterative_projection_params.getFrictionCoefficient()
        normals = iterative_projection_params.getNormals()
        
        
        ''' parameters to be tuned'''
        g = 9.81
#        print 'torque lims', torque_limits

        grav = array([0., 0., -g])
        contactsNumber = np.sum(stanceLegs)
        print 'num',contactsNumber
        
        # Unprojected state is:
        #
        #     x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
        Ex = np.zeros((0)) 
        Ey = np.zeros((0))        
        G = np.zeros((6,0))   
        
        stanceIndex = []
        swingIndex = []
#        print 'stance', stanceLegs
        for iter in range(0, 4):
            if stanceLegs[iter] == 1:
#                print 'new poly', stanceIndex, iter
                stanceIndex = np.hstack([stanceIndex, iter])
            else:
                swingIndex = iter
                
        for j in range(0,contactsNumber):
#            print 'iter',j, int(stanceIndex[j])
            r = contacts[int(stanceIndex[j]),:]
            graspMatrix = self.getGraspMatrix(r)[:,0:3]
            Ex = hstack([Ex, -graspMatrix[4]])
            Ey = hstack([Ey, graspMatrix[3]])
            G = hstack([G, graspMatrix])            
            
#        print 'grasp matrix',G
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
        
        actuation_polygons = np.zeros((1,1))
        C = np.zeros((0,0))
        d = np.zeros((0))
        
        for j in range(0,contactsNumber):    
            if constraint_mode == 'ONLY_FRICTION':
                #            print contactsNumber
                constraints_local_frame, d_cone = self.constr.linearized_cone_halfspaces_world(contactsNumber, ng, mu, normals)
                isIKoutOfWorkSpace = False

                n = self.math.normalize(normals[j,:])
                rotationMatrix = self.math.rotation_matrix_from_normal(n)
                Ctemp = np.dot(constraints_local_frame, rotationMatrix.T)
            
            if constraint_mode == 'ONLY_ACTUATION':
                Ctemp, d_cone, actuation_polygons, isIKoutOfWorkSpace = self.constr.compute_actuation_constraints(j, contacts, comWF, stanceLegs, stanceIndex, swingIndex, torque_limits, trunk_mass)
                #            print d.shape[0]            
                if isIKoutOfWorkSpace is False:
                    d_cone = d_cone.reshape(6) 
            
            if constraint_mode == 'FRICTION_AND_ACTUATION':
                C1, d1, actuation_polygons, isIKoutOfWorkSpace = self.constr.compute_actuation_constraints(j, contacts, comWF, stanceLegs, stanceIndex, swingIndex, torque_limits, trunk_mass)                           
                C2, d2 = self.constr.linearized_cone_halfspaces_world(contactsNumber, ng, mu, normals)
                #            print C1, C2
                if isIKoutOfWorkSpace is False:
                    #                print d1
                    Ctemp = np.vstack([C1, C2])
                    #               print np.size(C,0), np.size(C,1), C
                    d_cone = np.hstack([d1[0], d2])
                    #                print d
                    d_cone = d_cone.reshape((6+ng))
                
            C = block_diag(C, Ctemp)
            d = np.hstack([d, d_cone])
        
        ineq = (C, d)  # C * x <= d
#        print actuation_polygons
#        print C, d
        return proj, eq, ineq, actuation_polygons, isIKoutOfWorkSpace
        
    def iterative_projection_bretl(self, iterative_projection_params, saturate_normal_force = False):

        start_t_IP = time.time()
#        print stanceLegs, contacts, normals, comWF, ng, mu, saturate_normal_force
        proj, eq, ineq, actuation_polygons, isIKoutOfWorkSpace = self.setup_iterative_projection(iterative_projection_params, saturate_normal_force)       
#        print 'hereee'  
#        points = np.random.rand(30, 2)   # 30 random points in 2-D
#        print points
        vertices_WF = pypoman.project_polytope(proj, ineq, eq, method='bretl', max_iter=1000)
#        print vertices_WF
        compressed_vertices = np.compress([True, True], vertices_WF, axis=1)
        hull = ConvexHull(compressed_vertices)
#        print 'hull ', hull.vertices
        compressed_hull = compressed_vertices[hull.vertices]
        compressed_hull = self.geom.clockwise_sort(compressed_hull)
#        print compressed_hull
        #vertices_WF = vertices_BF + np.transpose(comWF[0:2])
        computation_time = (time.time() - start_t_IP)
        print("Iterative Projection (Bretl): --- %s seconds ---" % computation_time)

        return compressed_hull, actuation_polygons, computation_time
        
        
    def instantaneous_actuation_region_bretl(self, stanceLegs, contacts, normals, trunk_mass, comWF = np.array([0.0,0.0,0.0])):
        constraint_mode = 'ONLY_ACTUATION'
        number_of_generators = 4
        mu = 1.0
        IP_points, actuation_polygons, computation_time = self.iterative_projection_bretl(constraint_mode, stanceLegs, contacts, normals, trunk_mass, number_of_generators, mu, comWF)
        IP_points = self.geom.clockwise_sort(np.array(IP_points))
        
        return IP_points, actuation_polygons, computation_time

    def support_region_bretl(self, stanceLegs, contacts, normals, trunk_mass, number_of_generators = 4, mu = 1.0, comWF = np.array([0.0,0.0,0.0])):
        constraint_mode = 'ONLY_FRICTION'
        IP_points, actuation_polygons, computation_time = self.iterative_projection_bretl(constraint_mode, stanceLegs, contacts, normals, trunk_mass, number_of_generators, mu, comWF)
        IP_points = self.geom.clockwise_sort(np.array(IP_points))
        
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
    def LP_projection(self, constraint_mode, contacts, normals, mass, friction_coeff, ng, nc, tau_lim, useVariableJacobian = False, stepX = 0.05, stepY = 0.05, stepZ = 0.05):
        start_t_LP = time.time()
        feasible_points = np.zeros((0,3))
        unfeasible_points = np.zeros((0,3))
        contact_forces = np.zeros((0,nc*3))  
        verbose = False
        com_WF = np.array([0.0, 0.0, 0.0])
        default_com_WF = com_WF
        p, G, h, A, b, isConstraintOk, LP_actuation_polygons = self.setup_lp(mass, contacts, nc, ng, normals, com_WF, constraint_mode, friction_coeff, tau_lim)
        
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
        q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = self.kin.inverse_kin(np.transpose(contactsFourLegsWF[:,0] - comWorldFrame[0]),
                                              np.transpose(foot_vel[:,0]),
                                                    np.transpose(contactsFourLegsWF[:,1] - comWorldFrame[1]),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contactsFourLegsWF[:,2] - comWorldFrame[2]),
                                                    np.transpose(foot_vel[:,2]))
        if (not isOutOfWorkspace):
            #kin.update_jacobians(q)
            J_LF, J_RF, J_LH, J_RH = self.kin.update_jacobians(q)
            #print J_LF
            G, h, isLpOK, LP_actuation_polygons = self.constr.getInequalities(constraint_mode, nc, numberOfGenerators, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH, tau_lim)
        
        else:
            LP_actuation_polygons = [None]
            isLpOK = False
            G= [None]
            h = [None]
        
        lp = p, G, h, A, b
        return p, G, h, A, b, isLpOK, LP_actuation_polygons