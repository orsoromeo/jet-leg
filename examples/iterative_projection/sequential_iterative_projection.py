# -*- coding: utf-8 -*-
"""
Created on Thu Aug 16 14:36:48 2018

@author: romeoorsolino
"""
import numpy as np

from context import jet_leg 

import time

from pypoman.lp import solve_lp, GLPK_IF_AVAILABLE
from pypoman.bretl import Vertex
from pypoman.bretl import Polygon

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

class SequentialIterativeProjection:

    def setup_iterative_projection(self, constraint_mode, contacts, comWF, trunk_mass, mu, normals, point_contacts = True):
        ''' parameters to be tuned'''
        g = 9.81
        isOutOfWorkspace = False;
        compDyn = ComputationalDynamics()
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
            C, d = constr.linearized_cone_halfspaces_world(point_contacts, mu, normals)
            
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
                act_LF = constr.computeLegActuationPolygon(J_LF)
                act_RF = constr.computeLegActuationPolygon(J_RF)
                act_LH = constr.computeLegActuationPolygon(J_LH)
                act_RH = constr.computeLegActuationPolygon(J_RH)            
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
        
    def optimize_direction_variable_constraint(self, lp, vdir, solver=GLPK_IF_AVAILABLE):
        #print 'I am hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee'
        """
        Optimize in one direction.
    
        Parameters
        ----------
        vdir : (3,) array
            Direction in which the optimization is performed.
        lp : array tuple
            Tuple `(q, G, h, A, b)` defining the LP. See
            :func:`pypoman.lp..solve_lp` for details.
        solver : string, optional
            Backend LP solver to call.
    
        Returns
        -------
        succ : bool
            Success boolean.
        z : (3,) array, or 0
            Maximum vertex of the polygon in the direction `vdir`, or 0 in case of
            solver failure.
        """
        """ contact points """
                
        lp_q, lp_Gextended, lp_hextended, lp_A, lp_b = lp
        lp_q[-2] = -vdir[0]
        lp_q[-1] = -vdir[1]
        x = solve_lp(lp_q, lp_Gextended, lp_hextended, lp_A, lp_b, solver=solver)
        if len(x) != 0:
            tempSolution = x[-2:]
        else:
            tempSolution = x
            
        return tempSolution
        #return comWorld[0:2], errorNorm
    
    
    def optimize_angle_variable_constraint(self, lp, theta, solver=GLPK_IF_AVAILABLE):
    
        """
        Optimize in one direction.
    
        Parameters
        ----------
        theta : scalar
            Angle of the direction in which the optimization is performed.
        lp : array tuple
            Tuple `(q, G, h, A, b)` defining the LP. See
            :func:`pypoman.lp..solve_lp` for details.
        solver : string, optional
            Backend LP solver to call.
    
        Returns
        -------
        succ : bool
            Success boolean.
        z : (3,) array, or 0
            Maximum vertex of the polygon in the direction `vdir`, or 0 in case of
            solver failure.
        """
        #print "Optimize angle!!!!!!!!!!!!!!!!!!!!!!"
        d = array([cos(theta), sin(theta)])
        z = self.optimize_direction_variable_constraint(lp, d, solver=solver)
        return z
    
    
    def compute_polygon_variable_constraint(self, constraint_mode, comWorldFrame, contactsWorldFrame, max_iter=100, solver=GLPK_IF_AVAILABLE):
        """
        Expand a polygon iteratively.
    
        Parameters
        ----------
        lp : array tuple
            Tuple `(q, G, h, A, b)` defining the linear program. See
            :func:`pypoman.lp.solve_lp` for details.
        max_iter : integer, optional
            Maximum number of calls to the LP solver.
        solver : string, optional
            Name of backend LP solver.
    
        Returns
        -------
        poly : Polygon
            Output polygon.
        """
        math = Math()
        trunk_mass = 100
        mu = 0.8
    
        axisZ= array([[0.0], [0.0], [1.0]])
        
        n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2
        normals = np.vstack([n1, n2, n3, n4])
            
        lp, actuation_polygons, isOutOfWorkspace = self.setup_iterative_projection(constraint_mode, contactsWorldFrame, comWorldFrame, trunk_mass, mu, normals)
        
        if isOutOfWorkspace:
            return False
        else:
            two_pi = 2 * pi
            theta = pi * random()
            init_vertices = [self.optimize_angle_variable_constraint(lp, theta, solver)]
            step = two_pi / 3
            while len(init_vertices) < 3 and max_iter >= 0:
                theta += step
                if theta >= two_pi:
                    step *= 0.25 + 0.5 * random()
                    theta += step - two_pi
                #comWorldFrame = np.array([0.0, 0.0, 0.0])
                z = self.optimize_angle_variable_constraint(lp, theta, solver)
                #print z      
                #print init_vertices
                if len(z) != 0:
                    if all([norm(z - z0) > 1e-5 for z0 in init_vertices]):
                        init_vertices.append(z)
                max_iter -= 1
            if len(init_vertices) < 3:
                raise Exception("problem is not linearly feasible")
            v0 = Vertex(init_vertices[0])
            v1 = Vertex(init_vertices[1])
            v2 = Vertex(init_vertices[2])
            polygon = Polygon()
            polygon.from_vertices(v0, v1, v2)
            polygon.iter_expand(lp, max_iter)
            return polygon
       
    def optimize_direction_variable_constraints(self, constraint_mode, desired_direction, contacts, comWF):
        final_points = np.zeros((0,2))
        newCoM = comWF
        comToStack = np.zeros((0,3))
        increment = np.array([100.0, 100.0, 0.0])
        while_iter = 0
        math = Math()
        #print "enter while loop"
        while (np.amax(np.abs(increment))>0.01) and (while_iter<100):
            comToStack = np.vstack([comToStack, newCoM])
            polygon = self.compute_polygon_variable_constraint(constraint_mode, newCoM, contacts)
            #print polygon.export_vertices()
            if polygon:
                polygon.sort_vertices()
                vertices_list = polygon.export_vertices()
                vertices1 = [array([v.x, v.y]) for v in vertices_list]
                new_p, all_points = math.find_polygon_segment_intersection(vertices1, desired_direction, comWF)
                final_points = np.vstack([final_points, new_p])
                #print "new com: ", newCoM
                #print "new p ", new_p, np.size(new_p)
                if (np.size(new_p)==0):
                    print "intersections not found!!"
                else:
                    increment = np.stack([new_p[0], new_p[1], 0.0]) - newCoM
                
                newCoM = 0.2*increment + newCoM
                while_iter += 1
            else:
                print "foot position is out of workspace!"
                while_iter += 10
        
        return comToStack