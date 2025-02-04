# -*- coding: utf-8 -*-
"""
Created on Thu Oct 25 10:37:16 2018

@author: rorsolino
"""
import numpy as np

#from context import jet_leg 

import time

from pypoman.lp import solve_lp, GLPK_IF_AVAILABLE
from pypoman.bretl import Vertex
from pypoman.bretl import Polygon

import random
import cvxopt
from numpy import array, cos, cross, pi, sin
from numpy.random import random
from scipy.linalg import norm
from scipy.linalg import block_diag

from numpy import array, cross, dot, eye, hstack, vstack, zeros

import matplotlib.pyplot as plt
from jet_leg.plotting.plotting_tools import Plotter
from jet_leg.constraints.constraints import Constraints
from jet_leg.robot.hyq_kinematics import HyQKinematics
from jet_leg.maths.math_tools import Math
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.map.height_map import HeightMap
        
class PathIterativeProjection:
    def __init__(self):
        self.compDyn = ComputationalDynamics()
        
    def setup_path_iterative_projection(self, params):
        ''' parameters to be tuned'''
        g = 9.81
        ng = params.getNumberOfFrictionConesEdges();
        proj, eq, ineq, actuation_polygons, isIKoutOfWorkSpace = self.compDyn.setup_iterative_projection(params, False)

        if isIKoutOfWorkSpace:
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
#        print 'act pol ',lp
#        print 'a', actuation_polygons, trunk_mass, isIKoutOfWorkSpace
        
        return lp, actuation_polygons/params.getTotalMass(), isIKoutOfWorkSpace

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
        tempSolution = x[-2:]
        
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


    def compute_polygon_variable_constraint(self, params, max_iter=50, solver=GLPK_IF_AVAILABLE):
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
    
        total_mass = params.getTotalMass()
#        mu = params.getFrictionCoeffcient()

#        axisZ= array([[0.0], [0.0], [1.0]])
#        math = Math()
#        n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
#        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
#        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
#        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
#        # %% Cell 2
#        normals = np.vstack([n1, n2, n3, n4])
#        normals = params.getNormals()
            
        iterProj = PathIterativeProjection()
        
        lp, actuation_polygons, isOutOfWorkspace = iterProj.setup_path_iterative_projection(params)
        
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
    
    def line(self, p1, p2):
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0]*p2[1] - p2[0]*p1[1])
        return A, B, -C
    
    def two_lines_intersection(self, L1, L2):
        D  = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x,y
        else:
            return False
    
    
    def find_intersection(self, vertices_input, desired_direction, comWF):
        desired_direction = desired_direction/np.linalg.norm(desired_direction)
        #print "desired dir: ", desired_direction
        
        desired_com_line = self.line(comWF, comWF+desired_direction)
        #print "des line : ", desired_com_line
        tmp_vertices = np.vstack([vertices_input, vertices_input[0]])
        intersection_points = np.zeros((0,2))
        points = np.zeros((0,2))
        for i in range(0,len(vertices_input)):
            v1 = tmp_vertices[i]
            v2 = tmp_vertices[i+1]        
            actuation_region_edge = self.line(v1, v2)
            #print desired_com_line, actuation_region_edge
            new_point = self.two_lines_intersection(desired_com_line, actuation_region_edge)
            #print "new point ", new_point
            if new_point:
                intersection_points = np.vstack([intersection_points, new_point])
            else:
                print "lines are parallel!"
                while new_point is False:
                    desired_com_line = self.line(comWF, comWF+desired_direction+np.array([random()*0.01,random()*0.01,0.0]))
                    new_point = self.two_lines_intersection(desired_com_line, actuation_region_edge)
                    intersection_points = np.vstack([intersection_points, new_point])
#                    print new_point
                    
            #print intersection_points
            epsilon = 0.0001;
            if np.abs(desired_direction[0]- comWF[0]) > epsilon:
                alpha_com_x_line = (new_point[0] - comWF[0])/(desired_direction[0]- comWF[0])
            else:
                alpha_com_x_line = 1000000000.0;
                
            if np.abs(desired_direction[1]- comWF[1]) > epsilon:
                alpha_com_y_line = (new_point[1] - comWF[1])/(desired_direction[1]- comWF[1])
            else:
                alpha_com_y_line = 1000000000.0
                
            #print alpha_com_x_line, alpha_com_y_line
            
            if alpha_com_x_line > 0.0 and alpha_com_y_line >= 0.0:
                if np.abs(v2[0] - v1[0]) > epsilon:
                    alpha_vertices_x = (new_point[0] - v1[0])/(v2[0] - v1[0])
                else:
                    alpha_vertices_x = 0.5
                
                #print "alpha_vertices_x ", alpha_vertices_x
                if alpha_vertices_x >= 0.0 and alpha_vertices_x <= 1.0:
                    if np.abs(v2[1] - v1[1]) > epsilon:
                        alpha_vertices_y = (new_point[1] - v1[1])/(v2[1] - v1[1]) 
                    else:
                        alpha_vertices_y = 0.5
                    
                    #print "alpha vertx y ", alpha_vertices_y
                    if alpha_vertices_y >= 0.0 and alpha_vertices_y <= 1.0:                   
                        points = np.vstack([points, new_point])
                                
                elif np.abs(v2[1] - v1[1]):
                    alpha_vertices_y = (new_point[1] - v1[1])/(v2[1] - v1[1])            
                    if alpha_vertices_y >= 0.0 and alpha_vertices_y <= 1.0:                   
                        points = np.vstack([points, new_point])
                        
#        print points
        return points, intersection_points
    
    def find_vertex_along_path(self, params, desired_direction, tolerance = 0.05, max_iteration_number = 10):
        final_points = np.zeros((0,2))
        newCoM = params.getCoMPosWF()
        comWF = params.getCoMPosWF()
        contactsBF = params.getContactsPosBF()
        comToStack = np.zeros((0,3))
        stackedIncrements = np.zeros((0,3))
        increment = np.array([100.0, 100.0, 0.0])
        while_iter = 0
        polygon_to_stack = []
        
        while (np.amax(np.abs(increment))>tolerance) and (while_iter<max_iteration_number):
            comToStack = np.vstack([comToStack, newCoM])
            params.setCoMPosWF(newCoM)
            contactsBF_tmp = contactsBF - newCoM
            params.setContactsPosBF(contactsBF_tmp)
            polygon = self.compute_polygon_variable_constraint(params)
            
            if polygon:
                polygon.sort_vertices()
                vertices_list = polygon.export_vertices()
                vertices1 = [array([v.x, v.y]) for v in vertices_list]
                polygon_to_stack.append(vertices1)
                print while_iter
#                print 'poly',vertices1
                new_p, all_points = self.find_intersection(vertices1, desired_direction, comWF)
                if np.size(new_p, 0)==0:
                    while_iter+= max_iteration_number
                else:
                    final_points = np.vstack([final_points, new_p])
                    increment = np.hstack([new_p[0], 0.0]) - newCoM
                    stackedIncrements = np.vstack([stackedIncrements, increment])
                    newCoM = 0.2*increment + newCoM
                    while_iter += 1
            else:
                print "foot position is out of workspace!"
                while_iter += max_iteration_number
        
        print polygon_to_stack
        return comToStack, stackedIncrements, polygon_to_stack