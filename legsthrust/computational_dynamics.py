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
from scipy.linalg import block_diag
from plotting_tools import Plotter
from constraints import Constraints
from kinematics import Kinematics
from math_tools import Math

class ComputationalDynamics():
    def iterative_projection_bretl(self, constraint_mode, contacts, normals, mu, mass, nc = 3, ng = 8):

        g = 9.81
        grav = np.array([0., 0., -g])
        
        r1 = contacts[0,:]        
        r2 = contacts[0,:]
        r3 = contacts[0,:]
        
        math = Math()
        G1 = math.getGraspMatrix(r1)[:, 0:3]
        G2 = math.getGraspMatrix(r2)[:, 0:3]
        G3 = math.getGraspMatrix(r3)[:, 0:3]
        
        # Projection matrix
        Ex = -hstack((G1[4], G2[4], G3[4]))
        Ey = +hstack((G1[3], G2[3], G3[3]))
        E = vstack((Ex, Ey)) / (g * mass)
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
        n1 = np.transpose(normals[0,:])
        n2 = normals[1,:]
        n3 = normals[2,:]
        print n1
        n1, n2, n3 = (math.normalize(n) for n in [n1, n2, n3])
        
        R1, R2, R3 = (math.rotation_matrix_from_normal(n) for n in [n1, n2, n3])
        
        constr = Constraints()
        # Inequality matrix for stacked contact forces in world frame:
        if constraint_mode == 'only_friction':
            C_force = constr.linearized_cone_local_frame(mu, ng)
            C = block_diag(
            dot(C_force, R1.T),
            dot(C_force, R2.T),
            dot(C_force, R3.T))
            d = zeros(C.shape[0])
            
        elif constraint_mode == 'only_actuation':
            kin = Kinematics()
            foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
            q, q_dot, J_LF, J_RF, J_LH, J_RH = kin.compute_xy_IK(np.transpose(contacts[:,0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contacts[:,2]),
                                                    np.transpose(foot_vel[:,2]))
                                                    
            act_LF, act_RF, act_LH = (constr.computeActuationPolygon(J) for J in [J_LF, J_RF, J_LH])
        
            [c1, e1], [c2, e2] , [c3, e3] = (constr.hexahedron(act) for act in [act_LF, act_RF, act_LH])
            #print c1
            C = block_diag(c1, c2, c3)
            d = np.vstack([e1, e2, e3]).reshape(18)
        #print C, d
        
        ineq = (C, d)  # C * x <= d
        
        vertices = pypoman.project_polytope(proj, ineq, eq, method='bretl')
        
        pylab.ion()
        pylab.figure()
        print(vertices)
        pypoman.plot_polygon(vertices)
        
        # Project Tau_0 into CoM coordinates as in Eq. 53
        # p_g = (n/mg) x tau_0 + z_g*n
        n = array([0., 0., 1./(g)])
        points = []
        for j in range(0, len(vertices)):
            vx = vertices[j][0]
            vy = vertices[j][1]
            tau = array([vx, vy, 0.])
            p = np.cross(n, tau)
            points.append([p[0], p[1]])
        
        print points
        
        plotter = Plotter()
        plotter.plot_polygon(contacts[0:3,:])
        vertex = np.asarray(vertices)
        print vertex
        plotter.plot_polygon(vertex, '--r')
        return vertex
