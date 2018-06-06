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
    def getGraspMatrix(self, r):
        math = Math()
        G = np.block([[eye(3), zeros((3, 3))],
                  [math.skew(r), eye(3)]])
        return G
    
    def compute_bretl(self, constraint_mode, contacts, normals, mass, ng, mu):
        g = 9.81
        grav = array([0., 0., -g])
        # Unprojected state is:
        #
        #     x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
        math = Math()
        LF_foot = np.array([0.3, 0.2, -.5])
        RF_foot = np.array([0.3, -0.2, -.5])
        LH_foot = np.array([-0.3, 0.2, -.5])
        G1 = self.getGraspMatrix(LF_foot)[:, 0:3]
        G2 = self.getGraspMatrix(RF_foot)[:, 0:3]
        G3 = self.getGraspMatrix(LH_foot)[:, 0:3]
        
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
        print A,t
        eq = (A, t)  # A * x == t
        
        # Contact surface normals
        
        n1 = array([0.0, 0.0, 1.0])
        n2 = array([0.0, 0.0, 1.0])
        n3 = array([0.0, 0.0, 1.0])
        n1, n2, n3 = (math.normalize(n) for n in [n1, n2, n3])
        print n1, n2, n3
        R1, R2, R3 = (math.rotation_matrix_from_normal(n) for n in [n1, n2, n3])
        
        # Inequality matrix for a contact force in local contact frame:
        if ng == 4:
            C_force = array([
            [-1, 0, -mu],
            [+1, 0, -mu],
            [0, -1, -mu],
            [0, +1, -mu]])
        elif ng ==8:
            C_force = array([
            [-1, 0, -mu],
            [+1, 0, -mu],
            [0.7, 0.7, -mu],
            [0.7, -0.7, -mu],
            [0, -1, -mu],
            [0, +1, -mu],
            [-0.7, -0.7, -mu],
            [-0.7, 0.7, -mu]])    
        # Inequality matrix for stacked contact forces in world frame:
        if constraint_mode == 'only_friction':
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
            constr = Constraints()
            act_LF = constr.computeActuationPolygon(J_LF)
            act_LH = constr.computeActuationPolygon(J_LH)
            act_RF = constr.computeActuationPolygon(J_LF)
            c1, e1 = constr.hexahedron(act_LF)
            c2, e2 = constr.hexahedron(act_LH)
            c3, e3 = constr.hexahedron(act_RF)
            C = block_diag(c1, c2, c3)
            d = np.vstack([e1, e2, e3]).reshape(18)
        print C, d
        
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
