# -*- coding: utf-8 -*-
"""
Created on Mon May 28 13:00:59 2018

@author: rorsolino
"""
import numpy as np
from computational_geometry import ComputationalGeometry


class Constraints:
    def linear_cone(self, n, mu):
        m = np.eye(3) - np.dot(n,np.transpose(n))
        u = np.dot(n,mu)
        #cone_constraints = m - np.transpose(u)
        cone_constraints = np.vstack((m - np.transpose(u), - m - np.transpose(u)))
        known_term = np.zeros((6,1))
        return cone_constraints, known_term
    
    def zonotope(self, dx = 100, dy = 100 ,dz = 100):
        constraint = np.vstack([np.eye(3), -np.eye(3)])
        known_term = np.array([[dx],[dy],[dz],[dx],[dy],[dz]])
        return constraint, known_term
        
    def hexahedron(self, v_rep):
        geom = ComputationalGeometry()
        
        h_rep1, h_rep2, h_rep3, h_rep4, h_rep5, h_rep6 = geom.get_halfspace_rep(v_rep)        
        
        h_rep = np.vstack([h_rep1, -h_rep2, -h_rep3, -h_rep4, -h_rep5, h_rep6])        
        constraint = h_rep[:,0:3]
        known_term = np.vstack([[-h_rep1[3]],
                                [h_rep2[3]],
                                [h_rep3[3]],
                                [h_rep4[3]],
                                [h_rep5[3]],
                                [-h_rep6[3]]])
        print constraint, known_term
        return constraint, known_term
        
    def computeActuationPolygon(self, leg_jacobian_2D, tau_HAA = 80, tau_HFE = 120, tau_KFE = 120):
        """ This function computes the actuation polygon of a given mechanical chain
        This function assumes the same mechanical structure of the HyQ robot, meaning that 
        it is restricted to 3 DoFs and point contacts. If the latter assumption is not
        respected the Jacobian matrix might become not invertible.
        """
        dx = tau_HAA
        dy = tau_HFE
        dz = tau_KFE
        vertices = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
                         [dy, -dy, -dy, dy, dy, -dy, -dy, dy],
                         [dz, dz, dz, dz, -dz, -dz, -dz, -dz]])
                         
        vertices_xz = np.vstack([vertices[0,:],vertices[2,:]])
        actuation_polygon_xy = np.matmul(np.linalg.inv(np.transpose(leg_jacobian_2D)),vertices_xz) 
        actuation_polygon = np.vstack([actuation_polygon_xy[0,:],
                                   vertices[1,:],
                                   actuation_polygon_xy[1,:]])
        # Only for debugging:                          
        actuation_polygon = vertices
        return actuation_polygon