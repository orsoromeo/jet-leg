# -*- coding: utf-8 -*-
"""
Created on Mon May 28 13:00:59 2018

@author: Romeo Orsolino
"""
import numpy as np
from computational_geometry import ComputationalGeometry
from math_tools import Math
from cvxopt import matrix
from scipy.linalg import block_diag

class Constraints:        
    def linearized_cone_halfspaces_world(self, contactsNumber, ng, mu, normals, max_normal_force = 10000.0, saturate_max_normal_force = False):            
        math = Math()
        C = np.zeros((0,0))
        d = np.zeros((0))
        constraints_local_frame, d_cone = self.linearized_cone_halfspaces(ng, mu, max_normal_force, saturate_max_normal_force)
        for j in range(0,contactsNumber):    
            n = math.normalize(normals[j,:])
            rotationMatrix = math.rotation_matrix_from_normal(n)
            C = block_diag(C, np.dot(constraints_local_frame, rotationMatrix.T))
            d = np.hstack([d, d_cone])
         
        
        return C, d
        
    def linearized_cone_vertices(self, ng, mu, cone_height = 100.):
        if ng == 4:
            c_force = np.array([
            [0., 0., 0],
            [+mu*cone_height, +mu*cone_height, cone_height],
            [-mu*cone_height, +mu*cone_height, cone_height],
            [-mu*cone_height, -mu*cone_height, cone_height],
            [+mu*cone_height, -mu*cone_height, cone_height]])
        elif ng == 8:
            angle = 6.283/8.+np.arange(0,6.283,6.283/8.)
            #print angle
            c_force = np.array([
            [0., 0., 0.],
            [+mu*cone_height*np.cos(angle[0]), +mu*cone_height*np.sin(angle[0]), cone_height],
            [+mu*cone_height*np.cos(angle[1]), +mu*cone_height*np.sin(angle[1]), cone_height],
            [+mu*cone_height*np.cos(angle[2]), +mu*cone_height*np.sin(angle[2]), cone_height],
            [+mu*cone_height*np.cos(angle[3]), +mu*cone_height*np.sin(angle[3]), cone_height],
            [+mu*cone_height*np.cos(angle[4]), +mu*cone_height*np.sin(angle[4]), cone_height],
            [+mu*cone_height*np.cos(angle[5]), +mu*cone_height*np.sin(angle[5]), cone_height],
            [+mu*cone_height*np.cos(angle[6]), +mu*cone_height*np.sin(angle[6]), cone_height],
            [+mu*cone_height*np.cos(angle[7]), +mu*cone_height*np.sin(angle[7]), cone_height]])
            #print c_force
        return c_force
        
    def linearized_cone_halfspaces(self, ng, mu, max_normal_force, saturate_max_normal_force):
        #print ng
        ''' Inequality matrix for a contact force in local contact frame: '''
        if ng == 4:
            c_force = np.array([
            [-1, 0, -mu],
            [+1, 0, -mu],
            [0, -1, -mu],
            [0, +1, -mu]])
            d = np.zeros(c_force.shape[0])
        elif ng == 8:
            c_force = np.array([
            [-1, 0, -mu],
            [+1, 0, -mu],
            [0.7, 0.7, -mu],
            [0.7, -0.7, -mu],
            [0, -1, -mu],
            [0, +1, -mu],
            [-0.7, -0.7, -mu],
            [-0.7, 0.7, -mu]])
            d = np.zeros(c_force.shape[0])
            
        if saturate_max_normal_force:
            c_force = np.vstack([c_force, [0, 0, 1]])
            d = np.hstack([d, max_normal_force])
            
        return c_force, d

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
        h_rep = np.vstack([h_rep1, h_rep2, h_rep3, h_rep4, h_rep5, h_rep6])
        
        if(h_rep[1,3]>0):
            h_rep = np.vstack([h_rep1, -h_rep2, -h_rep3, -h_rep4, -h_rep5, h_rep6])        
            constraint = h_rep[:,0:3]
            known_term = np.vstack([[-h_rep1[3]],
                                [h_rep2[3]],
                                [h_rep3[3]],
                                [h_rep4[3]],
                                [h_rep5[3]],
                                [-h_rep6[3]]])                
        else:
            h_rep = np.vstack([-h_rep1, h_rep2, h_rep3, h_rep4, h_rep5, -h_rep6])        
            constraint = h_rep[:,0:3]
            known_term = np.vstack([[h_rep1[3]],
                                [-h_rep2[3]],
                                [-h_rep3[3]],
                                [-h_rep4[3]],
                                [-h_rep5[3]],
                                [h_rep6[3]]])  
        
        #h_rep = np.vstack([h_rep1, -h_rep2, -h_rep3, -h_rep4, -h_rep5, h_rep6])        
        #constraint = h_rep[:,0:3]
        #known_term = np.vstack([[-h_rep1[3]],
        #                        [h_rep2[3]],
        #                        [h_rep3[3]],
        #                        [h_rep4[3]],
        #                        [h_rep5[3]],
        #                        [-h_rep6[3]]])
                                
        
        #print constraint, known_term
        return constraint, known_term

    """ 
    This function computes the actuation polygon of a given mechanical chain
    This function assumes the same mechanical structure of the HyQ robot, meaning that 
    it is restricted to 3 DoFs and point contacts. If the latter assumption is not
    respected the Jacobian matrix might become not invertible.
    """        
    def computeActuationPolygon(self, leg_jacobian, tau_HAA = 80, tau_HFE = 120, tau_KFE = 120):
        dx = tau_HAA
        dy = tau_HFE
        dz = tau_KFE
        vertices = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
                         [dy, -dy, -dy, dy, dy, -dy, -dy, dy],
                         [dz, dz, dz, dz, -dz, -dz, -dz, -dz]])
                         
        if (np.size(leg_jacobian,0)==2):          
            torque_lims_xz = np.vstack([vertices[0,:],vertices[2,:]])
            legs_gravity = np.ones((2,8))*0 # TODO: correct computation of the force acting on the legs due to gravity
            actuation_polygon_xy = np.matmul(np.linalg.inv(np.transpose(leg_jacobian)),legs_gravity - torque_lims_xz) 
            actuation_polygon = np.vstack([actuation_polygon_xy[0,:],
                                   vertices[1,:],
                                   actuation_polygon_xy[1,:]])
                                   
        elif (np.size(leg_jacobian,0)==3):
            torque_lims = vertices
            legs_gravity = np.ones((3,8))*0 # TODO: correct computation of the force acting on the legs due to gravity
            actuation_polygon = np.matmul(np.linalg.inv(np.transpose(leg_jacobian)),legs_gravity - torque_lims)        
            
        # Only for debugging:                          
        # actuation_polygon = vertices
        return actuation_polygon
        
    def getInequalities(self, constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH, saturate_normal_force = False):
        cons1 = np.zeros((0,0))
        h_vec1 = np.zeros((0,1))
        cons2 = np.zeros((0,0))
        h_vec2 = np.zeros((0,1))
        error = True
        actuation_polygon_LF = self.computeActuationPolygon(J_LF)
        actuation_polygon_RF = self.computeActuationPolygon(J_RF)
        actuation_polygon_LH = self.computeActuationPolygon(J_LH)
        actuation_polygon_RH = self.computeActuationPolygon(J_RH)
        actuation_polygons = np.array([actuation_polygon_LF,
                                       actuation_polygon_RF,
                                       actuation_polygon_LH,
                                       actuation_polygon_RH])
        #print 'actuation polygon LF: ',actuation_polygon_LF
        #print 'actuation polygon RF: ',actuation_polygon_RF
        #print 'actuation polygon LH: ',actuation_polygon_LH
        #print 'actuation polygon RH: ',actuation_polygon_RH
        
        """ construct the equations needed for the inequality constraints of the LP """   
        for j in range(0,nc):
            c, h_term = self.linear_cone(normals[j,:],friction_coeff)
            cons1 = np.vstack([np.hstack([cons1, np.zeros((np.size(cons1,0),np.size(c,1)))]),
                                          np.hstack([np.zeros((np.size(c,0),np.size(cons1,1))), c])])
            if np.isnan(h_term).any:
                error = True    
            h_vec1 = np.vstack([h_vec1, h_term])
            
            c, h_term = self.hexahedron(actuation_polygons[j])
            
            if np.isnan(h_term).any:
                error = True            
                
            cons2 = np.vstack([np.hstack([cons2, np.zeros((np.size(cons2,0),np.size(c,1)))]),
                          np.hstack([np.zeros((np.size(c,0),np.size(cons2,1))), c])])    
            h_vec2 = np.vstack([h_vec2, h_term])
        
        n1 = normals[0,:]
        n2 = normals[1,:]
        n3 = normals[2,:]
        math_lp = Math()
        n1, n2, n3 = (math_lp.normalize(n) for n in [n1, n2, n3])
        
        if constraint_mode == 'ONLY_FRICTION':
            max_normal_force = 400;
            cons, h_vec = self.linearized_cone_halfspaces_world(nc, ng, friction_coeff, normals, max_normal_force, saturate_normal_force)  
            #print cons, h_vec

        elif constraint_mode == 'ONLY_ACTUATION':
            cons = cons2
            h_vec = h_vec2
            if np.isnan(h_term).any:
                error = True    

        elif constraint_mode == 'friction_and_actuation':
            cons = np.vstack([cons1, cons2])
            h_vec = np.vstack([h_vec1, h_vec2])
        
        """Definition of the inequality constraints"""
        m_ineq = np.size(cons,0)
        G = matrix(cons) 
        h = matrix(h_vec.reshape(m_ineq))
        return G, h, error, actuation_polygons
    