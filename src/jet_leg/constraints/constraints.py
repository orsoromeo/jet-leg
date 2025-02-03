# -*- coding: utf-8 -*-
"""
Created on Mon May 28 13:00:59 2018

@author: Romeo Orsolino
"""
import numpy as np
from jet_leg.maths.computational_geometry import ComputationalGeometry
from jet_leg.maths.math_tools import Math
from jet_leg.kinematics.kinematics_interface import KinematicsInterface
from scipy.linalg import block_diag
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics

class Constraints:    
    def __init__(self, robot_kinematics):
        #self.robotName = robot_name
        self.kin = robot_kinematics
        self.math = Math()
        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()

    def compute_actuation_constraints(self, contactIterator, torque_limits):

        J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = self.kin.get_jacobians()
        if isOutOfWorkspace:
            C1 = np.zeros((0,0))
            d1 = np.zeros((1,0))
            actuation_polygons = 0
            print('Out of workspace IK!!!')
        else:
            jacobianMatrices = np.array([J_LF, J_RF, J_LH, J_RH])
            actuation_polygons = self.computeActuationPolygons(jacobianMatrices, torque_limits)
            ''' in the case of the IP alg. the contact force limits must be divided by the mass
            because the gravito inertial wrench is normalized'''
                
            C1 = np.zeros((0,0))
            d1 = np.zeros((1,0))

            hexahedronHalfSpaceConstraints, knownTerm = self.hexahedron(actuation_polygons[contactIterator])
            C1 = block_diag(C1, hexahedronHalfSpaceConstraints)
            d1 = np.hstack([d1, knownTerm.T])

            #print "H description: ",C1, d1
            #print C1[0,0]
            #print "theta angles: "
            #for i in range(0,6):
            #    theta = np.arctan(C1[i,2]/C1[i,0])
            #    if (C1[i,2]<0):
            #        theta+=np.pi
            #    #print theta, "[rad] ", theta/np.pi*180, "[deg]"
            #print "V description: "
            #print actuation_polygons[contactIterator]
        return C1, d1, actuation_polygons, isOutOfWorkspace
        
    def linearized_cone_halfspaces_world(self, contactsNumber, ng, mu, normals, max_normal_force = 10000.0, saturate_max_normal_force = False):            

        C = np.zeros((0,0))
        d = np.zeros((0))
        constraints_local_frame, d_cone = self.linearized_cone_halfspaces(ng, mu, max_normal_force, saturate_max_normal_force)
#        for j in range(0,contactsNumber):    
#            n = math.normalize(normals[j,:])
#            rotationMatrix = math.rotation_matrix_from_normal(n)
#            C = block_diag(C, np.dot(constraints_local_frame, rotationMatrix.T))
#            d = np.hstack([d, d_cone])
         
#        return C, d
        return constraints_local_frame, d_cone
            
    def linearized_cone_vertices(self, ng, mu, cone_height = 100.):
        if ng == 4:
            c_force = np.array([
            [0., 0., 0],
            [+mu*cone_height, +mu*cone_height, cone_height],
            [-mu*cone_height, +mu*cone_height, cone_height],
            [-mu*cone_height, -mu*cone_height, cone_height],
            [+mu*cone_height, -mu*cone_height, cone_height]])
        elif ng == 8:
            angle = 6.283/8.0+np.arange(0,6.283,6.283/8.)
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

        return c_force
        
    def linearized_cone_halfspaces(self, ng, mu, max_normal_force, saturate_max_normal_force):
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
        
        return constraint, known_term

    def computeActuationPolygons(self, legsJacobians, torque_limits):

        actuation_polygons = np.array([self.computeLegActuationPolygon(legsJacobians[0], torque_limits[0]),
                                       self.computeLegActuationPolygon(legsJacobians[1], torque_limits[1]),
                                       self.computeLegActuationPolygon(legsJacobians[2], torque_limits[2]), 
                                        self.computeLegActuationPolygon(legsJacobians[3], torque_limits[3])])
#        if np.sum(stanceFeet) == 3:
##            print 'test', torque_limits, stanceIndex
#            actuation_polygons = np.array([self.computeLegActuationPolygon(legsJacobians[int(stanceIndex[0])], torque_limits[int(stanceIndex[0])]),
#                                       self.computeLegActuationPolygon(legsJacobians[int(stanceIndex[1])], torque_limits[int(stanceIndex[1])]),
#                                       self.computeLegActuationPolygon(legsJacobians[int(stanceIndex[2])], torque_limits[int(stanceIndex[2])]), 
#                                        self.computeLegActuationPolygon(legsJacobians[int(swingIndex)], torque_limits[int(swingIndex)])])       

        return actuation_polygons
        
    
    """ 
    This function computes the actuation polygon of a given mechanical chain (i.e. one leg).
    This function assumes the same mechanical structure of the HyQ robot, meaning that 
    it is restricted to 3 DoFs and point contacts. If the latter assumption is not
    respected the Jacobian matrix might become not invertible.
    """        
    def computeLegActuationPolygon(self, leg_jacobian, tau_lim):
        dx = tau_lim[0]
        dy = tau_lim[1]
        dz = tau_lim[2]
#        vertices = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
#                         [dy, -dy, -dy, dy, dy, -dy, -dy, dy],
#                         [dz, dz, dz, dz, -dz, -dz, -dz, -dz]])
                         
        vertices = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
                         [-dy, dy, dy, -dy, -dy, dy, dy, -dy],
                         [-dz, -dz, -dz, -dz, dz, dz, dz, dz]])
                         
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
            actuation_polygon = np.matmul(np.linalg.pinv(np.transpose(leg_jacobian)),legs_gravity - torque_lims)
            
        # Only for debugging:                          
        # actuation_polygon = vertices
        return actuation_polygon
    
    def getInequalities(self, params, saturate_normal_force = False):

        stanceLegs = params.getStanceFeet()
        
        contactsNumber = np.sum(stanceLegs)
        contactsWF = params.getContactsPosWF()
        comPositionWF = params.getCoMPosWF()
        comPositionBF = params.getCoMPosBF()
        rpy = params.getOrientation()
        #compute the contacs in the base frame for the inv kineamtics
        contactsBF = np.zeros((4,3))

        for j in np.arange(0, 4):
            j = int(j)
            contactsBF[j,:]= np.add( np.dot(self.math.rpyToRot(rpy[0], rpy[1], rpy[2]), (contactsWF[j,:] - comPositionWF)), comPositionBF)
        
        constraint_mode = params.getConstraintModes()

        tau_lim = params.getTorqueLims()
        ng = params.getNumberOfFrictionConesEdges()
        friction_coeff = params.getFrictionCoefficient()
        normals = params.getNormals()
        
        actuation_polygons = np.zeros((1,1))
        C = np.zeros((0,0))
        d = np.zeros((0))

        stanceIndex = params.getStanceIndex(stanceLegs)
        #we are static so we set to zero
        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])

        self.kin.inverse_kin(contactsBF, foot_vel)

        for j in stanceIndex:
            j = int(j)
            if constraint_mode[j] == 'ONLY_FRICTION':
                #            print contactsNumber
                constraints_local_frame, d_cone = self.linearized_cone_halfspaces_world(contactsNumber, ng, friction_coeff, normals)
                isIKoutOfWorkSpace = False
                n = self.math.normalize(normals[j,:])
                rotationMatrix = self.math.rotation_matrix_from_normal(n)
                Ctemp = np.dot(constraints_local_frame, rotationMatrix.T)
            
            if constraint_mode[j] == 'ONLY_ACTUATION':
                Ctemp, d_cone, actuation_polygons, isIKoutOfWorkSpace = self.compute_actuation_constraints(j, tau_lim)
                if isIKoutOfWorkSpace is False:
                    d_cone = d_cone.reshape(6) 
                else:
                    Ctemp = np.zeros((0,0))
                    d_cone = np.zeros((0))
            
            if constraint_mode[j] == 'FRICTION_AND_ACTUATION':
                C1, d1, actuation_polygons, isIKoutOfWorkSpace = self.compute_actuation_constraints(j, tau_lim)
                C2, d2 = self.linearized_cone_halfspaces_world(contactsNumber, ng, friction_coeff, normals)
                if isIKoutOfWorkSpace is False:
                    Ctemp = np.vstack([C1, C2])
                    d_cone = np.hstack([d1[0], d2])
                    d_cone = d_cone.reshape((6+ng))
                else:
                    Ctemp = np.zeros((0,0))
                    d_cone = np.zeros((0))
                
            C = block_diag(C, Ctemp)
            d = np.hstack([d, d_cone])
        
        if contactsNumber == 0:
            print('contactsNumber is zero, there are no stance legs set! This might be because Gazebo is in pause.')
        return C, d, isIKoutOfWorkSpace, actuation_polygons
    