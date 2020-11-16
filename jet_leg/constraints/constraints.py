# -*- coding: utf-8 -*-
"""
Created on Mon May 28 13:00:59 2018

@author: Romeo Orsolino
"""
import numpy as np
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.leg_force_polytopes import LegForcePolytopes
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation as Rot
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from jet_leg.computational_geometry.polytopes import Polytope
from jet_leg.constraints.friction_cone_constraint import FrictionConeConstraint
from jet_leg.constraints.force_polytope_constraint import ForcePolytopeConstraint
import copy

class Constraints:    
    def __init__(self, robot_kinematics, robot_model):
        #self.robotName = robot_name
        self.kin = robot_kinematics
        self.math = Math()
        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()
        self.frictionConeConstr = FrictionConeConstraint()
        self.forcePolytopeConstr = ForcePolytopeConstraint(robot_kinematics)
        self.model = robot_model
        self.currentLegForcePolytope = Polytope()

    def getInequalities(self, params, saturate_normal_force = False):

        stanceLegs = params.getStanceFeet()
        rpy = params. getOrientation()
        contactsNumber = np.sum(stanceLegs)

        contactsBF = copy.copy(params.computeContactsPosBF())
        #print "contacts BF ", contactsBF
        #print 'stance legs ', stanceLegs
        
        constraint_mode = params.getConstraintModes()

        joint_torque_lims = self.model.joint_torque_limits #= params.getTorqueLims()
        contact_torque_lims = self.model.contact_torque_limits
        ng = params.getNumberOfFrictionConesEdges()
        friction_coeff = params.getFrictionCoefficient()
        normals = params.getNormals()

        C = np.zeros((0,0))
        d = np.zeros((0))

        stanceIndex = params.getStanceIndex(stanceLegs)
        #print "stance ID", stanceIndex
        #we are static so we set to zero
        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])

        self.kin.inverse_kin(contactsBF, foot_vel, stanceIndex)
        #print ("q is ",q)

        forcePolytopes = LegForcePolytopes()
        leg_actuation_polygon = np.zeros((4,3,8))
        for j in stanceIndex:
            j = int(j)
            if constraint_mode[j] == 'ONLY_FRICTION':
                #            print contactsNumber
                Ctemp, d_cone = self.frictionConeConstr.linearized_cone_halfspaces_world(params.useContactTorque, friction_coeff, normals[j,:], contact_torque_lims)
                isIKoutOfWorkSpace = False
                leg_actuation_polygon[j] = np.zeros((3, 8))
#                n = self.math.normalize(normals[j,:])
#                rotationMatrix = self.math.rotation_matrix_from_normal(n)
#                Ctemp = np.dot(constraints_local_frame, rotationMatrix.T)
            
            if constraint_mode[j] == 'ONLY_ACTUATION':
                Ctemp, d_cone, leg_polygon, isIKoutOfWorkSpace = self.forcePolytopeConstr.compute_actuation_constraints(j, joint_torque_lims, params.useContactTorque, contact_torque_lims)
                leg_actuation_polygon[j] = leg_polygon
                if isIKoutOfWorkSpace is False:
                    if params.useContactTorque:
                        d_cone = d_cone.reshape(10)
                    else:
                        d_cone = d_cone.reshape(6)
                else:
                    Ctemp = np.zeros((0,0))
                    d_cone = np.zeros((0))
            
            if constraint_mode[j] == 'FRICTION_AND_ACTUATION':
                C1, d1, leg_polygon, isIKoutOfWorkSpace = self.forcePolytopeConstr.compute_actuation_constraints(j, joint_torque_lims, params.useContactTorque, contact_torque_lims, rpy)
                leg_actuation_polygon[j] = leg_polygon
                C2, d2 = self.frictionConeConstr.linearized_cone_halfspaces_world(params.useContactTorque, friction_coeff, normals[j,:], contact_torque_lims)

                if isIKoutOfWorkSpace is False:
                    #                print d1
                    Ctemp = np.vstack([C1, C2])
                    #               print np.size(C,0), np.size(C,1), C
                    d_cone = np.hstack([d1[0], d2])
                    #                print d
                    if params.useContactTorque:
                        d_cone = d_cone.reshape((10 + ng + 4))
                    else:
                        d_cone = d_cone.reshape((6+ng))

                else:
                    Ctemp = np.zeros((0,0))
                    d_cone = np.zeros((0))

            #print "ik out of ws", isIKoutOfWorkSpace
            if isIKoutOfWorkSpace is False:
                self.currentLegForcePolytope.setHalfSpaces(Ctemp, d_cone)
                #print "leg_actuation_polygon[j] ", leg_actuation_polygon[j]
                self.currentLegForcePolytope.setVertices(leg_actuation_polygon[j])
                #print "stance idx", j
                #print "currentLegForcePolytope ", self.currentLegForcePolytope.getVertices()
                forcePolytopes.forcePolytope[j].setVertices(leg_actuation_polygon[j])
                
            C = block_diag(C, Ctemp)
            d = np.hstack([d, d_cone])
        
        if contactsNumber == 0:
            print 'contactsNumber is zero, there are no stance legs set! This might be because Gazebo is in pause.'
            
        return C, d, isIKoutOfWorkSpace, forcePolytopes
    