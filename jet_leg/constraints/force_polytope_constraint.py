"""
Created on Mon May 28 13:00:59 2018

@author: Romeo Orsolino
"""
import numpy as np
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.computational_geometry.math_tools import Math
from scipy.spatial.transform import Rotation as Rot
from scipy.linalg import block_diag
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from jet_leg.constraints.friction_cone_constraint import FrictionConeConstraint


class ForcePolytopeConstraint:
    def __init__(self, robot_kinematics):
        # self.robotName = robot_name
        self.kin = robot_kinematics
        self.math = Math()
        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()
        self.frictionConeConstr = FrictionConeConstraint()
        self.compGeom = ComputationalGeometry()

    def compute_actuation_constraints(self, contact_iterator, torque_limits, use_contact_torque, contact_torque_lims, euler_angles):

        J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace, legIkSuccess = self.kin.get_jacobians()
        # print J_LF, J_RF, J_LH, J_RH
        #print J_LF
        #J_LF = [[0.00000000e+00, - 5.08742684e-01, - 2.92741990e-01],
        # [5.22681032e-01, - 3.53888237e-04, - 1.49860759e-02],
        # [1.56511093e-01, - 4.48279949e-03, - 1.89832737e-01]]
        #print J_LF
        if isOutOfWorkspace:
            C1 = np.zeros((0, 0))
            d1 = np.zeros((1, 0))
            current_actuation_polygon_WF = 0
            print 'Out of workspace IK!!!'
        else:
            jacobianMatrices = np.array([J_LF, J_RF, J_LH, J_RH])
            #            print 'Jacobians',jacobianMatrices
            actuation_polygons = self.computeActuationPolygons(jacobianMatrices, torque_limits)
            rot = Rot.from_euler('xyz', [euler_angles[0], euler_angles[1], euler_angles[2]], degrees=False)
            W_R_B = rot.as_dcm()
            current_actuation_polygon_WF = W_R_B.dot(actuation_polygons[contact_iterator])
            #current_actuation_polygon = actuation_polygons[contact_iterator]
            ''' in the case of the IP alg. the contact force limits must be divided by the mass
            because the gravito inertial wrench is normalized'''
            C1 = np.zeros((0, 0))
            d1 = np.zeros((1, 0))
            if not use_contact_torque:
                halfSpaceConstraints, offsetTerm = self.hexahedron(current_actuation_polygon_WF)
            else:
                hexahedronHalfSpaceConstraints, d = self.hexahedron(current_actuation_polygon_WF)
                wrench_term = np.array([[0, 0, 0, +1, 0],
                    [0, 0, 0, 0, +1],
                    [0, 0, 0, -1, 0],
                    [0, 0, 0, 0, -1]])
                force_term = np.hstack([hexahedronHalfSpaceConstraints, np.zeros((6,2))])
                halfSpaceConstraints = np.vstack([force_term, wrench_term])
                max_contact_torque = contact_torque_lims[1]
                min_contact_torque = contact_torque_lims[0]
                offsetTerm = np.vstack([d, max_contact_torque, max_contact_torque, -min_contact_torque, -min_contact_torque])

            C1 = block_diag(C1, halfSpaceConstraints)
            d1 = np.hstack([d1, offsetTerm.T])

            # print C1[0,0]
            # print "theta angles: "
            # for i in range(0,6):
            #    theta = np.arctan(C1[i,2]/C1[i,0])
            #    if (C1[i,2]<0):
            #        theta+=np.pi
            #    #print theta, "[rad] ", theta/np.pi*180, "[deg]"
            # print "V description: "
            # print actuation_polygons[contactIterator]

        return C1, d1, current_actuation_polygon_WF, isOutOfWorkspace

    def zonotope(self, dx=100, dy=100, dz=100):
        constraint = np.vstack([np.eye(3), -np.eye(3)])
        known_term = np.array([[dx], [dy], [dz], [dx], [dy], [dz]])
        return constraint, known_term

    def hexahedron(self, v_rep):

        h_rep1, h_rep2, h_rep3, h_rep4, h_rep5, h_rep6 = self.compGeom.get_hexahedron_halfspace_rep(v_rep)
        h_rep = np.vstack([h_rep1, h_rep2, h_rep3, h_rep4, h_rep5, h_rep6])

        if (h_rep[1, 3] > 0):
            h_rep = np.vstack([h_rep1, -h_rep2, -h_rep3, -h_rep4, -h_rep5, h_rep6])
            constraint = h_rep[:, 0:3]
            offset_term = np.vstack([[-h_rep1[3]],
                                    [h_rep2[3]],
                                    [h_rep3[3]],
                                    [h_rep4[3]],
                                    [h_rep5[3]],
                                    [-h_rep6[3]]])
        else:
            h_rep = np.vstack([-h_rep1, h_rep2, h_rep3, h_rep4, h_rep5, -h_rep6])
            constraint = h_rep[:, 0:3]
            offset_term = np.vstack([[h_rep1[3]],
                                    [-h_rep2[3]],
                                    [-h_rep3[3]],
                                    [-h_rep4[3]],
                                    [-h_rep5[3]],
                                    [h_rep6[3]]])

            # print constraint, known_term
        return constraint, offset_term

    def computeActuationPolygons(self, legsJacobians, torque_limits):

        #        if np.sum(stanceFeet) == 4:
        #            print 'test', torque_limits[0]
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

        if (np.size(leg_jacobian, 0) == 2):
            torque_lims_xz = np.vstack([vertices[0, :], vertices[2, :]])
            legs_gravity = np.ones(
                (2, 8)) * 0  # TODO: correct computation of the force acting on the legs due to gravity
            actuation_polygon_xy = np.matmul(np.linalg.inv(np.transpose(leg_jacobian)), legs_gravity - torque_lims_xz)
            actuation_polygon = np.vstack([actuation_polygon_xy[0, :],
                                           vertices[1, :],
                                           actuation_polygon_xy[1, :]])

        elif (np.size(leg_jacobian, 0) == 3):
            torque_lims = vertices
            legs_gravity = np.ones(
                (3, 8)) * 0  # TODO: correct computation of the force acting on the legs due to gravity
            actuation_polygon = np.matmul(np.linalg.pinv(np.transpose(leg_jacobian)), legs_gravity - torque_lims)

        # Only for debugging:
        # actuation_polygon = vertices
        return actuation_polygon