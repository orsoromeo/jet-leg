# -*- coding: utf-8 -*-
"""
Created on Mon May 28 13:00:59 2018

@author: Romeo Orsolino
"""
import numpy as np
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.leg_force_polytopes import LegForcePolytopes
from scipy.linalg import block_diag
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from jet_leg.computational_geometry.polytopes import Polytope


class FrictionConeConstraint:
    def __init__(self):
        self.max_contact_wrench = 10.0
        self.min_contact_wrench = -10.0

    def linearized_cone_halfspaces_world(self, contactsNumber, pointContacts, mu, normal, ng = 4, max_normal_force=10000.0,
                                         saturate_max_normal_force=False):

        C = np.zeros((0, 0))
        d = np.zeros((0))
        constraints_local_frame, d_cone = self.linearized_cone_halfspaces(ng, mu, max_normal_force,
                                                                          saturate_max_normal_force, pointContacts)
        n = self.math.normalize(normal)
        rotationMatrix = self.math.rotation_matrix_from_normal(n)
        Ctemp = np.dot(constraints_local_frame, rotationMatrix.T)

        return constraints_local_frame, d_cone

    def linearized_cone_vertices(self, ng, mu, cone_height=100.):
        if ng == 4:
            c_force = np.array([
                [0., 0., 0],
                [+mu * cone_height, +mu * cone_height, cone_height],
                [-mu * cone_height, +mu * cone_height, cone_height],
                [-mu * cone_height, -mu * cone_height, cone_height],
                [+mu * cone_height, -mu * cone_height, cone_height]])
        elif ng == 8:
            angle = 6.283 / 8.0 + np.arange(0, 6.283, 6.283 / 8.)
            c_force = np.array([
                [0., 0., 0.],
                [+mu * cone_height * np.cos(angle[0]), +mu * cone_height * np.sin(angle[0]), cone_height],
                [+mu * cone_height * np.cos(angle[1]), +mu * cone_height * np.sin(angle[1]), cone_height],
                [+mu * cone_height * np.cos(angle[2]), +mu * cone_height * np.sin(angle[2]), cone_height],
                [+mu * cone_height * np.cos(angle[3]), +mu * cone_height * np.sin(angle[3]), cone_height],
                [+mu * cone_height * np.cos(angle[4]), +mu * cone_height * np.sin(angle[4]), cone_height],
                [+mu * cone_height * np.cos(angle[5]), +mu * cone_height * np.sin(angle[5]), cone_height],
                [+mu * cone_height * np.cos(angle[6]), +mu * cone_height * np.sin(angle[6]), cone_height],
                [+mu * cone_height * np.cos(angle[7]), +mu * cone_height * np.sin(angle[7]), cone_height]])

        return c_force

    def linearized_cone_halfspaces(self, ng, mu, max_normal_force, saturate_max_normal_force, pointContacts):
        print "point contacts?",pointContacts
        ''' Inequality matrix for a contact force in local contact frame: '''
        if ng == 4:
            if pointContacts:
                c_force = np.array([
                    [-1, 0, -mu],
                    [+1, 0, -mu],
                    [0, -1, -mu],
                    [0, +1, -mu]])
                d = np.zeros(c_force.shape[0])
            else:
                c_force = np.array([
                    [-1, 0, -mu, 0.0, 0.0],
                    [+1, 0, -mu, 0.0, 0.0],
                    [0, -1, -mu, 0.0, 0.0],
                    [0, +1, -mu, 0.0, 0.0],
                    [0, 0, 0, +1, 0],
                    [0, 0, 0, 0, +1]])
                d = np.hstack([np.zeros(4), self.max_contact_wrench, self.max_contact_wrench, -self.min_contact_wrench, -self.min_contact_wrench])
                print "c force", c_force

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
        m = np.eye(3) - np.dot(n, np.transpose(n))
        u = np.dot(n, mu)
        # cone_constraints = m - np.transpose(u)
        cone_constraints = np.vstack((m - np.transpose(u), - m - np.transpose(u)))
        known_term = np.zeros((6, 1))
        return cone_constraints, known_term