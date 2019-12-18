# -*- coding: utf-8 -*-
"""
Created on Tue Dec 17 10:54:31 2019

@author: Romeo Orsolino
"""

import numpy as np
from jet_leg.dynamics.vertex_based_projection import VertexBasedProjection
from jet_leg.optimization.lp_vertex_redundancy import LpVertexRedundnacy
from scipy.spatial import HalfspaceIntersection
from cvxopt import matrix

class FeasibleWrenchPolytope():
    def __init__(self, params):
        self.vProj = VertexBasedProjection()
        self.lp = LpVertexRedundnacy()
        self.constraint_modes = params.getConstraintModes()

    def checkDynamicStability(self, FWP, w_gi):
        isPointRedundant, lambdas = self.lp.isPointRedundant(FWP, w_gi)
        # print "is point redundant? ", isPointRedundant, lambdas
        if isPointRedundant:
            isStateStable = True
        else:
            isStateStable = False
        return isStateStable

    #def computeAggregatedCentroidalWrench(self, fwp_params):
    #    static_linear = np.array([0.0, 0.0, fwp_params.getTotalMass() * -9.81])
    #    com_pos_WF = fwp_params.getCoMPosWF()
    #    static_angular = np.cross(com_pos_WF, static_linear)
    #    extForce = fwp_params.externalCentroidalWrench[0:3]
    #    inertial_linear = fwp_params.getTotalMass()*fwp_params.getCoMLinAcc() + extForce
    #    inertial_angular = fwp_params.getCoMLinAcc() # TODO: add the inertial matrix here
    #    linear_aggr_wrench = inertial_linear - static_linear
    #    angular_aggr_wrench = static_angular + inertial_angular
    #    print linear_aggr_wrench
    #    w_gi = np.hstack([linear_aggr_wrench, angular_aggr_wrench])
    #    return w_gi

    def computeAngularPart(self, fwp_params, forcePolygonsVertices):

        contactsWF = fwp_params.getContactsPosWF().T
        wrenchPolytopes = []
        stanceLegs = fwp_params.getStanceFeet()
        stanceIndex = fwp_params.getStanceIndex(stanceLegs)
        print stanceIndex
        contactsNumber = np.sum(stanceLegs)
        for i in range(0, contactsNumber):
            index = int(stanceIndex[i])
            print index
            footPosWF = contactsWF[:, index]
            currentPolytope = np.array(forcePolygonsVertices[i])
            dim, numOfVertices = np.shape(currentPolytope)
            angularPart = np.zeros((3, numOfVertices))
            for j in np.arange(0, numOfVertices):
                linear = currentPolytope[:, j]
                angularPart[:, j] = np.cross(footPosWF, linear)

            sixDpoly = np.vstack([currentPolytope, angularPart])
            wrenchPolytopes.append(sixDpoly)

        return wrenchPolytopes

    def computedPolytopeConeIntersection(self, fwp_params, forcePolytopes):
        hs = forcePolytopes.getHalfspaces()
        result = []
        feasible_point = np.array([0.0, 0.0, 1.0])

        stanceLegs = fwp_params.getStanceFeet()
        stanceIndex = fwp_params.getStanceIndex(stanceLegs)
        contactsNumber = np.sum(stanceLegs)
        for i in range(0, contactsNumber):
            index = int(stanceIndex[i])
            h = np.array([hs[index]])
            intersection = HalfspaceIntersection(h[0], feasible_point)
            vx = zip(*intersection.intersections)
            result.append(vx)

        return result

    def get3DforcePolytopeVertices(self, params, forcePolytopes):
        '''
        possible constraints for each foot:
         ONLY_ACTUATION = only joint-torque limits are enforces
         ONLY_FRICTION = only friction cone constraints are enforced
         FRICTION_AND_ACTUATION = both friction cone constraints and joint-torque limits
        '''
        if self.constraint_modes[0] == 'FRICTION_AND_ACTUATION':
            print 'FRICTION_AND_ACTUATION'
            forcePolygonsVertices = self.computedPolytopeConeIntersection(params, forcePolytopes)
        elif self.constraint_modes[0] == 'ONLY_ACTUATION':
            print 'ONLY_ACTUATION'
            forcePolygonsVertices = forcePolytopes.getVertices()
        elif self.constraint_modes[0] == 'ONLY_FRICTION':
            forcePolygonsVertices = self.getPiramidsVertices()

        return forcePolygonsVertices

    def computeFeasibleWrenchPolytopeVRep(self, fwp_params, forcePolytopes):

        intersectionVx = self.get3DforcePolytopeVertices(fwp_params, forcePolytopes)
        #forcePolygonsVertices = forcePolytopes.getVertices()
        #intersectionVx = self.computedPolytopeConeIntersection(fwp_params, forcePolytopes)


        actuationWrenchPolytopesVRep = self.computeAngularPart(fwp_params, intersectionVx)
        stanceLegs = fwp_params.getStanceFeet()
        contactsNumber = np.sum(stanceLegs)
        polytopesInContact = []
        for i in range(0, contactsNumber):
            polytopesInContact.append(actuationWrenchPolytopesVRep[i])

        tmpSum = np.array(polytopesInContact[0])
        for j in np.arange(0, contactsNumber - 1):
            nextPolygon = np.array(polytopesInContact[j + 1])
            print np.shape(nextPolygon)
            tmpSum = self.vProj.minksum(tmpSum, nextPolygon)

        print np.shape(tmpSum)
        currentPolygonSum = self.vProj.convex_hull(tmpSum)
        print np.shape(currentPolygonSum)

        return currentPolygonSum