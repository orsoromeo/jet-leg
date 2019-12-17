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
    def __init__(self):
        self.vProj = VertexBasedProjection()
        self.lp = LpVertexRedundnacy()

    def checkDynamicStability(self, FWP, w_gi):
        isPointRedundant, lambdas = self.lp.isPointRedundant(FWP, w_gi)
        # print "is point redundant? ", isPointRedundant, lambdas
        if isPointRedundant:
            isStateStable = True
        else:
            isStateStable = False
        return isStateStable

    def computeAggregatedCentroidalWrench(self, fwp_params):
        static_linear = np.array([0.0, 0.0, fwp_params.getTotalMass() * -9.81])
        com_pos_WF = fwp_params.getCoMPosWF()
        static_angular = np.cross(com_pos_WF, static_linear)
        extForce = fwp_params.externalCentroidalWrench[0:3]
        print extForce
        inertial_linear = fwp_params.getTotalMass()*fwp_params.getCoMLinAcc() + extForce
        inertial_angular = fwp_params.getCoMLinAcc() # TODO: add the inertial matrix here
        linear_aggr_wrench = inertial_linear - static_linear
        angular_aggr_wrench = static_angular + inertial_angular
        print linear_aggr_wrench
        w_gi = np.hstack([linear_aggr_wrench, angular_aggr_wrench])
        return w_gi

    def computeAngularPart(self, fwp_params, forcePolygonsVertices):
        print "compute ang part",np.shape(forcePolygonsVertices[0]),

        contactsWF = fwp_params.getContactsPosWF().T
        wrenchPolytopes = []
        stanceLegs = fwp_params.getStanceFeet()
        stanceIndex = fwp_params.getStanceIndex(stanceLegs)
        contactsNumber = np.sum(stanceLegs)
        for i in range(0, contactsNumber):
            index = int(stanceIndex[i])
            footPosWF = contactsWF[:, index]
            currentPolytope = np.array(forcePolygonsVertices[index])
            print "current poly", currentPolytope[:,0]
            dim, numOfVertices = np.shape(currentPolytope)
            angularPart = np.zeros((3, numOfVertices))
            for j in np.arange(0, 8):
                linear = currentPolytope[:, j]
                angularPart[:, j] = np.cross(footPosWF, linear)

            sixDpoly = np.vstack([currentPolytope, angularPart])
            print np.shape(sixDpoly)
            wrenchPolytopes.append(sixDpoly)

        return wrenchPolytopes

    def computedPolytopeConeIntersection(self, forcePolytopes):
        hs = forcePolytopes.getHalfspaces()
        result = []
        feasible_point = np.array([0.0, 0.0, 1.0])
        h = np.array([hs[0]])
        print np.shape(h[0])
        intersection = HalfspaceIntersection(h[0], feasible_point)
        vx1 = zip(*intersection.intersections)
        result.append(vx1)

        h = np.array([hs[1]])
        intersection = HalfspaceIntersection(h[0], feasible_point)
        vx2 = zip(*intersection.intersections)
        result.append(vx2)

        h = np.array([hs[2]])
        intersection = HalfspaceIntersection(h[0], feasible_point)
        vx3 = zip(*intersection.intersections)
        result.append(vx3)

        h = np.array([hs[3]])
        intersection = HalfspaceIntersection(h[0], feasible_point)
        vx4 = zip(*intersection.intersections)
        result.append(vx4)

        return result

    def computeFeasibleWrenchPolytopeVRep(self, fwp_params, forcePolytopes):

        forcePolygonsVertices = forcePolytopes.getVertices()
        intersectionVx = self.computedPolytopeConeIntersection(forcePolytopes)
        #intersectionVx = forcePolygonsVertices # only for debugging

        actuationWrenchPolytopesVRep = self.computeAngularPart(fwp_params, intersectionVx)

        stanceLegs = fwp_params.getStanceFeet()
        stanceIndex = fwp_params.getStanceIndex(stanceLegs)
        contactsNumber = np.sum(stanceLegs)

        polytopesInContact = []
        for i in range(0, contactsNumber):
            index = int(stanceIndex[i])
            polytopesInContact.append(actuationWrenchPolytopesVRep[index])

        tmpSum = np.array(polytopesInContact[0])
        i = 0
        for j in np.arange(0, contactsNumber - 1):
            nextPolygon = np.array(polytopesInContact[j + 1])
            #print "next", nextPolygon
            tmpSum = self.vProj.minksum(tmpSum, nextPolygon)

        currentPolygonSum = self.vProj.convex_hull(tmpSum)

        return currentPolygonSum