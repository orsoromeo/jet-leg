# -*- coding: utf-8 -*-
"""
Created on Tue Dec 17 10:54:31 2019

@author: Romeo Orsolino
"""

import numpy as np
from jet_leg.dynamics.vertex_based_projection import VertexBasedProjection
from jet_leg.optimization.lp_vertex_redundancy import LpVertexRedundnacy

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
        contactsBF = fwp_params.computeContactsPosBF().T
        contactsWF = fwp_params.getContactsPosWF().T
        wrenchPolytopes = []
        stanceLegs = fwp_params.getStanceFeet()
        stanceIndex = fwp_params.getStanceIndex(stanceLegs)
        contactsNumber = np.sum(stanceLegs)
        for i in range(0, contactsNumber):
            index = int(stanceIndex[i])
            footPosWF = contactsWF[:, index]
            currentPolytope = forcePolygonsVertices[index]
            angularPart = np.zeros((3, 8))
            for j in np.arange(0, 8):
                linear = currentPolytope[:, j]
                angularPart[:, j] = np.cross(footPosWF, linear)

            sixDpoly = np.vstack([currentPolytope, angularPart])
            wrenchPolytopes.append(sixDpoly)
        return wrenchPolytopes

    def computeFeasibleWrenchPolytopeVRep(self, fwp_params, forcePolygonsVertices):

        wrenchPolytopes = self.computeAngularPart(fwp_params, forcePolygonsVertices)
        numberOfForcePolygons = np.size(wrenchPolytopes, 0)
        tmpSum = wrenchPolytopes[0]
        i = 0
        for j in np.arange(0, numberOfForcePolygons - 1):
            nextPolygon = wrenchPolytopes[j + 1]
            tmpSum = self.vProj.minksum(tmpSum, nextPolygon)

        currentPolygonSum = self.vProj.convex_hull(tmpSum)
        return currentPolygonSum