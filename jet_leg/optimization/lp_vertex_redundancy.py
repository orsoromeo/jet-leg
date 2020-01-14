"""
Created on Tue Dec 17 15:57:17 2019

@author: Romeo Orsolino

"""

import pypoman
import numpy as np
from numpy import array, dot, eye, hstack, vstack, zeros
from scipy.spatial import ConvexHull
from jet_leg.constraints.constraints import Constraints
from jet_leg.kinematics.kinematics_interface import KinematicsInterface
from jet_leg.robots.robot_model_interface import RobotModelInterface
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.geometry import Geometry
from cvxopt import matrix, solvers
import time

class LpVertexRedundnacy():

    def isPointRedundant(self, vertices, point):
        #vertices = np.array([[1.0, 1.0, -1.0,-1.0], [1.0,-1.0, 1.0,-1.0]])
        #vertices = np.array([[1.0, 1.0, -1.0,-1.0, 1.0, 1.0, -1.0,-1.0],
        #                     [1.0,-1.0, 1.0,-1.0, 1.0,-1.0, 1.0,-1.0],
        #                     [0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 2.0]])
        #point = [-0.5, -0.5, 3.0]
        p, G, h, A, b = self.setup_lp(vertices, point)
        sol = solvers.lp(p, G, h, A, b)
        if sol['status'] == 'optimal':
            flag = True
        else:
            flag = False
        return flag, sol['x']

    def setup_lp(self, polytopeVertices, point2check):
        '''     Solves a pair of primal and dual LPs

        minimize    c'*x
        subject to  G*x + s = h
                    A*x = b
                    s >= 0

        maximize    -h'*z - b'*y
        subject to  G'*z + A'*y + c = 0
                    z >= 0. '''
        
        dimensionality, numberOfVertices = np.shape(polytopeVertices) # vertices must be along the columns
        #print "num of vertices", numberOfVertices
        p = matrix(np.zeros(numberOfVertices))
        A1 = matrix(polytopeVertices)
        A2 = matrix(np.ones((1,numberOfVertices)))
        #print A2
        A = matrix(np.vstack([A1, A2]))
        b1 = matrix(point2check)
        b2 = 1.0 # sum of all the multipliers equal 1
        b = matrix(np.vstack([b1, b2]))
        G = -matrix(np.eye(numberOfVertices))
        h = matrix(np.zeros(numberOfVertices))
        #print "p", p
        #print "G", G
        #print "h", h
        #print "A", A
        #print "B", b
        lp = p, G, h, A, b
        #print "is redundant", p, G, h, A, b
        return lp
