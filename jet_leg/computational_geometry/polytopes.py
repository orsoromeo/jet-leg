"""
Created on Tue Dec 17 10:54:31 2019

@author: Romeo Orsolino
"""
import numpy as np
from cvxopt import matrix

class Polytope():
    def __init__(self):
        self.vertices = []
        self.halfspaces = []

    def setVertices(self, vertices):
        self.vertices = vertices

    def setHalfSpaces(self, halfspaces, knownTerm):
        self.halfspaces = np.hstack([halfspaces, - matrix(knownTerm)])

    def getDoubleDescription(self):
        return self.vertices, self.halfspaces

    def getVertices(self):
        return self.vertices

    def getHalfspaces(self):
        return self.halfspaces