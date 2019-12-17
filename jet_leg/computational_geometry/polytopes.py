"""
Created on Tue Dec 17 10:54:31 2019

@author: Romeo Orsolino
"""
import numpy as np

class Polytope():
    def __init__(self, hs, b, vx):
        self.vertices = vx
        print hs, b
        self.halfspaces = np.hstack([hs, b.T])

    def setVertices(self, vertices):
        self.vertices = vertices

    def setHaldSpaces(self, halfspaces, knownTerm):
        self.halfspaces = np.hstack([halfspaces, knownTerm.T])

    def getDoubleDescription(self):
        return self.vertices, self.halfspaces
