"""
Created on Tue Dec 17 10:54:31 2019

@author: Romeo Orsolino
"""
import numpy as np
from jet_leg.computational_geometry.polytopes import Polytope

class LegForcePolytopes:
    def __init__(self):
        self.forcePolytopeLF = Polytope()
        self.forcePolytopes = np.hstack([self.forcePolytopeLF,
                                         self.forcePolytopeLF,
                                         self.forcePolytopeLF,
                                         self.forcePolytopeLF])
