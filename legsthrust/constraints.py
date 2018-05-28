# -*- coding: utf-8 -*-
"""
Created on Mon May 28 13:00:59 2018

@author: rorsolino
"""
import numpy as np

class Constraints:
    def linear_cone(self, n, mu):
        m = np.eye(3) - np.dot(n,np.transpose(n))
        u = np.dot(n,mu)
        #cone_constraints = m - np.transpose(u)
        cone_constraints = np.vstack((m - np.transpose(u), - m - np.transpose(u)))
        known_term = np.zeros((6,1))
        return cone_constraints, known_term
    
    def zonotope(self):
        dx = 200
        dy = 200
        dz = 700
        constraint = np.vstack([np.eye(3), -np.eye(3)])
        print constraint
        known_term = np.array([[dx],[dy],[dz],[dx],[dy],[dz]])
        return constraint, known_term