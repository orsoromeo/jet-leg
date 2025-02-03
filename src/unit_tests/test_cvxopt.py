# -*- coding: utf-8 -*-
"""
Created on Mon Aug  6 12:07:22 2018

@author: Romeo Orsolino
"""

from cvxopt import matrix, solvers
import unittest

class TestCvxopt(unittest.TestCase):
    
    def testLinearProgram(self):
        A = matrix([ [-1.0, -1.0, 0.0, 1.0], [1.0, -1.0, -1.0, -2.0] ])
        b = matrix([ 1.0, -2.0, 0.0, 4.0 ])
        c = matrix([ 2.0, 1.0 ])    
        sol=solvers.lp(c,A,b)    
        #print(sol['x'])
        self.assertTrue(sol)
        
    def testQuadraticProgram(self):
        Q = 2*matrix([ [2, .5], [.5, 1] ])
        p = matrix([1.0, 1.0])
        G = matrix([[-1.0,0.0],[0.0,-1.0]])
        h = matrix([0.0,0.0])
        A = matrix([1.0, 1.0], (1,2))
        b = matrix(1.0)
        sol=solvers.qp(Q, p, G, h, A, b)   
        #print(sol['x'])
        self.assertTrue(sol)
        
    def testSecondOrderConeProgram(self):
        # min c
        # subject to: ||Q||^2 <= h
        c = matrix([-2., 1., 5.])
        G = [ matrix( [[12., 13., 12.], [6., -3., -12.], [-5., -5., 6.]] ) ]
        G += [ matrix( [[3., 3., -1., 1.], [-6., -6., -9., 19.], [10., -2., -2., -3.]] ) ]     
        h = [ matrix( [-12., -3., -2.] ),  matrix( [27., 0., 3., -42.] ) ]
        sol = solvers.socp(c, Gq = G, hq = h)  
        #print(sol['x'])
        self.assertTrue(sol)
        

