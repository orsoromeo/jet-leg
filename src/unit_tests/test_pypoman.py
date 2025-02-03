# -*- coding: utf-8 -*-
"""
Created on Mon Aug  6 11:52:42 2018

@author: romeoorsolino
"""

import numpy
import pypoman
import unittest

#class TestPypoman(unittest.TestCase):
#    def testPolytopeProjection(self):
A = numpy.array([
[-1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
[0, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
[0,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0],
[0,  0,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0],
[0,  0,  0,  0, -1,  0,  0,  0,  0,  0,  0,  0],
[0,  0,  0,  0,  0, -1,  0,  0,  0,  0,  0,  0],
[0,  0,  0,  0,  0,  0, -1,  0,  0,  0,  0,  0],
[0,  0,  0,  0,  0,  0,  0, -1,  0,  0,  0,  0],
[0,  0,  0,  0,  0,  0,  0,  0, -1,  0,  0,  0],
[0,  0,  0,  0,  0,  0,  0,  0,  0, -1,  0,  0],
[0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1,  0],
[0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1],
[1,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0],
[0,  0,  0,  1,  1,  1,  0,  0,  0,  0,  0,  0],
[0,  0,  0,  0,  0,  0,  1,  1,  1,  0,  0,  0],
[0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1],
[1,  0,  0,  1,  0,  0,  1,  0,  0,  1,  0,  0],
[0,  1,  0,  0,  1,  0,  0,  1,  0,  0,  1,  0],
[0,  0,  1,  0,  0,  1,  0,  0,  1,  0,  0,  1]])
b = numpy.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 2, 2, 1, 2, 3])
vertices = pypoman.compute_polytope_vertices(A, b)
print vertices
#unitTest = unittest()
#unitTest.assertAlmostEquals(388, numpy.size(vertices,0))self
    