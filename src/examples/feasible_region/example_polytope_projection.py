# -*- coding: utf-8 -*-
"""
Created on Mon Jun  4 13:20:07 2018

@author: rorsolino
"""

import pypoman
from numpy import array, eye, ones, vstack, zeros

n = 10  # dimension of the original polytope
p = 2   # dimension of the projected polytope

# Original polytope:
# - inequality constraints: \forall i, |x_i| <= 1
# - equality constraint: sum_i x_i = 0
A = vstack([+eye(n), -eye(n)])
b = ones(2 * n)
C = ones(n).reshape((1, n))
d = array([0])
print A,b
ineq = (A, b)  # A * x <= b
eq = (C, d)    # C * x == d

# Projection is proj(x) = [x_0 x_1]
E = zeros((p, n))
E[0, 0] = 1.
E[1, 1] = 1.
f = zeros(p)
proj = (E, f)  # proj(x) = E * x + f

vertices = pypoman.project_polytope(proj, ineq, eq, method='bretl')

if __name__ == "__main__":   # plot projected polytope
    import pylab
    pylab.ion()
    pylab.figure()
    pypoman.plot_polygon(vertices)