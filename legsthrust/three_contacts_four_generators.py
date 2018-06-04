# -*- coding: utf-8 -*-
"""
Created on Sat May  5 14:35:48 2018

@author: Romeo Orsolino
"""

import pylab
import pypoman
import numpy as np

from numpy import array, cross, dot, eye, hstack, vstack, zeros
from numpy.linalg import norm
from scipy.linalg import block_diag

pylab.close("all")


def skew(v):
    if len(v) == 4:
        v = v[:3]/v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T


def getGraspMatrix(r):
    G = np.block([[eye(3), zeros((3, 3))],
                  [skew(r), eye(3)]])
    return G


# number of contacts
nc = 3

# number of decision variables of the problem
n = nc*6

# contact positions
r1 = array([10.0, 10.0, 0.0])
r2 = array([-10.0, -10.0, 0.0])
r3 = array([10.0, -10.0, 0.0])

g = 9.81
mass = 1.
mu = 1.
grav = array([0., 0., -g])

# Unprojected state is:
#
#     x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]

G1 = getGraspMatrix(r1)[:, 0:3]
G2 = getGraspMatrix(r2)[:, 0:3]
G3 = getGraspMatrix(r3)[:, 0:3]

# Projection matrix
Ex = hstack((G1[4], G2[4], G3[4]))
Ey = hstack((G1[3], G2[3], G3[3]))
E = vstack((Ex, Ey)) / (g * mass)
f = zeros(2)
proj = (E, f)  # y = E * x + f

# Contact surface normals
n1 = array([[0.0], [0.0], [1.0]])
n2 = array([[0.0], [0.0], [1.0]])
n3 = array([[0.0], [0.0], [1.0]])

# number of the equality constraints
m_eq = 6

A = hstack((G1, G2, G3))
t = hstack([-grav, zeros(3)])
eq = (A, t)  # A * x == t

# Definition of the inequality constraints
n_generators = 4
m_ineq = (n_generators+6+1)*nc
u1 = hstack((dot(mu, n1), dot(mu, n1), dot(1.0, n1)))
u2 = hstack((dot(mu, n2), dot(mu, n2), dot(1.0, n2)))
u3 = hstack((dot(mu, n3), dot(mu, n3), dot(1.0, n3)))
w1 = hstack((np.transpose(u1), zeros((3, 3))))
w2 = hstack((np.transpose(u2), zeros((3, 3))))
w3 = hstack((np.transpose(u3), zeros((3, 3))))
U = np.block([[w1, zeros((3, 6)), zeros((3, 6))],
              [zeros((3, 6)), w2, zeros((3, 6))],
              [zeros((3, 6)), zeros((3, 6)), w3]])
''' Setting up the linear inequality constraints
Linearized friction cone:'''
b1 = hstack((eye(3)-dot(n1, np.transpose(n1)), zeros((3, 3))))
b2 = hstack((eye(3)-dot(n2, np.transpose(n2)), zeros((3, 3))))
b3 = hstack((eye(3)-dot(n3, np.transpose(n3)), zeros((3, 3))))

B = np.block([[b1, zeros((3, 6)), zeros((3, 6))],
              [zeros((3, 6)), b2, zeros((3, 6))],
              [zeros((3, 6)), zeros((3, 6)), b3]])
''' fx <= mu*fz, fy <= mu*fz and fz > 0 '''
C1 = - B - U

''' fx >= -mu*fz and fy >= -mu*fz '''
C2 = B - U

C4a = hstack([zeros((3, 3)), eye(3), zeros((3, 12))])
C4b = hstack([zeros((3, 9)), eye(3), zeros((3, 6))])
C4c = hstack([zeros((3, 15)), eye(3)])
C4 = vstack([C4a, C4b, C4c])

c2a = C2[0:2, :]
c2b = C2[3:5, :]
c2c = C2[6:8, :]
C3 = vstack([c2a, c2b, c2c])

C = vstack([C1, C4, C3, -C4])
if m_ineq == np.size(C, 0):
    b = zeros((m_ineq, 1)).reshape(m_ineq)
else:
    print 'Warning: wrong number of inequality constraints!'
    print 'm_ineq: ', m_ineq

print ""
print "Inequality constraints:"
print C, b

ineq = (C, b)  # C * x <= b

vertices = pypoman.project_polytope(proj, ineq, eq, method='bretl')

pylab.ion()
pylab.figure()
print(vertices)
pypoman.plot_polygon(vertices)

# Project Tau_0 into CoM coordinates as in Eq. 53
# p_g = (n/mg) x tau_0 + z_g*n
n = array([0., 0., 1./(g)])
points = []
for j in range(0, len(vertices)):
    vx = vertices[j][0]
    vy = vertices[j][1]
    tau = array([vx, vy, 0.])
    p = np.cross(n, tau)
    points.append([p[0], p[1]])

print points
# pypoman.plot_polygon(points)
