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

# number of the equality constraints
m_eq = 6

A = hstack((G1, G2, G3))
t = hstack([-grav, zeros(3)])
eq = (A, t)  # A * x == t

# Contact surface normals
n1 = array([[0.0], [0.0], [1.0]])
n2 = array([[0.0], [0.0], [1.0]])
n3 = array([[0.0], [0.0], [1.0]])


def rotation_matrix_from_normal(n):
    n = n.reshape((3,))
    e_x = array([1., 0., 0.])
    t = e_x - dot(e_x, n) * n
    t = t / norm(t)
    b = cross(n, t)
    return vstack([t, b, n]).T


R1, R2, R3 = (rotation_matrix_from_normal(n) for n in [n1, n2, n3])

# Inequality matrix for a contact force in local contact frame:
C_force = array([
    [-1, 0, -mu],
    [+1, 0, -mu],
    [0, -1, -mu],
    [0, +1, -mu]])

# Inequality matrix for stacked contact forces in world frame:
C = block_diag(
    dot(C_force, R1.T),
    dot(C_force, R2.T),
    dot(C_force, R3.T))

ineq = (C, zeros(C.shape[0]))  # C * x <= 0

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
