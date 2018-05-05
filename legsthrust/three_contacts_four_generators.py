# -*- coding: utf-8 -*-
"""
Created on Sat May  5 14:35:48 2018

@author: romeo
"""

import pylab
pylab.close("all")
import pypoman
import numpy as np
from numpy import array, eye, ones, vstack, zeros


def skew(v):
    if len(v) == 4: v = v[:3]/v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T

# number of contacts
nc = 3
# number of decision variables of the problem 
# (3 contact forces for each contact + 2 com coordinates x and y)
n = nc*3

# postions of the 4 contacts
r3x = 1.
r3y = -1.
r3z = 1.

r1 = array([0.0, 1.0, 1.0])
r2 = array([-1.0,-1.0, 2.0])
r3 = array([r3x+0.1, r3y-0.1, r3z+0.1])

# I set the com coordinates to be in the bottom of the state array:
# x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
E = zeros((2, n))
# tau_0x
E[0, 1] = -r1[2]
E[0, 2] = +r1[1]

E[0, 4] = -r2[2]
E[0, 5] = +r2[1]

E[0, 7] = -r3z
E[0, 8] = +r3y

#tau_0y
E[1, 0] = +r1[2]
E[1, 2] = -r1[0]

E[1, 3] = +r2[2]
E[1, 5] = -r2[0]

E[1, 6] = +r3z
E[1, 8] = -r3x

f = zeros(2)
proj = (E, f)  # y = E * x + f

g = -9.81
mass = 1.
mu = 0.15
grav = array([[0.], [0.], [mass*g]])

# contact surface normals
n1 = array([[0.0], [0.0], [1.0]])
n2 = array([[0.0], [0.0], [1.0]])
n3 = array([[0.0], [0.0], [1.0]])

# projection matrix
P = array([[1., 0., 0.],
          [0., 1., 0.]])
Pt = np.transpose(P)

## Definition of the equality constraints
m_eq = 6
A2 = vstack([zeros((3,2)), -np.dot(skew(grav),Pt)])

a1 = vstack([+eye(3), skew(r1)])
a2 = vstack([+eye(3), skew(r2)])
a3 = vstack([+eye(3), skew(r3)])

A1 = np.hstack((a1, a2, a3))

#A = np.hstack((A1,A2))
A = A1
t = vstack([-grav, zeros((3,1))]).reshape((6))
eq = (A, t)  # A * x == t

## Definition of the inequality constraints
n_generators = 4
m_ineq = (n_generators+1)*nc
u1 = np.hstack((np.dot(mu,n1),np.dot(mu,n1),np.dot(1.0,n1)))
u2 = np.hstack((np.dot(mu,n2),np.dot(mu,n2),np.dot(1.0,n2)))
u3 = np.hstack((np.dot(mu,n3),np.dot(mu,n3),np.dot(1.0,n3)))

U = np.block([[np.transpose(u1), zeros((3,3)), zeros((3,3))],
               [zeros((3,3)), np.transpose(u2), zeros((3,3))],
                [zeros((3,3)), zeros((3,3)), np.transpose(u3)]])
#print(u1)
### Setting up the linear inequality constraints
## Linearized friction cone:
b1 = eye(3)-np.dot(n1,np.transpose(n1))
b2 = eye(3)-np.dot(n2,np.transpose(n2))
b3 = eye(3)-np.dot(n3,np.transpose(n3))

#print(b1)
B = np.block([[b1, zeros((3,3)), zeros((3,3))],
            [zeros((3,3)), b2, zeros((3,3))],
             [zeros((3,3)), zeros((3,3)), b3]])
# fx <= mu*fz, fy <= mu*fz and fz > 0
C1 = - B - U
# fx >= -mu*fz and fy >= -mu*fz
C2 = B - U
c2a = C2[0:2,0:3*nc]
c2b = C2[3:5,0:3*nc]
c2c = C2[6:8,0:3*nc]
#c2d = C2[9:11,0:12]
C3 = vstack([c2a, c2b, c2c])
C = vstack([C1, C3])
print ""
print "constraints:"
print(C)

b = zeros((m_ineq,1)).reshape(m_ineq)
print(b)
ineq = (C, b)  # C * x <= b

vertices = pypoman.project_polytope(proj, ineq, eq, method='bretl')
pylab.ion()
pylab.figure()
print(vertices)
#pypoman.plot_polygon(vertices)

# Project Tau_0 into CoM coordinates as in Eq. 53
# p_g = (n/mg) x tau_0 + z_g*n
n = array([0., 0., 1./(mass*g)])
v1, v2, v3 = vertices
tau_1 = array([v1[0], v1[1], 0.])
tau_2 = array([v2[0], v2[1], 0.])
tau_3 = array([v3[0], v3[1], 0.])

p1 = np.cross(n,tau_1)
p2 = np.cross(n,tau_2)
p3 = np.cross(n,tau_3)
#print p1, p2, p3
points = array([[p1[0], p1[1]],
                [p2[0], p2[1]],
                [p3[0], p3[1]]])
print points
pypoman.plot_polygon(points)