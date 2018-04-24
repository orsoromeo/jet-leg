# -*- coding: utf-8 -*-
"""
Created on Wed Apr 18 14:15:45 2018

@author: Romeo Orsolino
"""

import pylab
import pypoman
import numpy as np
from numpy import array, eye, ones, vstack, zeros


def skew(v):
    if len(v) == 4: v = v[:3]/v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T

# number of contacts
nc = 4
# number of decision variables of the problem 
# (3 contact forces for each contact + 2 com coordinates x and y)
n = nc*3

# I set the com coordinates to be in the bottom of the state array:
# x = [f1_x, f1_y, f1_z, f2_x, f2_y, f2_z, ... , f4_x, f4_y, f4_z, com_x, com_y]
E = zeros((2, n))
E[0, 0] = 1.
E[1, 1] = 1.
f = zeros(2)
proj = (E, f)  # y = E * x + f

g = -7.5
mass = 1.
mu = 0.25
grav = array([[0.], [0.], [mass*g]])

# postions of the 4 contacts (I am assuming them all to be point-contacts as we normally do for HyQ)
r1 = array([1.0, 1.0, 0.])
r2 = array([1.0, -1.0, 0.])
r3 = array([-1.0, 1.0, 0.])
r4 = array([-1.0, -1.0, 0.])

# contact surface normals
n1 = array([[0.0], [0.0], [1.0]])
n2 = array([[0.0], [0.0], [1.0]])
n3 = array([[0.0], [0.0], [1.0]])
n4 = array([[0.0], [0.0], [1.0]])

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
a4 = vstack([+eye(3), skew(r4)])
A1 = np.hstack((a1, a2, a3, a4))

#A = np.hstack((A1,A2))
A = A1
t = vstack([-grav, zeros((3,1))]).reshape((6))
eq = (A, t)  # A * x == t

## Start defining the inequality constraints
m_ineq = 28
u1 = np.hstack((np.dot(mu,n1),np.dot(mu,n1),np.dot(1.0,n1)))
u2 = np.hstack((np.dot(mu,n2),np.dot(mu,n2),np.dot(1.0,n2)))
u3 = np.hstack((np.dot(mu,n3),np.dot(mu,n3),np.dot(1.0,n3)))
u4 = np.hstack((np.dot(mu,n4),np.dot(mu,n4),np.dot(1.0,n4)))
U = np.block([[np.transpose(u1), zeros((3,3)), zeros((3,3)), zeros((3,3))],
               [zeros((3,3)), np.transpose(u2), zeros((3,3)), zeros((3,3))],
                [zeros((3,3)), zeros((3,3)), np.transpose(u3), zeros((3,3))],
                 [zeros((3,3)), zeros((3,3)), zeros((3,3)), np.transpose(u4)]])
### Setting up the linear constraints
## Linearized friction cone:
b1 = eye(3)-np.dot(n1,np.transpose(n1))
b2 = eye(3)-np.dot(n2,np.transpose(n2))
b3 = eye(3)-np.dot(n3,np.transpose(n3))
b4 = eye(3)-np.dot(n4,np.transpose(n4))
#print(b1)
B = np.block([[b1, zeros((3,3)), zeros((3,3)), zeros((3,3))],
            [zeros((3,3)), b2, zeros((3,3)), zeros((3,3))],
             [zeros((3,3)), zeros((3,3)), b3, zeros((3,3))],
            [zeros((3,3)), zeros((3,3)), zeros((3,3)), b4]])
# fx <= mu*fz, fy <= mu*fz and fz > 0
C1 = - B - U
# fx >= -mu*fz and fy >= -mu*fz
C_tmp = B - U      
c2a = C_tmp[0:2,0:12]
c2b = C_tmp[3:5,0:12]
c2c = C_tmp[6:8,0:12]
c2d = C_tmp[9:11,0:12]
C2 = vstack([c2a, c2b, c2c, c2d])
# 0.75*fx + 0.75*fy <= mu*fz
C3 = array([[0.75, 0.75, -mu*1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0.75, 0.75, -mu*1., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0.75, 0.75, -mu*1., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., 0.75, 0.75, -mu*1.]])
# 0.75*fx + 0.75*fy >= -mu*fz
C4 = array([[-0.75, -0.75, -mu*1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., -0.75, -0.75, -mu*1., 0., 0., 0., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., -0.75, -0.75, -mu*1., 0., 0., 0.],
            [0., 0., 0., 0., 0., 0., 0., 0., 0., -0.75, -0.75, -mu*1.]])
# concatenate all the inequality constraints above
C = vstack([C1, C2, C3, C4])

print(C)
f_max = 10.
# vector of known coefficients
b = zeros((m_ineq,1)).reshape(m_ineq)
print(b)
ineq = (C, b)  # C * x <= b

# Solve the projection problem
vertices = pypoman.project_polytope(proj, ineq, eq, method='bretl')
pylab.ion()
pylab.figure()
print(vertices)
pypoman.plot_polygon(vertices)