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
nc = 1
# number of decision variables of the problem 
# (3 contact forces for each contact + 2 com coordinates x and y)
n = 3

# I set the com coordinates to be in the bottom of the state array:
# x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z, com_x, com_y]
E = zeros((2, n))
E[0, n-2] = 1.
E[1, n-1] = 1.
f = zeros(2)
proj = (E, f)  # y = E * x + f

g = -9.81
mass = 10
mu = 0.5
grav = array([[0.], [0.], [mass*g]])

# postion of the contact. 
r1 = array([0.0, 5.0, 0.0])

# contact surface normal
n1 = array([[0.0], [0.0], [1.0]])

# projection matrix
P = array([[1., 0., 0.],
          [0., 1., 0.]])
Pt = np.transpose(P)

## Definition of the equality constraints
m_eq = 6
#grav_skew = skew(grav);
#A2 = vstack([zeros((3,2)), -np.dot(grav_skew,Pt)])

a1 = vstack([+eye(3), skew(r1)])

#A1 = np.hstack((a1, a2, a3))

C = a1#np.hstack((a1,A2))

d = vstack([-grav, zeros((3,1))]).reshape((6))
C = zeros((6,6))
d = zeros((6,1)).reshape((6))
#print(C)
#print(d)

eq = (C, d)  # C * x == d

## Definition of the inequality constraints
n1_t = np.transpose(n1)

C = +eye(3) - np.dot(n1, n1_t)
#print(C)
C = C[0:2,0:3]
#print(C)
u = mu*n1
#print(u)
U = vstack([np.transpose(u),
            np.transpose(u)])
#print(U)
c1 = C - U
#print(C)
c2 = C + U
#d1 = np.block([[c1, zeros((3,2))],
#              [zeros((2,3)), +eye(2)]])
#d2 = np.block([[c2, zeros((3,2))],
#              [zeros((2,3)), -eye(2)]])

A1 = vstack([c1, c2])
A = np.block([[A1, zeros((4,3))],
               [zeros((4,3)), A1]])
#b1 = vstack([zeros((3,1)), +1000.*ones((2,1))])
#b2 = vstack([zeros((3,1)), +1000.*ones((2,1))])
b = zeros((8,1)).reshape((8))
print(A)
print(b)
ineq = (A, b)  # A * x <= b

vertices = pypoman.project_polytope(proj, ineq, eq, method='bretl')
pylab.ion()
pylab.figure()
print(vertices)
pypoman.plot_polygon(vertices)