# -*- coding: utf-8 -*-
"""
Created on Wed Apr 18 14:15:45 2018

@author: Romeo Orsolino
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

def getGraspMatrix(r):
    G = np.block([[np.eye(3), zeros((3,3))],
                   [skew(r), np.eye(3)]])
    return G

# number of contacts
nc = 4
# number of decision variables of the problem 
# (3 contact forces for each contact + 2 com coordinates x and y)
n = nc*3

use_grasp_matrix = True

# postions of the 4 contacts
r1 = array([10.0, 10.0, 0.0])
r2 = array([10.0, -10.0, 0.0])
r3 = array([-10.0, -10.0, 0.0])
r4 = array([-10.0, 10.0, 0.0])

std_dev = np.sqrt(r1[0]*r1[0]+r1[1]*r1[1])/1000
print ""
print "standard dev:"
print std_dev
noise = np.random.normal(0,std_dev,16)
# I set the com coordinates to be in the bottom of the state array:
# x = [f1_x, f1_y, f1_z, f2_x, f2_y, f2_z, ... , f4_x, f4_y, f4_z, com_x, com_y]

if use_grasp_matrix == 'True':
    G1 = getGraspMatrix(r1)
    G2 = getGraspMatrix(r2)
    G3 = getGraspMatrix(r3)
    G4 = getGraspMatrix(r4)
    Ex = np.hstack((G1[4,0:3],G2[4,0:3],G3[4,0:3],G3[4,0:3])) 
    Ey = np.hstack((G1[3,0:3],G2[3,0:3],G3[3,0:3],G3[3,0:3]))
    E = vstack((Ex, Ey))
else:
    E = zeros((2, nc*3))
    # tau_0x
    E[0, 1] = -r1[2]+noise[0]
    E[0, 2] = +r1[1]+noise[1]
    
    E[0, 4] = -r2[2]+noise[2]
    E[0, 5] = +r2[1]+noise[3]
    
    E[0, 7] = -r3[2]+noise[4]
    E[0, 8] = +r3[1]+noise[5]
    
    E[0, 10] = -r4[2]+noise[6]
    E[0, 11] = +r4[1]+noise[7]
    
    #tau_0y
    E[1, 0] = +r1[2]+noise[8]
    E[1, 2] = -r1[0]+noise[9]
    
    E[1, 3] = +r2[2]+noise[10]
    E[1, 5] = -r2[0]+noise[11]
    
    E[1, 6] = +r3[2]+noise[12]
    E[1, 8] = -r3[0]+noise[13]
    
    E[1, 9] = +r4[2]+noise[14]
    E[1, 11] = -r4[0]+noise[15]

f = zeros(2)
proj = (E, f)  # y = E * x + f

g = -9.81
mass = 10.
mu = 1.
grav = array([[0.], [0.], [mass*g]])

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
print A, t
eq = (A, t)  # A * x == t

## Definition of the inequality constraints
n_generators = 4
m_ineq = (n_generators+1)*nc
u1 = np.hstack((np.dot(mu,n1),np.dot(mu,n1),np.dot(1.0,n1)))
u2 = np.hstack((np.dot(mu,n2),np.dot(mu,n2),np.dot(1.0,n2)))
u3 = np.hstack((np.dot(mu,n3),np.dot(mu,n3),np.dot(1.0,n3)))
u4 = np.hstack((np.dot(mu,n4),np.dot(mu,n4),np.dot(1.0,n4)))
U = np.block([[np.transpose(u1), zeros((3,3)), zeros((3,3)), zeros((3,3))],
               [zeros((3,3)), np.transpose(u2), zeros((3,3)), zeros((3,3))],
                [zeros((3,3)), zeros((3,3)), np.transpose(u3), zeros((3,3))],
                 [zeros((3,3)), zeros((3,3)), zeros((3,3)), np.transpose(u4)]])
#print(u1)
### Setting up the linear inequality constraints
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
C2 = B - U
c2a = C2[0:2,0:12]
c2b = C2[3:5,0:12]
c2c = C2[6:8,0:12]
c2d = C2[9:11,0:12]
C3 = vstack([c2a, c2b, c2c, c2d])
C = vstack([C1, C3])
print(C)

b = zeros((m_ineq,1)).reshape(m_ineq)
print(b)
ineq = (C, b)  # C * x <= b

vertices = pypoman.project_polytope(proj, ineq, eq, method='bretl')
pylab.ion()
pylab.figure()
print(vertices)
pypoman.plot_polygon(vertices)

# Project Tau_0 into CoM coordinates as in Eq. 53
# p_g = (n/mg) x tau_0 + z_g*n
n = array([0., 0., 1./(mass*g)])
points = []
for j in range(0,len(vertices)):
    vx = vertices[j][0]
    vy = vertices[j][1]
    tau = array([vx, vy, 0.])
    p = np.cross(n,tau)
    points.append([p[0],p[1]])
    #print points

print points
pypoman.plot_polygon(points)