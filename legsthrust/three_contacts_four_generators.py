# -*- coding: utf-8 -*-
"""
Created on Sat May  5 14:35:48 2018

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
nc = 3
# number of decision variables of the problem 
# (3 contact forces for each contact + 2 com coordinates x and y)
n = nc*6

# postions of the 4 contacts
r1 = array([10.0, 10.0, 0.0])
r2 = array([-10.0,-10.0, 0.0])
r3 = array([10.0, -10.0, 0.0])
print ""
print "grasp matrix:"
print getGraspMatrix(r1)

g = 9.81
mass = 1.
contact_wrench = 'with_torque'

mu = 1.
grav = array([[0.], [0.], [-g]])

std_dev = np.sqrt(r1[0]*r1[0]+r1[1]*r1[1])/1000.
print ""
print "standard dev:"
print std_dev
noise = np.random.normal(0,std_dev,18)
print ""
print "noise:"
print noise
# I set the com coordinates to be in the bottom of the state array:
# x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
#E = zeros((2, n))
# tau_0x

G1 = getGraspMatrix(r1)
G2 = getGraspMatrix(r2)
G3 = getGraspMatrix(r3)
#print G1
if contact_wrench == 'with_torque':
    Ex = np.hstack((G1[4,:],G2[4,:],G3[4,:])) 
    Ey = np.hstack((G1[3,:],G2[3,:],G3[3,:]))
elif contact_wrench == 'pure_force':
    Ex = np.hstack((G1[4,0:3],G2[4,0:3],G3[4,0:3])) 
    Ey = np.hstack((G1[3,0:3],G2[3,0:3],G3[3,0:3]))
    
E = vstack((Ex, Ey))/(g*mass)

#E[0, 1] = -r1[2]/(mass*g)+noise[0]
#E[0, 2] = +r1[1]/(mass*g)+noise[1]

#E[0, 4] = -r2[2]/(mass*g)+noise[2]
#E[0, 5] = +r2[1]/(mass*g)+noise[3]

#E[0, 7] = -r3[2]/(mass*g)+noise[4]
#E[0, 8] = +r3[1]/(mass*g)+noise[5]

#tau_0y
#E[1, 0] = +r1[2]/(mass*g)+noise[6]
#E[1, 2] = -r1[0]/(mass*g)+noise[7]

#E[1, 3] = +r2[2]/(mass*g)+noise[8]
#E[1, 5] = -r2[0]/(mass*g)+noise[9]

#E[1, 6] = +r3[2]/(mass*g)+noise[10]
#E[1, 8] = -r3[0]/(mass*g)+noise[11]

print "E matrix:"
print E
f = zeros(2)
proj = (E, f)  # y = E * x + f

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
#A2 = vstack([zeros((3,2)), -np.dot(skew(grav),Pt)])

#a1 = vstack([+eye(3), skew(r1)])
#a2 = vstack([+eye(3), skew(r2)])
#a3 = vstack([+eye(3), skew(r3)])

A = np.hstack((G1,G2,G3))

#A = np.hstack((A1,A2))
print ""
print "Equality constraints:"

t = vstack([-grav, zeros((3,1))]).reshape((6))
print A, t
eq = (A, t)  # A * x == t

## Definition of the inequality constraints
n_generators = 4
m_ineq = (n_generators+6+1)*nc
u1 = np.hstack((np.dot(mu,n1),np.dot(mu,n1),np.dot(1.0,n1)))
u2 = np.hstack((np.dot(mu,n2),np.dot(mu,n2),np.dot(1.0,n2)))
u3 = np.hstack((np.dot(mu,n3),np.dot(mu,n3),np.dot(1.0,n3)))
w1 = np.hstack((np.transpose(u1), zeros((3,3))))
w2 = np.hstack((np.transpose(u2), zeros((3,3))))
w3 = np.hstack((np.transpose(u3), zeros((3,3))))
U = np.block([[w1, zeros((3,6)), zeros((3,6))],
               [zeros((3,6)), w2, zeros((3,6))],
                [zeros((3,6)), zeros((3,6)), w3]])
#print(u1)
''' Setting up the linear inequality constraints
Linearized friction cone:'''
b1 = np.hstack((eye(3)-np.dot(n1,np.transpose(n1)), zeros((3,3))))
b2 = np.hstack((eye(3)-np.dot(n2,np.transpose(n2)), zeros((3,3))))
b3 = np.hstack((eye(3)-np.dot(n3,np.transpose(n3)), zeros((3,3))))

#print(b1)
B = np.block([[b1, zeros((3,6)), zeros((3,6))],
            [zeros((3,6)), b2, zeros((3,6))],
             [zeros((3,6)), zeros((3,6)), b3]])
#print B
''' fx <= mu*fz, fy <= mu*fz and fz > 0 '''
C1 = - B - U

''' fx >= -mu*fz and fy >= -mu*fz '''
C2 = B - U

C4a = np.hstack([np.zeros((3,3)),np.eye(3),np.zeros((3,12))])
C4b = np.hstack([np.zeros((3,9)),np.eye(3),np.zeros((3,6))])
C4c = np.hstack([np.zeros((3,15)),np.eye(3)])
C4 = np.vstack([C4a, C4b, C4c])

c2a = C2[0:2,:]
c2b = C2[3:5,:]
c2c = C2[6:8,:]
#c2d = C2[9:11,0:12]
C3 = vstack([c2a, c2b, c2c])

C = vstack([C1, C4, C3, -C4])
if m_ineq == np.size(C,0):
    if contact_wrench == 'pure_force':
        b = zeros((m_ineq,1)).reshape(m_ineq)
    elif contact_wrench == 'with_torque':
        b = np.vstack([np.zeros((9,1)),np.ones((9,1)),np.zeros((6,1)),np.ones((9,1))]).reshape(m_ineq)
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
for j in range(0,len(vertices)):
    vx = vertices[j][0]
    vy = vertices[j][1]
    tau = array([vx, vy, 0.])
    p = np.cross(n,tau)
    points.append([p[0],p[1]])
    
print points
#pypoman.plot_polygon(points)
