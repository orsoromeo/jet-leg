# -*- coding: utf-8 -*-
"""
Created on Sat May 26 15:09:24 2018

@author: rorsolino
"""
import cvxopt
from cvxopt import matrix, solvers
import numpy as np
# for plotting
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def skew(v):
    if len(v) == 4: v = v[:3]/v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T
    
def getGraspMatrix(r):
    G = np.block([[np.eye(3), np.zeros((3,3))],
                   [skew(r), np.eye(3)]])
    return G
    
def linear_cone_constraint(n, mu):
    m = np.eye(3) - np.dot(n,np.transpose(n))
    u = np.dot(n,mu)
    #cone_constraints = m - np.transpose(u)
    cone_constraints = np.vstack((m - np.transpose(u), - m - np.transpose(u)))
    return cone_constraints

nc = 3;
g = 9.81
mass = 100
grav = np.array([[0.], [0.], [-g*mass]])
# contact points
r1 = np.array([00.0, 10.0, 0.0])
r2 = np.array([10.0, -10.0, 0.0])
r3 = np.array([-10.0, -10.0, 0.0])

# contact surface normals
n1 = np.array([[0.0], [0.0], [1.0]])
n2 = np.array([[0.0], [0.0], [1.0]])
n3 = np.array([[0.0], [0.0], [1.0]])

contacts = np.vstack((r1, r2, r3))
friction_coeff = 1.0

c1 = linear_cone_constraint(n1,friction_coeff)
cons = np.zeros((0,0))
for j in range(0,nc):
    cons = np.block([[cons, np.zeros((np.size(cons,0),np.size(c1,1)))],
                  [np.zeros((np.size(c1,0),np.size(cons,1))), c1]])
#print C
Q = 2*matrix(np.zeros((3*nc,3*nc)))
p = matrix(np.ones((3*nc,1)))
print Q, p
print np.size(Q,0), np.size(Q,1)

# Inequality constraints
m_ineq = 6*nc
#A=A.astype(double) 
#cons = cons.astype(np.double)
G = matrix(cons) #matrix([[-1.0,0.0],[0.0,-1.0]])
h = matrix(np.zeros((m_ineq,1)).reshape(m_ineq)) #matrix([0.0,0.0])
print G, h
print np.size(G,0), np.size(G,1)

# Equality constraints
com = np.array([-20.0, -10.0, 0.0])
print com, grav
torque = -np.cross(com, np.transpose(grav))
print 'torque: ', torque
A = np.zeros((6,0))
for j in range(0,nc):
    r = contacts[j,:]
    GraspMat = getGraspMatrix(r)
    A = np.hstack((A, GraspMat[:,0:3]))
A = matrix(A)
b = matrix(np.vstack([-grav, np.transpose(torque)]).reshape((6)))
#A = matrix([1.0, 1.0], (1,2))
#b = matrix(1.0)
print A, b
print np.size(A,0), np.size(A,1)

sol=solvers.lp(p, G, h, A, b)
x = sol['x']
print sol
status = sol['status']
print status
print x
print(sol['primal objective'])

feasible_points = np.zeros((0,3))
unfeasible_points = np.zeros((0,3))
print len(com), len(feasible_points)
if status == 'optimal':
    feasible_points = np.vstack([feasible_points,com])
else:
    unfeasible_points = np.vstack([unfeasible_points,com])
   
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(contacts[:,0], contacts[:,1], contacts[:,2],c='b',s=70)
if np.size(feasible_points,0) != 0:
    ax.scatter(feasible_points[:,0], feasible_points[:,1], feasible_points[:,2],c='g',s=50)
if np.size(unfeasible_points,0) != 0:
    ax.scatter(unfeasible_points[:,0], unfeasible_points[:,1], unfeasible_points[:,2],c='r',s=50)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()

print "bye bye" 