# -*- coding: utf-8 -*-
"""
Created on Sat May 26 15:09:24 2018

@author: rorsolino
"""
import cvxopt
from cvxopt import matrix, solvers
import numpy as np
from kinematics import Kinematics
from constraints import Constraints
# for plotting
from plotting_tools import Plotter
from arrow3D import Arrow3D
import matplotlib.pyplot as plt


def skew(v):
    if len(v) == 4: v = v[:3]/v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T
    
def getGraspMatrix(r):
    G = np.block([[np.eye(3), np.zeros((3,3))],
                   [skew(r), np.eye(3)]])
    return G
    
def normalize(n):
    norm1 = np.linalg.norm(n)
    n = np.true_divide(n, norm1)
    return n

nc = 3;
g = 9.81
mass = 80
grav = np.array([[0.], [0.], [-g*mass]])
# contact points
LF_foot = np.array([0.3, 0.2, -.5])
RF_foot = np.array([0.3, -0.2, -.5])
LH_foot = np.array([-0.3, 0.2, -.5])
RH_foot = np.array([-0.3, -0.2, -.5])
contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
# contact surface normals
n1 = np.array([[0.0], [0.0], [1.0]])
n1 = normalize(n1)
n2 = np.array([[0.0], [0.0], [1.0]])
n2 = normalize(n2)
n3 = np.array([[0.0], [0.0], [1.0]])
n3 = normalize(n3)
n4 = np.array([[0.0], [0.0], [1.0]])
n4 = normalize(n4)

normals = np.hstack((n1, n2, n3, n4))

friction_coeff = 1.0

#print C
Q = 2*matrix(np.zeros((3*nc,3*nc)))
p = matrix(np.ones((3*nc,1)))
#print Q, p
#print np.size(Q,0), np.size(Q,1)

cons1 = np.zeros((0,0))
h_vec1 = np.zeros((0,1))
cons2 = np.zeros((0,0))
h_vec2 = np.zeros((0,1))

kin = Kinematics()
foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
q, q_dot, J_LF, J_RF, J_LH, J_RH = kin.compute_xy_IK(np.transpose(contacts[:,0]),
                                          np.transpose(foot_vel[:,0]),
                                            np.transpose(contacts[:,2]),
                                            np.transpose(foot_vel[:,2]))
                                            
constraint = Constraints()
dx = 80
dy = 120
dz = 120
vertices = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
                     [dy, -dy, -dy, dy, dy, -dy, -dy, dy],
                     [dz, dz, dz, dz, -dz, -dz, -dz, -dz]])
                     
vertices_xz = np.vstack([vertices[0,:],vertices[2,:]])
actuation_polygon_xy = np.matmul(np.linalg.inv(np.transpose(J_LF)),vertices_xz) 
actuation_polygon_LF = np.vstack([actuation_polygon_xy[0,:],
                               vertices[1,:],
                               actuation_polygon_xy[1,:]])
                     
for j in range(0,nc):
    c, h_term = constraint.linear_cone(normals[:,j],friction_coeff)
    cons1 = np.block([[cons1, np.zeros((np.size(cons1,0),np.size(c,1)))],
                  [np.zeros((np.size(c,0),np.size(cons1,1))), c]])
    h_vec1 = np.vstack([h_vec1, h_term])
    c, h_term = constraint.hexahedron(actuation_polygon_LF)
    #c, h_term = constraint.zonotope()    
    cons2 = np.block([[cons2, np.zeros((np.size(cons2,0),np.size(c,1)))],
                  [np.zeros((np.size(c,0),np.size(cons2,1))), c]])    
    h_vec2 = np.vstack([h_vec2, h_term])

constraint_mode = 'only_friction'

if constraint_mode == 'only_friction':
    cons = cons1
    h_vec = h_vec1
elif constraint_mode == 'only_actuation':
    cons = cons2
    h_vec = h_vec2
elif constraint_mode == 'friction_and_actuation':
    cons = np.vstack([cons1, cons2])
    h_vec = np.vstack([h_vec1, h_vec2])

# Inequality constraints
m_ineq = np.size(cons,0)
#A=A.astype(double) 
#cons = cons.astype(np.double)
G = matrix(cons) #matrix([[-1.0,0.0],[0.0,-1.0]])
h = matrix(h_vec.reshape(m_ineq)) #matrix([0.0,0.0])
print G, h
print np.size(G,0), np.size(G,1)

feasible_points = np.zeros((0,3))
unfeasible_points = np.zeros((0,3))
# Equality constraints
for com_x in np.arange(-0.7,0.7,0.01):
    for com_y in np.arange(-0.6,0.5,0.01):
        com = np.array([com_x, com_y, 0.0])
        torque = -np.cross(com, np.transpose(grav))
        A = np.zeros((6,0))
        for j in range(0,nc):
            r = contacts[j,:]
            GraspMat = getGraspMatrix(r)
            A = np.hstack((A, GraspMat[:,0:3]))
        A = matrix(A)
        b = matrix(np.vstack([-grav, np.transpose(torque)]).reshape((6)))
        #A = matrix([1.0, 1.0], (1,2))
        #b = matrix(1.0)

        sol=solvers.lp(p, G, h, A, b)
        x = sol['x']
        status = sol['status']
        #print status
        if status == 'optimal':
            feasible_points = np.vstack([feasible_points,com])
        else:
            unfeasible_points = np.vstack([unfeasible_points,com])
        #print 'iteration ', com_x


# Plotting the results   
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for j in range(0,nc):
    ax.scatter(contacts[j,0], contacts[j,1], contacts[j,2],c='b',s=100)
    a = Arrow3D([contacts[j,0], contacts[j,0]+normals[0,j]/3], [contacts[j,1], contacts[j,1]+normals[1,j]/3],[contacts[j,2], contacts[j,2]+normals[2,j]/3], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
    ax.add_artist(a)
    
if np.size(feasible_points,0) != 0:
    ax.scatter(feasible_points[:,0], feasible_points[:,1], feasible_points[:,2],c='g',s=50)
if np.size(unfeasible_points,0) != 0:
    ax.scatter(unfeasible_points[:,0], unfeasible_points[:,1], unfeasible_points[:,2],c='r',s=50)

vertices = np.array([[500, 500, -500, -500, 500, 500, -500, -500],
                     [500, -500, -500, 500, 500, -500, -500, 500],
                     [500, 500, 500, 500, -500, -500, -500, -500]])

plotter = Plotter()                    
plotter.plot_actuation_polygon(ax, actuation_polygon_LF, LF_foot)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.draw()
plt.show()

print "bye bye" 
