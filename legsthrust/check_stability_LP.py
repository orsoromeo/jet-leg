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
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
        
def computeIK(x,x_dot,z,z_dot):
    footPosDes = np.vstack([x,z])
    BASE2HAA_offsets = np.array([[0.3735,0.3735,-0.3735,-0.3735],
                             [-.08, -.08, -.08, -.08]]);
    upperLegLength = 0.35;
    lowerLegLength = 0.341;
    footPosHAA = footPosDes - BASE2HAA_offsets;
    haa2hfeLength = 0.045;
    M_PI = 3.1415;

    sz = np.size(x,0);
    # 1 -> LF
    # 2 -> RF
    # 3 -> LH
    # 4 -> RH
    q = np.zeros(12,sz);
    q_dot = np.zeros(12,sz);

    # remove the haa2hfe offset and rotate in the sagittal plane of the leg
    hfe2foot = np.sqrt(footPosHAA(1)^2 + footPosHAA(2)^2) - haa2hfeLength;
    # add the x component
    # hfe2foot = sqrt(hfe2foot * hfe2foot);
    # HAA joints
    q[0] = -np.arctan2(footPosHAA[0,0],-footPosHAA[1,0]); # LF HAA
    q[6] = -np.arctan2(footPosHAA(0,1),-footPosHAA(1,1)); # LH HAA
    q[3] = -np.arctan2(-footPosHAA(0,2),-footPosHAA(1,2));# RF HAA
    q[9] = -np.arctan2(-footPosHAA(0,3),-footPosHAA(1,3));# RH HAA

    # HFE and KFE joints (use cosine law)
    cos_arg = (upperLegLength * upperLegLength + lowerLegLength * lowerLegLength - hfe2foot * hfe2foot) / (2 * upperLegLength * lowerLegLength);
    q[3] = - M_PI + acos(cos_arg); # LF KFE
    q[6] = - M_PI + acos(cos_arg); # RF KFE
    cos_arg = (upperLegLength^2 + hfe2foot^2 - lowerLegLength^2) / (2 * upperLegLength * hfe2foot) ;
    sin_arg = footPosHAA(1) / hfe2foot; #it should be footPosHFE(rbd::X)/hfe2foot but footPosHFE(rbd::X) = footPosHAA(rbd::X)	
    q[2] = -asin(sin_arg) + acos(cos_arg);# LF HFE
    q[5] = -asin(sin_arg) + acos(cos_arg);# RF HFE

    cos_arg = (upperLegLength * upperLegLength + lowerLegLength * lowerLegLength - hfe2foot * hfe2foot)/ (2 * upperLegLength * lowerLegLength);
    q[9]= + M_PI- acos(cos_arg); # LH KFE
    q[12] = + M_PI- acos(cos_arg); # RH KFE
    cos_arg = (upperLegLength * upperLegLength + hfe2foot * hfe2foot- lowerLegLength * lowerLegLength) / (2 * upperLegLength * hfe2foot);
    sin_arg = footPosHAA(1,3) / hfe2foot; # it should be footPosHFE(rbd::X)/hfe2foot but footPosHFE(rbd::X) = footPosHAA(rbd::X)
    q[8] = -asin(sin_arg)- acos(cos_arg);# LH HFE
    q[11] = -asin(sin_arg)- acos(cos_arg);# RH HFE

    # compute joint velocities updating the jac with the computed position
    l1 = upperLegLength;
    l2 = lowerLegLength;
    
    Jac_LF = np.array([[-l1*cos(q(2)) - l2 * cos(q(2) + q(3)), - l2 * cos(q(2) + q(3))],
                        [l1*sin(q(2)) + l2 * sin(q(2)+q(3)),  l2 * sin(q(2)+q(3))]]);
    Jac_RF = np.array([[l1*cos(q(5)) + l2 * cos(q(5)+q(6)),  l2 * cos(q(5) + q(6))],
             [l1*sin(q(5)) + l2 * sin(q(5)+q(6)),  l2 * sin(q(5) + q(6))]]);
     
    Jac_LH = np.array([[-l1*cos(q(8)) - l2 * cos(q(8)+q(9)),  -l2 * cos(q(8)+q(9))],
              [l1*sin(q(8)) + l2 * sin(q(8)+q(9)),  l2 * sin(q(8)+q(9))]]);
    Jac_RH = np.array([[l1*cos(q(11)) + l2 * cos(q(11)+q(12)),  l2 * cos(q(11)+q(12))],
              [l1*sin(q(11)) + l2 * sin(q(11)+q(12)),  l2 * sin(q(11)+q(12))]]);
    
    footVelDes = np.vstack([x_dot, z_dot]); 
    return q, Jac_LF, Jac_RF, Jac_LH, Jac_RH

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
    known_term = np.zeros((6,1))
    return cone_constraints, known_term
    
def actuation_polygon_constraint():
    dx = 200
    dy = 200
    dz = 700
    constraint = np.vstack([np.eye(3), -np.eye(3)])
    print constraint
    known_term = np.array([[dx],[dy],[dz],[dx],[dy],[dz]])
    return constraint, known_term
    
def normalize(n):
    norm1 = np.linalg.norm(n)
    n = np.true_divide(n, norm1)
    return n
    
def plot_cube(vertices):
    vx = vertices[0,0:8]
    vy = vertices[1,0:8]
    vz = vertices[2,0:8]
    face1x = np.hstack([vx[0:4],vx[0]])
    face1y = np.hstack([vy[0:4],vy[0]]) 
    face1z = np.hstack([vz[0:4],vz[0]])     
    surf = ax.plot_wireframe(face1x, face1y, face1z, alpha = 1.0)
    face2x = np.hstack([vx[4:8],vx[4]])
    face2y = np.hstack([vy[4:8],vy[4]]) 
    face2z = np.hstack([vz[4:8],vz[4]])     
    surf = ax.plot_wireframe(face2x, face2y, face2z, alpha = 1.0)

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)
        
nc = 4;
g = 9.81
mass = 100
grav = np.array([[0.], [0.], [-g*mass]])
# contact points
r1 = np.array([0.0, 10.0, 2.0])
r2 = np.array([10.0, -10.0, 1.5])
r3 = np.array([-10.0, -10.0, 1.0])
r4 = np.array([0.0, -20.0, 1.0])
contacts = np.vstack((r1, r2, r3, r4))
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
print Q, p
print np.size(Q,0), np.size(Q,1)

cons1 = np.zeros((0,0))
h_vec1 = np.zeros((0,1))
cons2 = np.zeros((0,0))
h_vec2 = np.zeros((0,1))
for j in range(0,nc):
    c, h_term = linear_cone_constraint(normals[:,j],friction_coeff)
    cons1 = np.block([[cons1, np.zeros((np.size(cons1,0),np.size(c,1)))],
                  [np.zeros((np.size(c,0),np.size(cons1,1))), c]])
    h_vec1 = np.vstack([h_vec1, h_term])
    c, h_term = actuation_polygon_constraint()
    cons2 = np.block([[cons2, np.zeros((np.size(cons2,0),np.size(c,1)))],
                  [np.zeros((np.size(c,0),np.size(cons2,1))), c]])
    h_vec2 = np.vstack([h_vec2, h_term])

constraint_mode = 'friction_and_actuation'

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
for com_x in np.arange(-25,25,5.):
    for com_y in np.arange(-35,15,5.):
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
    a = Arrow3D([contacts[j,0], contacts[j,0]+normals[0,j]], [contacts[j,1], contacts[j,1]+normals[1,j]],[contacts[j,2], contacts[j,2]+normals[2,j]], mutation_scale=20, lw=3, arrowstyle="-|>", color="r")
    ax.add_artist(a)
    
if np.size(feasible_points,0) != 0:
    ax.scatter(feasible_points[:,0], feasible_points[:,1], feasible_points[:,2],c='g',s=50)
if np.size(unfeasible_points,0) != 0:
    ax.scatter(unfeasible_points[:,0], unfeasible_points[:,1], unfeasible_points[:,2],c='r',s=50)

vertices = np.array([[10, 10, -10, -10, 10, 10, -10, -10],
                     [10, -10, -10, 10, 10, -10, -10, 10],
                     [10, 10, 10, 10, -10, -10, -10, -10]])
plot_cube(vertices)

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.draw()
plt.show()

print "bye bye" 
