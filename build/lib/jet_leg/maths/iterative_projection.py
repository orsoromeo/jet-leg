# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 19:55:29 2018

@author: romeo orsolino
"""

from numpy.linalg import lstsq
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import numpy as np
from cvxopt import matrix, solvers
from computational_dynamics import ComputationalDynamics
from hyq_kinematics import HyQKinematics
from math_tools import Math
from constraints import Constraints
import random
from plotting_tools import Plotter

plt.close('all')


class Polygon:
    
    def __init__(self, vertices = np.zeros((0,2)), halfspaces = np.zeros((0,2))):
        self.vx = vertices
        self.hs = halfspaces  
        
    def get_vertices(self):
        return self.vx
        
    def set_vertices(self, vertices):
        self.vx = vertices
        
    def get_halfspaces(self):
        return self.hs
        
    def set_halfspace(self, halfspaces):
        self.hs = halfspaces

    def clockwise_sort(self, polygon):
        polygon.vx
        vertices_number = np.size(polygon.vx,0)-1
        angle = [0,0,0,0]
        for j in range(0,vertices_number):
            angle[j] = np.arctan2(polygon.vx[j,0], polygon.vx[j,1])
        
        index = np.argsort(angle)
        
        sorted_vertices = np.zeros((vertices_number,2))
        for j in range(0,vertices_number):
            sorted_vertices[j,:] = polygon.vx[index[j],:]       
        
        sorted_vertices = np.vstack([sorted_vertices, sorted_vertices[0,:]])
        self.set_vertices(sorted_vertices)
        
        
class InnerPolygon(Polygon):
    
    def __init__(self, neighbour_outer_vertices = np.zeros((0,2))):       
        self.neighbours = neighbour_outer_vertices
        
class PolygonTriple(Polygon):
  
    def __init__(self):       
        self.v1 = Polygon()
        self.v2 = Polygon()
        self.v3 = Polygon()
        

class IterativeProjection:
    
    def line(self, p1, p2):
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0]*p2[1] - p2[0]*p1[1])
        return A, B, -C

    def intersection(self, L1, L2):
        D  = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x,y
        else:
            return False
            
    def initiliaze_outer_appriximation(self, inner_approximation, directions):
        
        v1 = inner_approximation[0,:]
        hs1 = directions[0,:]
        
        v2 = inner_approximation[1,:]
        hs2= directions[1,:]
        
        v3 = inner_approximation[2,:]
        hs3 = directions[2,:]
        
        R1 = self.intersection(hs1, hs2)
        R2 = self.intersection(hs2, hs3)
        R3 = self.intersection(hs3, hs1)


        
        outer_vertices = np.vstack([R1, R2, R3])
        return outer_vertices
        
    def initiliaze_inner_appriximation(self, constraint_mode, mass, contactsNumber, contacts, com, normals):
        random.seed()
        random.seed(9001)        
        direction1 = np.vstack([random.uniform(-1,1),random.uniform(-1,1)])
        inner_vertex1, force1 = self.expand_direction(constraint_mode, mass, contactsNumber, contacts, com, normals, direction1)
        print inner_vertex1
        print direction1        
        c1 = np.dot(np.transpose(inner_vertex1), direction1)
        line1 = np.vstack([direction1, c1])

        direction2 = np.vstack([random.uniform(-1,1),random.uniform(-1,1)])
        inner_vertex2, force2 = self.expand_direction(constraint_mode, mass, contactsNumber, contacts, com, normals, direction2)
        c2 = np.dot(np.transpose(inner_vertex2), direction2)
        line2 = np.vstack([direction2, c2])
        
        direction3 = np.vstack([random.uniform(-1,1),random.uniform(-1,1)])
        inner_vertex3, force3 = self.expand_direction(constraint_mode, mass, contactsNumber, contacts, com, normals, direction3)
        c3 = np.dot(np.transpose(inner_vertex3), direction3)
        line3 = np.vstack([direction3, c3])
        
        vertices = np.hstack([inner_vertex1,inner_vertex2,inner_vertex3])
        directions = np.hstack([line1,line2, line3])
        return np.transpose(vertices), np.transpose(directions)
        
    def expand_direction(self, constraint_mode, mass, contactsNumber, contacts, com, normals, direction):
        g = 9.81    
        friction_coeff = 0.6
        ng = 4
        grav = np.array([[0.], [0.], [-g*mass]])
        cost_function = np.vstack([direction, np.zeros((3*nc,1))])
        p = matrix(cost_function) 
        #p = matrix(np.zeros((3*nc,1)))
        torque = -np.cross(com, np.transpose(grav))
        gravity_term = math.skew(grav)
        #print gravity_term
        A2 = np.vstack([np.zeros((3,2)), gravity_term[:,0:2]])
        
        A1 = A2#np.zeros((6,0))
        #print np.size(A2,0)
        compDyn = ComputationalDynamics()
        for j in range(0,contactsNumber):
            r = contacts[j,:]
            GraspMat = compDyn.getGraspMatrix(r)
            A1 = np.hstack((A1, GraspMat[:,0:3]))
            #A1 = matrix(A1)
            b = matrix(np.vstack([-grav, np.transpose(torque)]).reshape((6)))
        
        #A = np.hstack((A2, A1))
        A = matrix(A1)
        #contactsFourLegs = np.vstack([contacts, np.zeros((4-nc,3))])\
        kin = HyQKinematics()
        foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
        contactsFourLegs = np.vstack([contacts, np.zeros((4-contactsNumber,3))])
        q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = kin.inverse_kin(np.transpose(contactsFourLegs[:,0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contactsFourLegs[:,1]),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contactsFourLegs[:,2]),
                                                    np.transpose(foot_vel[:,2]))
        if (not isOutOfWorkspace):
            #kin.update_jacobians(q)
            J_LF, J_RF, J_LH, J_RH = kin.update_jacobians(q)
            #print J_LF
            constraint = Constraints()
            G_force, h, isConstraintOk = constraint.inequalities(constraint_mode, nc, ng, normals, friction_coeff, J_LF, J_RF, J_LH, J_RH)
            G = np.hstack([np.zeros((np.size(G_force,0),2)), G_force])
            G = matrix(G)

            #print G, h
            if not isConstraintOk:
                print 'something is wrong in the inequalities'
            else:
                #print p
                #print G,h
                #print A,b
                sol=solvers.lp(p, G, h, A, b)
                x = sol['x']
                status = sol['status']
                inner_vertex = x[0:2]
                force = x[2:np.size(A,1)]
                
        return inner_vertex, force
                        
    def compute_halfspaces(self, polygon):
        ''' we assume the vertices matrix contains all the vertices on the rows. Therefore the size of
        vertices will be N x 2 where N is the number of vertices'''
        vertices = polygon.vx
        hs = np.zeros((0,3))   
        for i in range(0,np.size(vertices,0)-1):
            p1 = vertices[i,:];
            p2 = vertices[i+1,:]
            #print p1, p2
            if (p1[0]!=p2[0]):       
                #x_coords = np.array([p1[0], p2[0]])
                #y_coords = np.array([p1[1], p2[1]])
                #A = np.vstack([x_coords, np.ones(len(x_coords))]).T
                #m, c = lstsq(A, y_coords)[0]
                #print("Line Solution is y = {m}x + {c}".format(m=m,c=c))
                A = (p2[1]- p1[1])/(p2[0] - p1[0])                
                a = A
                b = -1
                c = p1[1] - A*p1[0]
            else:
                a = 1
                b = 0
                c = p1[0]
            new_hs = np.hstack([a,b,c])
            #print new_hs
            hs = np.vstack([hs, new_hs])
        return hs
            

constraint_mode = 'ONLY_ACTUATION'
com = np.array([0.0, 0.0, 0.0])
mass = 80
LF_foot = np.array([0.3, 0.3, -0.5])
RF_foot = np.array([0.3, -0.3, -0.5])
LH_foot = np.array([-0.3, 0.3, -0.5])
RH_foot = np.array([-0.3, -0.3, -0.5])
Y_inner = np.vstack((LF_foot[0:2],RF_foot[0:2],LH_foot[0:2],RH_foot[0:2],LF_foot[0:2]))

#Y_inner = np.random.rand(30, 2)

hull = ConvexHull(Y_inner)
polygon = Polygon(Y_inner)
iterativeProjection = IterativeProjection()

innerPolygon = InnerPolygon(Y_inner)

set1 = PolygonTriple()


polygon.clockwise_sort(polygon)
hs = iterativeProjection.compute_halfspaces(polygon)
#print hs

nc = 3

contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts = contactsToStack[0:nc, :]

axisZ= np.array([[0.0], [0.0], [1.0]])

math = Math()
n1 = np.transpose(np.transpose(math.rpyToRot(1.5,1.5,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
normals = np.vstack([n1, n2, n3, n4])


''' initialize Y_inner and Y_outer '''
inner_vx, directions = iterativeProjection.initiliaze_inner_appriximation(constraint_mode, mass, nc, contacts, com, normals)
outer_vx = iterativeProjection.initiliaze_outer_appriximation(inner_vx, directions)
'''1) Compute the edges of Y inner'''

'''2) Pick the edge cutting off the greatest fraction  of Y outer '''

'''3) Find the point in Y furthest outside this edge '''

'''4) Update the outer approximation '''

'''5) Update the inner approximation '''


#plt.plot(Y_inner[:,0], Y_inner[:,1], 'o')
#for simplex in hull.simplices:
#    plt.plot(Y_inner[simplex, 0], Y_inner[simplex, 1], 'k-')
fig, ax = plt.subplots()
plotter = Plotter()
ax.grid(True)
#plt.plot(Y_inner[hull.vertices,0], Y_inner[hull.vertices,1], 'r--', lw=5)
plt.plot(Y_inner[hull.vertices,0], Y_inner[hull.vertices,1], 'bo', lw=5)
plotter.plot_polygon(inner_vx, color = '--g')
plotter.plot_polygon(outer_vx, color = '--r')
plt.show()