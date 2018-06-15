# -*- coding: utf-8 -*-
"""
Created on Thu Jun  7 22:14:22 2018

@author: Romeo Orsolino
"""


import time
import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial
from math_tools import Math
from constraints import Constraints
from kinematics import Kinematics

class AnalyticProjection():
    def minksum(self, a,b):
        n_a = np.size(a,1)
        n_b = np.size(b,1)
        #n_sum = n_a*n_b
        sum = np.zeros((0,np.size(a,0)))
        for j in range(0,n_a):
            for i in range(0,n_b):
                sum = np.vstack([sum, a[:,j] + b[:,i]])
            
    
            
        return np.transpose(sum)   
    
    def convex_hull(self, input_matrix):#requires 6XN
        
        hull_matrix = np.zeros((0,6))  
        input_matrix_t = np.transpose(input_matrix)
        hull = scipy.spatial.ConvexHull(input_matrix_t, qhull_options="Qx") #is pX6    
        #get the matrix
        indices = hull.vertices
        n = len(indices)    
        #print n
        for i in range(n):
            hull_matrix = np.vstack([hull_matrix, input_matrix_t[indices[i]]])   
        
        return np.transpose(hull_matrix)
    
    
    def compute_section_pointsZ(self, a, mg):
        
        #improved for loop: this has complexity n!/2!(n-2) = n^2/2 instead of n^2 (with the normal for)
        n_a = np.size(a,1)
        points = np.zeros((0,6))
        for j in range(0,n_a):
            for i in range(j+1,n_a): 
                lambda1 = a[:,j]
                lambda2 = a[:,i]
                if lambda1[5]!=lambda2[5]:
                    alpha = (mg - lambda1[5])/(lambda2[5] - lambda1[5])
                    #print alpha
                    if(alpha>=0.0)&(alpha<=1.0):
                        new_point = lambda1 + (lambda2 - lambda1)*alpha
                        points = np.vstack([points, new_point])
        #print 'number of edges', np.size(points,0)
        return np.transpose(points), np.size(points,0)
 
 
    def compute_section_points(self, a, mg):
        
        #improved for loop: this has complexity n!/2!(n-2) = n^2/2 instead of n^2 (with the normal for)
        n_a = np.size(a,1)
        pointsZ = np.zeros((0,6))
        for j in range(0,n_a):
            for i in range(j+1,n_a): 
                lambda1 = a[:,j]
                lambda2 = a[:,i]
                if not np.array_equal(lambda1,lambda2):
                    #cut with plane fz = mg with normal =[0 0 0 0 0 1] passing through point p3=[0 0 0 0 0 mg]
                    n= np.array([0,0,0,0,0,1])
                    p3 = np.array([0,0,0,0,0,mg])                             
                    alpha = np.dot(n, (lambda1- p3)) / np.dot(n, (lambda1-lambda2))
                    #print alpha
                    if(alpha>=0.0)&(alpha<=1.0):
                        new_point = lambda1 + (lambda2 - lambda1)*alpha
                        pointsZ = np.vstack([pointsZ, new_point])
        
        pointsZ = pointsZ.T
        
        n_z = np.size(pointsZ,1)                
        pointsX = np.zeros((0,6))
        for j in range(0,n_z):
            for i in range(j+1,n_a): 
                lambda1 = a[:,j]
                lambda2 = a[:,i]
                if not np.array_equal(lambda1,lambda2):
                    #cut with plane fz = mg with normal =[0 0 0 0 0 1] passing through point p3=[0 0 0 0 0 mg]
                    n= np.array([0,0,0,1,0,0])
                    p3 = np.array([0,0,0,0,0,0])                             
                    alpha = np.dot(n, (lambda1- p3)) / np.dot(n, (lambda1-lambda2))
                    #print alpha
                    if(alpha>=0.0)&(alpha<=1.0):
                        new_point = lambda1 + (lambda2 - lambda1)*alpha
                        pointsX = np.vstack([pointsX, new_point])                        
        #print 'number of edges', np.size(points,0)
        return np.transpose(pointsX), np.size(pointsX,0)       
    
         
    def project_points(self, vertices, mg):
        n = np.size(vertices,1)
        vertices2d = np.zeros((2,0))
        for j in range(0,n):
            tau_x = vertices[0,j]
            tau_y = vertices[1,j]
            v_new_x = - tau_y / mg
            v_new_y = tau_x / mg
            v_new = np.array([[v_new_x],[v_new_y]])
            vertices2d = np.hstack([vertices2d, v_new])
        #print 'number of com points', np.size(vertices2d,1)
        return vertices2d
    
    
    
    def analytic_projection(self, constraint_mode, contacts, normals, mass, ng, mu):
        start_t = time.time()
        r1 = contacts[0,:]
        r2 = contacts[1,:]
        r3 = contacts[2,:]        
        g = 9.81
        mg = mass*g
        
        n1 = normals[0,:]
        n2 = normals[1,:]
        n3 = normals[2,:]
        math_lp = Math()
        constr = Constraints()
        
        if constraint_mode == 'ONLY_ACTUATION':
            tau_HAA = 80
            tau_HFE = 120
            tau_KFE = 120
            dx = tau_HAA
            dy = tau_HFE
            dz = tau_KFE
            vertices_cl = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
                                 [dy, -dy, -dy, dy, dy, -dy, -dy, dy],
                                 [dz, dz, dz, dz, -dz, -dz, -dz, -dz]])
            kin = Kinematics()
            foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
            q, q_dot, J_LF, J_RF, J_LH, J_RH = kin.compute_xy_IK(np.transpose(contacts[:,0]),
                                                    np.transpose(foot_vel[:,0]),
                                                    np.transpose(contacts[:,2]),
                                                    np.transpose(foot_vel[:,2]))
            vertices_1 = np.transpose(constr.computeActuationPolygon(J_LF))
            vertices_2 = np.transpose(constr.computeActuationPolygon(J_LF))
            vertices_3 = np.transpose(constr.computeActuationPolygon(J_LF))
            #print vertices_1
            #print vertices_2
            #print vertices_3
            
        elif constraint_mode == 'ONLY_FRICTION':
        
            n1, n2, n3 = (math_lp.normalize(n) for n in [n1, n2, n3])
            R1, R2, R3 = (math_lp.rotation_matrix_from_normal(n) for n in [n1, n2, n3])
            # Inequality matrix for a contact force in local contact frame:
            #constr = Constraints()
            #C_force = constr.linearized_cone_local_frame(ng, mu)
            vertices_cl = constr.linearized_cone_vertices(ng, mu, 100.0*mg)
            #vertices_cl = np.array([[0., dx, dx, -dx, -dx],
            #                     [0., dy, -dy, -dy, dy],
            #                     [0., dz, dz, dz, dz]])
            #vertices_1 = np.dot(vertices_cl, R1.T)
            #vertices_2 = np.dot(vertices_cl, R2.T)
            #vertices_3 = np.dot(vertices_cl, R3.T)
            vertices_1 = vertices_cl
            vertices_2 = vertices_cl
            vertices_3 = vertices_cl

        
        vertices_1 = np.transpose(vertices_1)
        vertices_2 = np.transpose(vertices_2)
        vertices_3 = np.transpose(vertices_3)

        tau1 = np.zeros((3,np.size(vertices_1,1)))
        tau2 = np.zeros((3,np.size(vertices_2,1)))
        tau3 = np.zeros((3,np.size(vertices_3,1)))     
        #print tau1        
        for j in range(0,np.size(vertices_cl,1)):
            tau1[:,j] = np.cross(r1, vertices_1[:,j])
            tau2[:,j] = np.cross(r2, vertices_2[:,j])
            tau3[:,j] = np.cross(r3, vertices_3[:,j])
        w1 = np.vstack([tau1, vertices_1])
        w2 = np.vstack([tau2, vertices_2])
        w3 = np.vstack([tau3, vertices_3]) #6XN
        print w1, w2, w3
        w12 = self.minksum(w1, w2)
        w123 = self.minksum(w12, w3) #CWC con punti interni 6XN
        
        
        #print 'number of vertex before convex hull', np.size(w123,1)
        w123_hull = self.convex_hull(w123) #this speeds up significantly!
        #print 'number of vertex after convex hull', np.size(w123_hull,1)
        
        points, points_num = self.compute_section_points(w123_hull, mg) #compute edges and slice them with mg
        points2d = self.project_points(points, mg) #cimpute com points
        
        #print points
        vertices2d = np.transpose(points2d)
        chull = scipy.spatial.ConvexHull(vertices2d)
        #print chull.vertices[0]
        print("Closed form algorith: --- %s seconds ---" % (time.time() - start_t))
        
        return vertices2d, chull.simplices