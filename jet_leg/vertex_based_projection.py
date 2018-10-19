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
#from kinematics import Kinematics
from hyq_kinematics import HyQKinematics

class VertexBasedProjection():
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
        
        hull_matrix = np.zeros((0,np.size(input_matrix,0)))  
        input_matrix_t = np.transpose(input_matrix)
        #print input_matrix_t
        hull = scipy.spatial.ConvexHull(input_matrix_t, qhull_options="QbB") #is pX6    
        #get the matrix
        indices = hull.vertices
        n = len(indices)    
        #print n
        for i in range(n):
            hull_matrix = np.vstack([hull_matrix, input_matrix_t[indices[i]]])   
        
        return np.transpose(hull_matrix)
    
    
    def compute_section_points(self, input_points, normal_plane, point_plane):
        
        #improved for loop: this has complexity n!/2!(n-2) = n^2/2 instead of n^2 (with the normal for)
        n_a = np.size(input_points,1)
        output_points = np.zeros((0,np.size(input_points,0)))
        for j in range(0,n_a):
            for i in range(j+1,n_a): 
                lambda1 = input_points[:,j]
                lambda2 = input_points[:,i]
                if (not np.array_equal(lambda1,lambda2)):
                    #check non complanarity of edge and of plane otherwise the intersection goes to infinite
                    if (np.abs(np.dot(normal_plane, (lambda1-lambda2))) >1e-04):                     
                        #cut with plane fz = mg with plane whose normal is normal_plane passing through point_plane
                        alpha = np.dot(normal_plane, (lambda1 - point_plane)) / np.dot(normal_plane, (lambda1-lambda2))
                    
                        if(alpha>=0.0)&(alpha<=1.0):
                            #print alpha
                            new_point = lambda1 + (lambda2 - lambda1)*alpha
                            output_points = np.vstack([output_points, new_point])
        #print 'number of edges', np.size(points,0)
        return np.transpose(output_points), np.size(output_points,0)
 
    
         
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
        print vertices2d
        return vertices2d
    
    
    
    def project(self, constraint_mode, contacts, normals, mass, ng, mu):
        start_t = time.time()
        contactsNumber = np.size(contacts,0)
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
            kin = HyQKinematics()
            foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
            contactsFourLegs = np.vstack([contacts, np.zeros((4-contactsNumber,3))])
            q, q_dot, J_LF, J_RF, J_LH, J_RH,isOutOfWorkspace = kin.inverse_kin(np.transpose(contactsFourLegs[:,0]),
                                                    np.transpose(foot_vel[:,0]),
                                                    np.transpose(contactsFourLegs[:,1]),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contactsFourLegs[:,2]),
                                                    np.transpose(foot_vel[:,2]))
            J_LF, J_RF, J_LH, J_RH = kin.update_jacobians(q)
            vertices_1 = np.transpose(constr.computeActuationPolygon(J_LF))
            vertices_2 = np.transpose(constr.computeActuationPolygon(J_RF))
            vertices_3 = np.transpose(constr.computeActuationPolygon(J_LH))
            #print vertices_1
            #print vertices_2
            #print vertices_3
            
        elif constraint_mode == 'ONLY_FRICTION':
        
            n1, n2, n3 = (math_lp.normalize(n) for n in [n1, n2, n3])
            R1, R2, R3 = (math_lp.rotation_matrix_from_normal(n) for n in [n1, n2, n3])
            # Inequality matrix for a contact force in local contact frame:
            #constr = Constraints()
            #C_force = constr.linearized_cone_local_frame(ng, mu)
            vertices_cl = constr.linearized_cone_vertices(ng, mu, mg)
            #vertices_cl = np.array([[0., dx, dx, -dx, -dx],
            #                     [0., dy, -dy, -dy, dy],
            #                     [0., dz, dz, dz, dz]])
            vertices_1 = np.dot(vertices_cl, R1.T)
            vertices_2 = np.dot(vertices_cl, R2.T)
            vertices_3 = np.dot(vertices_cl, R3.T)
            #vertices_1 = vertices_cl
            #vertices_2 = vertices_cl
            #vertices_3 = vertices_cl

        
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
     
        #compute edges and slice them with mg
        pointsfZ, points_num = self.compute_section_points(w123_hull, np.array([0,0,0,0,0,1]), np.array([0,0,0,0,0,mg]))       
        #to avoid that convex hull complains because they are complanar remove the fz dimension
        pointsfZhull = self.convex_hull(pointsfZ[0:5,:])        
        
        pointsfY, points_num = self.compute_section_points(pointsfZhull, np.array([0,0,0,0,1]), np.array([0,0,0,0,0])) 
        #to avoid that convex hull complains because they are complanar remove the fz dimension        
        pointsfYhull = self.convex_hull(pointsfY[0:4,:])        
        
        
        pointsfX, points_num = self.compute_section_points(pointsfYhull, np.array([0,0,0,1]), np.array([0,0,0,0])) 
        #to avoid that convex hull complains because they are complanar remove the fz dimension        
        pointsfXhull = self.convex_hull(pointsfX[0:3,:])        

        pointstZ, points_num = self.compute_section_points(pointsfXhull, np.array([0,0,1]), np.array([0,0,0])) 
        #to avoid that convex hull complains because they are complanar remove the fz dimension        
        pointstZhull = self.convex_hull(pointstZ[0:2,:])          
#        pointsfX, points_num = self.compute_section_points(pointsfZhull, np.array([0,0,0,1,0,0]), np.array([0,0,0,0,0,0])) 
        
       
        #pointstZ, points_num = self.compute_section_points(pointsfY, np.array([0,0,1,0,0,0]), np.array([0,0,0,0,0,0])) 
        
            
        
        print np.size(w123_hull,1) 
        print "pointsfZ"        
        print np.size(pointsfZ,1)      
        print np.size(pointsfZhull,1) 
        
        print "pointsfy"
        print np.size(pointsfY,1)
        print np.size(pointsfYhull,1) #
        print "pointsfx"
        print np.size(pointsfX,1)
        print np.size(pointsfXhull,1) #
        print "pointstZx"
        print np.size(pointstZ,1)
        print np.size(pointstZhull,1) #
        
        print pointstZhull
#        points, points_num = self.compute_section_points(w123_hull, mg) 
#        
        # TODO: use pointsfZhull instead for "ONLY_FRICTION" option:
        #points2d = self.project_points(pointsfZhull, mg) #cimpute com points
        if constraint_mode == 'ONLY_ACTUATION':
            points2d = self.project_points(pointstZhull, mg) #cimpute com points
        elif constraint_mode == 'ONLY_FRICTION':
            points2d = self.project_points(pointsfZhull, mg) #cimpute com points
        
        #print points
        vertices2d = np.transpose(points2d)
        chull = scipy.spatial.ConvexHull(vertices2d)
        #print chull.vertices[0]
        print("Closed form algorith: --- %s seconds ---" % (time.time() - start_t))
        
        return vertices2d, chull.simplices