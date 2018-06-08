# -*- coding: utf-8 -*-
"""
Created on Thu Jun  7 22:14:22 2018

@author: Romeo Orsolino
"""


import time
import numpy as np
import matplotlib.pyplot as plt
import scipy.spatial


start_time = time.time()

plt.close('all')
def minksum(a,b):
    n_a = np.size(a,1)
    n_b = np.size(b,1)
    #n_sum = n_a*n_b
    sum = np.zeros((0,np.size(a,0)))
    for j in range(0,n_a):
        for i in range(0,n_b):
            sum = np.vstack([sum, a[:,j] + b[:,i]])
        

        
    return np.transpose(sum)   

def convex_hull(input_matrix):#requires 6XN
    
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


def compute_points(a, mg):
    n_a = np.size(a,1)
    points = np.zeros((0,6))
    for j in range(0,n_a):
        for i in range(0,n_a):
            lambda1 = a[:,j]
            lambda2 = a[:,i]
            if lambda1[5]!=lambda2[5]:
                alpha = (mg - lambda1[5])/(lambda2[5] - lambda1[5])
                #print alpha
                if(alpha>=0.0)&(alpha<=1.0):
                    new_point = lambda1 + (lambda2 - lambda1)*alpha
                    points = np.vstack([points, new_point])
    print 'number of edges', np.size(points,0)
    return np.transpose(points), np.size(points,0)
    

     
def project_points(vertices, mass):
    n = np.size(vertices,1)
    vertices2d = np.zeros((2,0))
    for j in range(0,n):
        tau_x = vertices[0,j]
        tau_y = vertices[1,j]
        v_new_x = - tau_y / mass
        v_new_y = tau_x / mass
        v_new = np.array([[v_new_x],[v_new_y]])
        vertices2d = np.hstack([vertices2d, v_new])
    print 'number of com points', np.size(vertices2d,1)
    return vertices2d
    
'''MAIN'''
r1 = np.array([1.0, 0.0, 0.0])
r2 = np.array([2.3, 1.5, 0.0])
r3 = np.array([3.0, 0.2, 0.0])
contacts = np.vstack([r1, r2, r3, r1])
mg = 170
dx = 100*mg
dy = 100*mg
dz = 100*mg
#vertices = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
#                     [dy, -dy, -dy, dy, dy, -dy, -dy, dy],
#                     [dz, dz, dz, dz, -dz, -dz, -dz, -dz]])
vertices = np.array([[0., dx, dx, -dx, -dx],
                         [0., dy, -dy, -dy, dy],
                         [0., dz, dz, dz, dz]])
tau1 = np.zeros((3,np.size(vertices,1)))
tau2 = np.zeros((3,np.size(vertices,1)))
tau3 = np.zeros((3,np.size(vertices,1)))             
for j in range(0,np.size(vertices,1)):
    tau1[:,j] = np.cross(r1, vertices[:,j])
    tau2[:,j] = np.cross(r2, vertices[:,j])
    tau3[:,j] = np.cross(r3, vertices[:,j])
w1 = np.vstack([tau1, vertices])
w2 = np.vstack([tau2, vertices])
w3 = np.vstack([tau3, vertices]) #6XN
print w1, w2, w3
w12 = minksum(w1, w2)
w123 = minksum(w12, w3) #CWC con punti interni 6XN


print 'number of vertex before convex hull', np.size(w123,1)
w123_hull = convex_hull(w123)
print 'number of vertex after convex hull', np.size(w123_hull,1)

points, points_num = compute_points(w123_hull, mg) #compute edges and slice them with mg
points2d = project_points(points, mg) #cimpute com points

#print points
vertices2d = np.transpose(points2d)
chull = scipy.spatial.ConvexHull(vertices2d)
#print chull.vertices[0]
print("--- %s seconds ---" % (time.time() - start_time))

plt.plot(contacts[:,0],contacts[:,1],'--b')
plt.plot(points2d[0,:], points2d[1,:], 'ro')

for simplex in chull.simplices:
    plt.plot(vertices2d[simplex, 0], vertices2d[simplex, 1], 'k-')
plt.show()