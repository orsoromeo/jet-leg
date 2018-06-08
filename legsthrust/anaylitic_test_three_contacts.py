# -*- coding: utf-8 -*-
"""
Created on Thu Jun  7 22:14:22 2018

@author: Romeo Orsolino
"""


import time
import numpy as np
import matplotlib.pyplot as plt
import scipy

start_time = time.time()

plt.close('all')
def minksum(a,b):
    n_a = np.size(a,1)
    n_b = np.size(b,1)
    #n_sum = n_a*n_b
    sum = np.zeros((1,np.size(a,0)))
    for j in range(0,n_a):
        for i in range(0,n_b):
            sum = np.vstack([sum, a[:,j] + b[:,i]])
        
    return np.transpose(sum)    

def compute_points(a, mass):
    n_a = np.size(a,1)
    points = np.zeros((0,6))
    for j in range(0,n_a):
        for i in range(0,n_a):
            lambda1 = a[:,j]
            lambda2 = a[:,i]
            if lambda1[5]!=lambda2[5]:
                alpha = (mass - lambda1[5])/(lambda2[5] - lambda1[5])
                #print alpha
                if(alpha>=-10e-2)&(alpha<=1.0+10e-2):
                    new_point = lambda1 + (lambda2 - lambda1)*alpha
                    points = np.vstack([points, new_point])
    print np.size(points,0)
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
    print np.size(vertices2d,1)
    return vertices2d
    
r1 = array([1.0, 0.0, 0.0])
r2 = array([2.3, 1.5, 0.0])
r3 = array([3.0, 0.2, 0.0])
contacts = np.vstack([r1, r2, r3, r1])
dx = 100
dy = 100
dz = 100
#vertices = np.array([[dx, dx, -dx, -dx, dx, dx, -dx, -dx],
#                         [dy, -dy, -dy, dy, dy, -dy, -dy, dy],
#                         [dz, dz, dz, dz, -dz, -dz, -dz, -dz]])
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
w3 = np.vstack([tau3, vertices])
print w1, w2, w3
w12 = minksum(w1, w2)
w123 = minksum(w12, w3)
#print w123, np.size(w123,1)

mass = 170
points, points_num = compute_points(w123, mass)
points2d = project_points(points, mass)
print np.size(points2d,0)
#print points
vertices2d = np.transpose(points2d)
chull = scipy.spatial.ConvexHull(vertices2d)
print chull.vertices[0]
print("--- %s seconds ---" % (time.time() - start_time))

plt.plot(contacts[:,0],contacts[:,1],'--b')
plt.plot(points2d[0,:], points2d[1,:], 'ro')
plt.plot(vertices2d[chull.vertices[0],0], vertices2d[chull.vertices[0],1], 'r')


for simplex in chull.simplices:
    plt.plot(vertices2d[simplex, 0], vertices2d[simplex, 1], 'k-')
plt.show()