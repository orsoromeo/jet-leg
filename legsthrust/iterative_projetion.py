# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 19:55:29 2018

@author: romeo orsolino
"""

from numpy.linalg import lstsq
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import numpy as np

plt.close('all')


class Polygon:
    
    def __init__(self, vertices = np.zeros((0,3)), halfspaces = np.zeros((0,3))):
        self.vx = vertices
        self.hs = halfspaces        
        
    def get_halfspaces(self, polygon):
        ''' we assume the vertices matrix contains all the vertices on the rows. Therefore the size of
        vertices will be N x 2 where N is the number of vertices'''
        vertices = polygon.vx
        hs = np.zeros((0,3))   
        for i in range(0,np.size(vertices,0)-1):
            p1 = vertices[i,:];
            p2 = vertices[i+1,:]
            print p1, p2
            if (p1[0]!=p2[0]):       
                x_coords = np.array([p1[0], p2[0]])
                y_coords = np.array([p1[1], p2[1]])
                A = np.vstack([x_coords, np.ones(len(x_coords))]).T
                m, c = lstsq(A, y_coords)[0]
                print("Line Solution is y = {m}x + {c}".format(m=m,c=c))
                a = -m
                b = 1
                c = c
            else:
                a = 1
                b = 0
                c = p1[0]
            
            hs = np.vstack([hs, np.hstack([a,b,c])])
        return hs
                  
    
'''1) Compute the edges of Y inner'''
LF_foot = np.array([0.3, 0.4, -0.3])
RF_foot = np.array([0.3, -0.1, -0.0])
LH_foot = np.array([-0.3, 0.2, -0.0])
RH_foot = np.array([-0.3, -0.2, -0.0])
Y_inner = np.vstack((LF_foot[0:2],RF_foot[0:2],LH_foot[0:2],RH_foot[0:2],LF_foot[0:2]))

#Y_inner = np.random.rand(30, 2)

hull = ConvexHull(Y_inner)
polygon = Polygon(Y_inner)
hs = polygon.get_halfspaces(polygon)
print hs
'''2) Pick the (unique) edge separating Y_inner from Y '''

'''3) Check the convergence of Y inner and Y outer '''

'''4) Otherwise find the point in Y furthest outside the edge i '''

'''5) Update the outer approzimation '''

'''6) Update the inner approximation '''


#plt.plot(Y_inner[:,0], Y_inner[:,1], 'o')
#for simplex in hull.simplices:
#    plt.plot(Y_inner[simplex, 0], Y_inner[simplex, 1], 'k-')
fig, ax = plt.subplots()
ax.grid(True)
plt.plot(Y_inner[hull.vertices,0], Y_inner[hull.vertices,1], 'r--', lw=5)
plt.plot(Y_inner[hull.vertices,0], Y_inner[hull.vertices,1], 'ro', lw=5)
plt.show()