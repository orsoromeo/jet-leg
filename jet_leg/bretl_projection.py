# -*- coding: utf-8 -*-
"""
Created on Mon Jun  4 15:45:43 2018

@author: Romeo Orsolino

"""
import math
import numpy as np
# for plotting
from plotting_tools import Plotter
from arrow3D import Arrow3D
import matplotlib.pyplot as plt
    
def get_edges(vertices):
    edges = np.zeros((0,3))
    for j in range(1, points_num):
        diff = vertices[j,:] - vertices[j-1,:]
        edge = np.hstack([diff, np.dot(diff,vertices[j,:])])
        edges = np.vstack([edges,edge])
    
    diff = vertices[0,:] - vertices[-1,:]
    edge = np.hstack([diff, np.dot(diff,vertices[0,:])])
    edges = np.vstack([edges,edge])
    return edges
    
def find_closest_points(vertex_out, inner_approx):
    points_num = np.size(inner_approx,0)
    sorted_distance = [None]*points_num
    for j in range(0, points_num):
        v_in = inner_approx[j,:]
        sorted_distance[j] = math.hypot(vertex_out[0] - v_in[0], vertex_out[1] - v_in[1])
    
    sorted_indices = np.argsort(sorted_distance)
    return sorted_distance, sorted_indices
    
def get_largest_area(inner_approx, outer_approx):
    points_num = np.size(inner_approx,0)
    old_area = 0.
    area = 0.
    for j in range(0, points_num):
        closest_v, indices = find_closest_points(outer_approx[j,:], inner_approx)
        p1 = np.hstack([outer_approx[j,:],0.])
        p2 = np.hstack([inner_approx[indices[0],:],0.])
        p3 = np.hstack([inner_approx[indices[1],:],0.])
        cross_prod = np.cross(p2 - p1, p3 - p1)
        area = cross_prod[2]        
        if area > old_area :
            edge_to_expand = np.vstack([closest_v[indices[0],:], closest_v[indices[1],:]])            
            
    ordered_indices = np.argsort(area)

    return largest_area 
    
r1 = array([10.0, 10.0, 0.0])
r2 = array([10.0,-10.0, 0.0])
r3 = array([-10.0,-10.0, 0.0])
r4 = array([-10.0,10.0, 0.0])
''' init outer and inner approximation'''
outer_approx = np.vstack([r1[0:2], r2[0:2], r3[0:2], r4[0:2]])
inner_approx = np.zeros((0,2))
points_num = np.size(outer_approx,0)
for j in range(0, points_num-1):
    avg = (outer_approx[j,:] + outer_approx[j+1,:])/2.
    inner_approx = np.vstack([inner_approx, avg])

avg = (outer_approx[points_num-1,:] + outer_approx[0,:])/2.
inner_approx = np.vstack([inner_approx, avg])

''' get the edges from the v-description '''
outer_edges = get_edges(outer_approx)
inner_edges = get_edges(inner_approx)

''' compute max area '''
find_closest_points(outer_approx[0,:], inner_approx)
get_largest_area(outer_approx, inner_approx)
print 

print outer_edges
print inner_edges
''' plotting '''
inner_x = np.hstack([inner_approx[:,0], inner_approx[0,0]])
inner_y = np.hstack([inner_approx[:,1], inner_approx[0,1]])
outer_x = np.hstack([outer_approx[:,0], outer_approx[0,0]])
outer_y = np.hstack([outer_approx[:,1], outer_approx[0,1]])

plt.plot(inner_x, inner_y)
plt.plot(outer_x, outer_y)