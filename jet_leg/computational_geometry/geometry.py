# -*- coding: utf-8 -*-
"""
Created on Tue Oct 30 15:18:59 2018

@author: Romeo Orsolino
"""

import numpy as np
from scipy.spatial import ConvexHull
from jet_leg.computational_geometry.math_tools import Math

class Geometry(Math):
    
    def clockwise_sort(self, polygon):
        ''' we assume here that the vertices are on the rows'''
        vertices_number = np.size(polygon,0)
        angle = np.zeros((1,vertices_number))
        centroidX = np.sum(polygon[:,0])
        centroidX = centroidX/float(vertices_number)
        centroidY = np.sum(polygon[:,1])
        centroidY = centroidY/float(vertices_number)
        for j in range(0,vertices_number):
            angle[0,j] = np.arctan2(polygon[j,0] - centroidX, polygon[j,1] - centroidY)
        
        index = np.array(np.argsort(angle))
        sorted_vertices = np.zeros((vertices_number,2))
        for j in range(0,vertices_number):
            sorted_vertices[j,:] = polygon[index[0,j],:]       
        
        # adding an extra point to close the polytop (last point equal to the first)
        sorted_vertices = np.vstack([sorted_vertices, sorted_vertices[0,:]])
        
        return sorted_vertices

    def compute_halfspaces_convex_hull(self, vertices):
        ''' we assume the vertices matrix contains all the vertices on the rows. Therefore the size of
        vertices will be N x 2 where N is the number of vertices'''
        vertices = self.clockwise_sort(vertices)

        hull = ConvexHull(vertices)
        facets = hull.equations
        ''' if the offset term is positive it means that the corresponding line refers to >= .'''
        #flipped_facets = self.flip_normal_inwards(facets, vertices)
        flipped_facets = facets
        return flipped_facets

    def flip_normal_inwards(self, facets, vertices):
        '''This function makes sure that the cross product of the normal to one facets times
        the corresponding vertices is always positive. In this way we make sure that if we multiply the facets matrix
        times a point inside the polygon we always get a negative value (distance between the point and the facets). '''
        numberOfFacets = np.shape(facets)[0]
        vertices_number = numberOfFacets # this is true only for 2D polygons
        A = facets[:, :-1]
        b = facets[:, -1]

        centroidX = np.sum(vertices[:-1,0]) # sum all the rows but the last one (the last vertex is a duplicate of the first one)
        centroidX = centroidX/float(vertices_number)
        centroidY = np.sum(vertices[:-1,1])
        centroidY = centroidY/float(vertices_number)
        centroid = [centroidX, centroidY] # the centroid is always inside the polygon, therefore its distance from the edges has to be always negative
        print vertices_number, centroid
        new_facets = np.zeros((numberOfFacets, 3))
        for j in np.arange(0, numberOfFacets):
            #normalVec = self.normalize(A[j])
            normalVec = A[j]
            distance = np.dot(normalVec, centroid) + b[j]
            print "old distance", distance
            if distance >= 0:
                new_facets[j,0:2] = -A[j]
                new_facets[j, 2] = -b[j]
            else:
                new_facets[j,0:2] = A[j]
                new_facets[j, 2] = b[j]
            print "new distance", np.dot(new_facets[j,0:2], centroid) + new_facets[j,2]

        return new_facets


    def compute_halfspaces(self, vertices):
        ''' we assume the vertices matrix contains all the vertices on the rows. Therefore the size of
        vertices will be N x 2 where N is the number of vertices'''

        vertices = self.clockwise_sort(vertices)
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