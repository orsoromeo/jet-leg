# -*- coding: utf-8 -*-
"""
Created on Tue Oct 30 15:18:59 2018

@author: Romeo Orsolino
"""

import numpy as np

class Geometry:
    
    def clockwise_sort(self, polygon):
        
        vertices_number = np.size(polygon,0)
        angle = np.zeros((1,vertices_number))
#        print polygon, vertices_number
        for j in range(0,vertices_number):
#            print j
            angle[0,j] = np.arctan2(polygon[j,0], polygon[j,1])
        
        index = np.array(np.argsort(angle))
#        print index
        sorted_vertices = np.zeros((vertices_number,2))
        for j in range(0,vertices_number):
#            print j, index[0,j]
            sorted_vertices[j,:] = polygon[index[0,j],:]       
        
        sorted_vertices = np.vstack([sorted_vertices, sorted_vertices[0,:]])
        
        return sorted_vertices