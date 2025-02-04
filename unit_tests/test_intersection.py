# -*- coding: utf-8 -*-
"""
Created on Wed Aug 15 19:22:30 2018

@author: romeoorsolino
"""

from context import jet_leg 
from jet_leg.math_tools import Math
import numpy as np
import matplotlib.pyplot as plt
import unittest

class TestStringMethods(unittest.TestCase):

    epsilon = 10e-06
    
    def test_square_line_intersection_predefined_values(self):
        math = Math()
        #starting_point = np.array([np.random.randint(0,100)/100.0,np.random.randint(0,100)/100.0,0.0])
            
        starting_point = np.array([0.0,0.0,0.0])
        #print "starting point is: ",starting_point
        #angle = np.random.randint(0,600)/600.0
        search_direction = np.array([1.0,1.0,0.0])
        vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
        new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
        expected_intersection_point = np.array([1.0, 1.0])
        self.assertTrue((new_p - expected_intersection_point < self.epsilon).all()) 

        search_direction = np.array([-1.0,1.0,0.0])
        vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
        new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
        expected_intersection_point = np.array([-1.0, 1.0])
        self.assertTrue((new_p - expected_intersection_point < self.epsilon).all()) 

        search_direction = np.array([1.0,-1.0,0.0])
        vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
        new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
        expected_intersection_point = np.array([1.0, -1.0])
        self.assertTrue((new_p - expected_intersection_point < self.epsilon).all()) 

        search_direction = np.array([-1.0,-1.0,0.0])
        vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
        new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
        expected_intersection_point = np.array([-1.0, -1.0])
        self.assertTrue((new_p - expected_intersection_point < self.epsilon).all()) 
        
        search_direction = np.array([1.0,0.0,0.0])
        vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
        new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
        expected_intersection_point = np.array([1.0, 0.0])
        self.assertTrue((new_p - expected_intersection_point < self.epsilon).all())         
        
        search_direction = np.array([0.0,1.0,0.0])
        vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
        new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
        expected_intersection_point = np.array([0.0, 1.0])
        self.assertTrue((new_p - expected_intersection_point < self.epsilon).all()) 

        search_direction = np.array([0.0,-1.0,0.0])
        vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
        new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
        expected_intersection_point = np.array([0.0, -1.0])
        self.assertTrue((new_p - expected_intersection_point < self.epsilon).all()) 

        search_direction = np.array([-1.0,0.0,0.0])
        vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
        new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
        expected_intersection_point = np.array([-1.0, 0.0])
        self.assertTrue((new_p - expected_intersection_point < self.epsilon).all())         
        return new_p, all_points
        
    def test_square_line_intersection_random_values(self):
        math = Math()
        
        for iter in range(0,10):
            starting_point = np.array([np.random.randint(-100,100)/100.0,np.random.randint(-100,100)/100.0,0.0])
            
            #starting_point = np.array([0.0,0.0,0.0])
            #print "starting point is: ",starting_point
            #angle = np.random.randint(0,600)/600.0
            search_direction = np.array([1.0,1.0,0.0])
            vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
            new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
            expected_intersection_point = np.array([1.0, 1.0])
            self.assertTrue((new_p[1] >= -new_p[0]).all())

            
        for iter in range(0,10):
            starting_point = np.array([np.random.randint(-100,100)/100.0,np.random.randint(-100,100)/100.0,0.0])
            
            #starting_point = np.array([0.0,0.0,0.0])
            #print "starting point is: ",starting_point
            #angle = np.random.randint(0,600)/600.0
            search_direction = np.array([-1.0,-1.0,0.0])
            vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
            new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
            expected_intersection_point = np.array([1.0, 1.0])
            self.assertTrue((new_p[1] <= -new_p[0]).all())
        
        for iter in range(0,10):
            starting_point = np.array([np.random.randint(-100,100)/100.0,np.random.randint(-100,100)/100.0,0.0])
            
            #starting_point = np.array([0.0,0.0,0.0])
            #print "starting point is: ",starting_point
            #angle = np.random.randint(0,600)/600.0
            search_direction = np.array([1.0,0.0,0.0])
            vertices = np.array([[1.0,1.0,0.0],
                             [1.0,-1.0,0.0],
                            [-1.0,-1.0,0.0],
                            [-1.0,1.0,0.0]])
            new_p, all_points = math.find_polygon_segment_intersection(vertices, search_direction, starting_point)
            expected_intersection_point = np.array([1.0, 1.0])
            self.assertTrue((new_p[0] > 0).all())
        
'''main'''
#point, all_points = test_square_line_intersection()
#print 'all points : ',all_points, point
#plt.grid()
#plt.plot(all_points[:,0], all_points[:,1], 'bo')
#plt.plot(point[0], point[1], 'r^', markersize = 20)    