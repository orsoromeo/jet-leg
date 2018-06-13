# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 09:43:27 2018

@author: romeo orsolino
"""
import numpy as np

class Math:
    def normalize(self, n):
        norm1 = np.linalg.norm(n)
        n = np.true_divide(n, norm1)
        return n
    
    def skew(self, v):
        if len(v) == 4: v = v[:3]/v[3]
        skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
        return skv - skv.T
    
    def rotation_matrix_from_normal(self, n):
        n = n.reshape((3,))
        e_x = np.array([1., 0., 0.])
        t = e_x - np.dot(e_x, n) * n
        t = t / np.linalg.norm(t)
        b = np.cross(n, t)
        return np.vstack([t, b, n]).T
        
    def getGraspMatrix(self, r):
        math = Math()
        G = block([[np.eye(3), np.zeros((3, 3))],
                       [math.skew(r), np.eye(3)]])
        return G    


    def rpyToRot(self, roll, pitch, yaw):
    

        
        Rx =  np.array([ [   1   ,    0     	  ,  	  0], 
                         [0   ,    np.cos(roll) ,  np.sin(roll)],
                         [0   ,    -np.sin(roll),  np.cos(roll)]]);
               
        
        Ry = np.array([[np.cos(pitch) 	,	 0  ,   -np.sin(pitch)],
              [      0       ,    1  ,   0],
              [np.sin(pitch) 	,	0   ,  np.cos(pitch)]]);
          
        
        Rz = np.array([[ np.cos(yaw)  ,  np.sin(yaw) ,		0],
                      [-np.sin(yaw) ,  np.cos(yaw) ,  		0],
                      [0      ,     0     ,       1]]);
        
        
        R =  Rx.dot(Ry.dot(Rz));
        return R