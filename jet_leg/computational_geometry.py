# -*- coding: utf-8 -*-
"""
Created on Mon May 28 13:05:01 2018

@author: rorsolino
"""
import numpy as np 

class ComputationalGeometry:
    
    def get_facets(self, v_rep):
        vx = v_rep[0,0:8]
        vy = v_rep[1,0:8]
        vz = v_rep[2,0:8]
        face1 = np.vstack([np.hstack([vx[0:4],vx[0]]),
                         np.hstack([vy[0:4],vy[0]]),
                         np.hstack([vz[0:4],vz[0]])])
        face2 = np.vstack([np.hstack([vx[4:8],vx[4]]),
                         np.hstack([vy[4:8],vy[4]]),
                        np.hstack([vz[4:8],vz[4]])])
        face3 = np.vstack([np.hstack([vx[0],vx[1],vx[5],vx[4],vx[0]]),
                         np.hstack([vy[0],vy[1],vy[5],vy[4],vy[0]]),
                         np.hstack([vz[0],vz[1],vz[5],vz[4],vz[0]])])
        face4 = np.vstack([np.hstack([vx[1],vx[2],vx[6],vx[5],vx[1]]),
                         np.hstack([vy[1],vy[2],vy[6],vy[5],vy[1]]),
                         np.hstack([vz[1],vz[2],vz[6],vz[5],vz[1]])])
        face5 = np.vstack([np.hstack([vx[2],vx[3],vx[7],vx[6],vx[2]]),
                         np.hstack([vy[2],vy[3],vy[7],vy[6],vy[2]]),
                         np.hstack([vz[2],vz[3],vz[7],vz[6],vz[2]])])
        face6 = np.vstack([np.hstack([vx[0],vx[3],vx[7],vx[4],vx[0]]),
                         np.hstack([vy[0],vy[3],vy[7],vy[4],vy[0]]),
                         np.hstack([vz[0],vz[3],vz[7],vz[4],vz[0]])])
        return face1, face2, face3, face4, face5, face6
        
    def get_halfspace_rep(self, v_rep):
        face1, face2, face3, face4, face5, face6 = self.get_facets(v_rep)
        edge1 = face1[:,2] - face1[:,1]
        edge2 = face1[:,0] - face1[:,1]
        normal = np.cross(edge1, edge2)
        known_term = np.dot(normal, face1[:,0])
        h_rep1 = np.hstack([normal, known_term])
        
        edge1 = face2[:,2] - face2[:,1]
        edge2 = face2[:,0] - face2[:,1]
        normal = np.cross(edge1, edge2)
        known_term = np.dot(normal, face2[:,0])
        h_rep2 = np.hstack([normal, known_term])
 
        edge1 = face3[:,2] - face3[:,1]
        edge2 = face3[:,0] - face3[:,1]
        normal = np.cross(edge1, edge2)
        known_term = np.dot(normal, face3[:,0])
        h_rep3 = np.hstack([normal, known_term])
 
        edge1 = face4[:,2] - face4[:,1]
        edge2 = face4[:,0] - face4[:,1]
        normal = np.cross(edge1, edge2)
        known_term = np.dot(normal, face4[:,0])
        h_rep4 = np.hstack([normal, known_term])
 
        edge1 = face5[:,2] - face5[:,1]
        edge2 = face5[:,0] - face5[:,1]
        normal = np.cross(edge1, edge2)
        known_term = np.dot(normal, face5[:,0])
        h_rep5 = np.hstack([normal, known_term])
 
        edge1 = face6[:,2] - face6[:,1]
        edge2 = face6[:,0] - face6[:,1]
        normal = np.cross(edge1, edge2)
        known_term = np.dot(normal, face6[:,0])
        h_rep6 = np.hstack([normal, known_term])
 
        return h_rep1, h_rep2, h_rep3, h_rep4, h_rep5, h_rep6
        
    def computePolygonArea(self, v_rep):
        v_rep = np.transpose(v_rep)        
#        print 'v rep ', v_rep
        # Initialze area
        area = 0.0;
        num_of_vertices = np.size(v_rep,1)
#        print num_of_vertices
        #Calculate value of shoelace formula
        j = num_of_vertices - 1;
        for i in range(0, num_of_vertices):
            area += (v_rep[0,j] + v_rep[0,i]) * (v_rep[1,j] - v_rep[1,i]); 
            j = i;   #j is previous vertex to i
    
        # Return absolute value
        return np.abs(area / 2.0)
        