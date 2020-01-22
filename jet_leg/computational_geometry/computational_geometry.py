# -*- coding: utf-8 -*-
"""
Created on Mon May 28 13:05:01 2018

@author: rorsolino
"""
import numpy as np
from jet_leg.computational_geometry.geometry import Geometry

class ComputationalGeometry(Geometry):

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
        
    def get_hexahedron_halfspace_rep(self, v_rep):
        face1, face2, face3, face4, face5, face6 = self.get_facets(v_rep)
        edge1 = face1[:,2] - face1[:,1]
        edge2 = face1[:,0] - face1[:,1]
        normal = np.cross(edge1, edge2)
        norm = np.linalg.norm(normal)
        normal = normal/norm
        known_term = np.dot(normal, face1[:,0])
        h_rep1 = np.hstack([normal, known_term])
        
        edge1 = face2[:,2] - face2[:,1]
        edge2 = face2[:,0] - face2[:,1]
        normal = np.cross(edge1, edge2)
        norm = np.linalg.norm(normal)
        normal = normal/norm
        known_term = np.dot(normal, face2[:,0])
        h_rep2 = np.hstack([normal, known_term])
 
        edge1 = face3[:,2] - face3[:,1]
        edge2 = face3[:,0] - face3[:,1]
        normal = np.cross(edge1, edge2)
        norm = np.linalg.norm(normal)
        normal = normal/norm
        known_term = np.dot(normal, face3[:,0])
        h_rep3 = np.hstack([normal, known_term])
 
        edge1 = face4[:,2] - face4[:,1]
        edge2 = face4[:,0] - face4[:,1]
        normal = np.cross(edge1, edge2)
        norm = np.linalg.norm(normal)
        normal = normal/norm
        known_term = np.dot(normal, face4[:,0])
        h_rep4 = np.hstack([normal, known_term])
 
        edge1 = face5[:,2] - face5[:,1]
        edge2 = face5[:,0] - face5[:,1]
        normal = np.cross(edge1, edge2)
        #print edge1, edge2, normal
        norm = np.linalg.norm(normal)
        #print "norms ", norm, normal
        normal = normal/norm
        #print normal
        known_term = np.dot(normal, face5[:,0])
        h_rep5 = np.hstack([normal, known_term])
 
        edge1 = face6[:,2] - face6[:,1]
        edge2 = face6[:,0] - face6[:,1]
        normal = np.cross(edge1, edge2)
        norm = np.linalg.norm(normal)
        normal = normal/norm
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

    def isPointRedundant(self, facets, point2check):
        ''' This code assumes that the facets are given in the form [A|b] where:
                            A x < b                                            '''

        A = facets[:, :-1]
        b = facets[:, -1]
        distances_from_edges = np.dot(A, point2check) + b
        min_distance = min(-distances_from_edges)
        if min_distance < 0.0:
            isPointInside = False
        else:
            isPointInside = True
        return isPointInside, min_distance

    def isPointRedundantGivenVertices(self, IP_points, point2check):

        facets = self.compute_halfspaces_convex_hull(IP_points)
        isPointFeasible = self.isPointRedundant(facets, point2check)

        return isPointFeasible

    def computeFeasibleRegionBinaryMatrix(self, IP_points, resolutionX=0.01, resolutionY=0.01, windowSizeX=0.9,
                                          windowSizeY=0.7):

        binary_matrix = np.zeros((int(windowSizeY / resolutionY), int(windowSizeX / resolutionX)))
        margin_matrix = np.zeros((int(windowSizeY / resolutionY), int(windowSizeX / resolutionX)))
        feasible_points = np.zeros((0, 2))
        unfeasible_points = np.zeros((0, 2))

        facets = self.compute_halfspaces_convex_hull(IP_points)

        idX = 0
        for point2checkX in np.arange(-windowSizeX / 2.0, (windowSizeX / 2.0), resolutionX):
            idY = 0
            for point2checkY in np.arange(windowSizeY / 2.0, (-windowSizeY / 2.0), -resolutionY):
                point2check = np.array([point2checkX, point2checkY])
                isPointFeasible, margin = self.isPointRedundant(facets, point2check)
                # LPparams.setCoMPosWF(com_WF)
                # isPointFeasible, x = lpCheck.isPointRedundant(IP_points.T, point2check)
                margin_matrix[idY, idX] = margin
                if isPointFeasible:
                    binary_matrix[idY, idX] = 1
                    feasible_points = np.vstack([feasible_points, point2check])
                else:
                    binary_matrix[idY, idX] = 0
                    unfeasible_points = np.vstack([unfeasible_points, point2check])
                idY += 1
            idX += 1
        return binary_matrix, feasible_points, unfeasible_points, margin_matrix

    def computeGlobalFeasibleRegionBinaryMatrix(self, params, compDyn, resolutionX=0.01, resolutionY=0.01, windowSizeX=0.9,
                                          windowSizeY=0.7):

        matrix_sizeX = int(windowSizeX / resolutionY)+1
        matrix_sizeY = int(windowSizeY / resolutionY)+1
        print "matrix size", matrix_sizeX, matrix_sizeY
        binary_matrix = np.zeros((matrix_sizeY, matrix_sizeX))
        margin_matrix = np.zeros((matrix_sizeY, matrix_sizeX))
        feasible_points = np.zeros((0, 2))
        unfeasible_points = np.zeros((0, 2))

        idX = 0
        for point2checkX in np.arange(-windowSizeX / 2.0, windowSizeX / 2.0, resolutionX):
            idY = 0
            for point2checkY in np.arange(windowSizeY / 2.0, -windowSizeY / 2.0, -resolutionY):
                comWF = params.getCoMPosWF()
                comWF = np.array([point2checkX, point2checkY, comWF[2]])
                point2check = np.array([point2checkX, point2checkY])
                params.setCoMPosWF(comWF)
                IP_points, force_polytopes, IP_computation_time = compDyn.try_iterative_projection_bretl(params)
                print "IP points", IP_points
                if IP_points is not False:
                    facets = self.compute_halfspaces_convex_hull(IP_points)
                    isPointFeasible, margin = self.isPointRedundant(facets, point2check)
                else:
                    isPointFeasible = False
                    margin = -0.5
                # LPparams.setCoMPosWF(com_WF)
                # isPointFeasible, x = lpCheck.isPointRedundant(IP_points.T, point2check)
                print idX, idY
                margin_matrix[idY, idX] = margin
                if isPointFeasible:
                    binary_matrix[idY, idX] = 1
                    feasible_points = np.vstack([feasible_points, point2check])
                else:
                    binary_matrix[idY, idX] = 0
                    unfeasible_points = np.vstack([unfeasible_points, point2check])
                idY += 1
            idX += 1
        return binary_matrix, feasible_points, unfeasible_points, margin_matrix