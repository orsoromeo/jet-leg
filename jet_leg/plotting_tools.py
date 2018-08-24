# -*- coding: utf-8 -*-
"""
Created on Mon May 28 12:36:24 2018

@author: rorsolino
"""

import matplotlib.pyplot as plt
import numpy as np
from computational_geometry import ComputationalGeometry

class Plotter:
    
  def plot_polygon(self, points, color = '--b', Label = ''):
      if np.size(points,1)==2:
          x = np.hstack([points[:,0], points[0,0]])
          y = np.hstack([points[:,1], points[0,1]])
      else:
          x = np.hstack([points[0,:], points[0,0]])
          y = np.hstack([points[1,:], points[1,0]])
          
      plt.plot(x, y, color, linewidth=10., label = Label)
    
  def plot_line(self, ax, line_coefficients):
      a = line_coefficients[0]
      b = line_coefficients[1]
      c = line_coefficients[2]          
      x1 = -2
      y1 = -a/b*x1 - c/b
 
      x2 = 2
      y2 = -a/b*x2 - c/b
      x = np.array([x1, x2])
      y = np.array([y1, y2])      
      ax.plot(x,y) 

  def plot_actuation_region(self, ax, line_coefficients):
      edges_number = np.size(line_coefficients,1)
      for j in range(0, edges_number):
          self.plot_line(ax, line_coefficients[:,j])     
      
  def plot_facet(self,ax,facet):
      facet_x = facet[0,:]
      facet_y = facet[1,:]
      facet_z = facet[2,:]
      surf = ax.plot_trisurf(facet_x, facet_y, facet_z,  linewidth=0., alpha = 0.3)
      surf = ax.plot_wireframe(facet_x, facet_y, facet_z, linewidth=1.) 
  
  def plot_cube(self,ax,vertices):
      geom = ComputationalGeometry()
      face1, face2, face3, face4, face5, face6 = geom.get_facets(vertices)
      self.plot_facet(ax,face1)
      self.plot_facet(ax,face2) 
      self.plot_facet(ax,face3)
      self.plot_facet(ax,face4)
      self.plot_facet(ax,face5)
      self.plot_facet(ax,face6)
      
  def plot_actuation_polygon(self, ax, vertices, foot_pos, scaling_factor = 2000):
      vertex = np.zeros((3,8))
      for j in range(0,8):
          vertex[0,j] = float(vertices[0,j])/float(scaling_factor)
          vertex[1,j] = vertices[1,j]/float(scaling_factor)
          vertex[2,j] = vertices[2,j]/float(scaling_factor)
          vertex[:,j] = np.add(vertex[:,j],np.transpose(foot_pos))
          
        
          
      self.plot_cube(ax,vertex)