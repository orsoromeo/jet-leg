# -*- coding: utf-8 -*-
"""
Created on Mon May 28 12:36:24 2018

@author: rorsolino
"""

import matplotlib.pyplot as plt
import numpy as np
from computational_geometry import ComputationalGeometry

class Plotter:
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
      #self.plot_facet(ax,face4)
      self.plot_facet(ax,face5)
      #self.plot_facet(ax,face6)
      
  def plot_actuation_polygon(self, ax, vertices, foot_pos):
      scaling_factor = 300
      for j in range(0,8):
          vertices[0,j] /= scaling_factor
          vertices[1,j] /= scaling_factor
          vertices[2,j] /= scaling_factor
          vertices[:,j] = np.add(vertices[:,j],np.transpose(foot_pos))
          
      self.plot_cube(ax,vertices)