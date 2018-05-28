# -*- coding: utf-8 -*-
"""
Created on Mon May 28 12:36:24 2018

@author: rorsolino
"""

import matplotlib.pyplot as plt
import numpy as np

class Plotter:
  def plot_facet(self,ax,facet):
      facet_x = facet[0,:]
      facet_y = facet[1,:]
      facet_z = facet[2,:]
      surf = ax.plot_trisurf(facet_x, facet_y, facet_z,  linewidth=0., alpha = 0.3)
      surf = ax.plot_wireframe(facet_x, facet_y, facet_z, linewidth=1.) 
  
  def plot_cube(self,ax,vertices):
      vx = vertices[0,0:8]
      vy = vertices[1,0:8]
      vz = vertices[2,0:8]
      face1 = np.vstack([np.hstack([vx[0:4],vx[0]]),
                         np.hstack([vy[0:4],vy[0]]),
                         np.hstack([vz[0:4],vz[0]])])
      self.plot_facet(ax,face1)
      face2 = np.vstack([np.hstack([vx[4:8],vx[4]]),
                         np.hstack([vy[4:8],vy[4]]),
                        np.hstack([vz[4:8],vz[4]])])
      self.plot_facet(ax,face2) 
      face3 = np.vstack([np.hstack([vx[0],vx[1],vx[5],vx[4],vx[0]]),
                         np.hstack([vy[0],vy[1],vy[5],vy[4],vy[0]]),
                         np.hstack([vz[0],vz[1],vz[5],vz[4],vz[0]])])
      self.plot_facet(ax,face3)
      face4 = np.vstack([np.hstack([vx[1],vx[2],vx[6],vx[5],vx[1]]),
                         np.hstack([vy[1],vy[2],vy[6],vy[5],vy[1]]),
                         np.hstack([vz[1],vz[2],vz[6],vz[5],vz[1]])])
      self.plot_facet(ax,face4)
      face5 = np.vstack([np.hstack([vx[2],vx[3],vx[7],vx[6],vx[2]]),
                         np.hstack([vy[2],vy[3],vy[7],vy[6],vy[2]]),
                         np.hstack([vz[2],vz[3],vz[7],vz[6],vz[2]])])
      self.plot_facet(ax,face5)
      face6 = np.vstack([np.hstack([vx[0],vx[3],vx[7],vx[4],vx[0]]),
                         np.hstack([vy[0],vy[3],vy[7],vy[4],vy[0]]),
                         np.hstack([vz[0],vz[3],vz[7],vz[4],vz[0]])])
      self.plot_facet(ax,face6)