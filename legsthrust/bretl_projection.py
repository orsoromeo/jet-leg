# -*- coding: utf-8 -*-
"""
Created on Mon Jun  4 15:45:43 2018

@author: Romeo Orsolino

"""
import numpy as np

r1 = array([10.0, 1.0, 0.0])
r2 = array([-10.0,-10.0, 0.0])
r3 = array([-10.0,10.0, 0.0])
''' init outer and inner approximation'''
outer_approx = np.vstack([r1, r2, r3])
inner_approx = np.zeros((0,3))
points_num = np.size(outer_approx,0)
for j in range(0, points_num-1):
    avg = (outer_approx[j,:] + outer_approx[j+1,:])/2.
    inner_approx = np.vstack([inner_approx, avg])

avg = (outer_approx[points_num-1,:] + outer_approx[0,:])/2.
inner_approx = np.vstack([inner_approx, avg])

''' get the edges from the v-description '''


print inner_approx
