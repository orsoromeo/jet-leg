# -*- coding: utf-8 -*-
"""
Created on Thu Aug 16 14:27:10 2018

@author: romeoorsolino
"""

import numpy as np

from context import jet_leg 
from jet_leg.sequential_iterative_projection import SequentialIterativeProjection

nc = 3
comWF = np.array([0.2, 0.05, 0.0])
constraint_mode = 'ONLY_ACTUATION'

angle = 0.12
print "angle is: ", angle
dir_x = np.cos(angle)
dir_y = np.sin(angle)
desired_direction = np.array([dir_x, dir_y, 0.0])
#print "direction: ", desired_direction
""" contact points """
LF_foot = np.array([0.4, 0.3, -0.5])
RF_foot = np.array([0.4, -0.3, -0.5])
LH_foot = np.array([-0.4, 0.3, -0.5])
#print "Terrain height: ", LH_foot        
RH_foot = np.array([-0.4, -0.3, -0.5])
contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts = contactsToStack[0:nc, :]
sequentialIP = SequentialIterativeProjection()

for iter in range(0,50):
    intersection_point = sequentialIP.optimize_direction_variable_constraints(constraint_mode, desired_direction, contacts, comWF)
    print "intersection points: ", intersection_point