# -*- coding: utf-8 -*-
"""
Created on Thu Aug 16 14:27:10 2018

@author: romeoorsolino

This code performs a test on the exploration of the actuation region along an arbitrary direction.
The direction is given by the variable "angle" that tell the desired yaw direction of the robot; 
the other angles are assumed here to be zero.

The same test can be repeated N number of times to emulate the estimation of the actuation region which 
requires the solution of this problem multiple times along different directions.

The use can change the following parameters:

1- comWF: position of the Center of Mass wrt to the world frame

2- contstraint_mode: type of constraint to be applied on the contact forces (this 
can be either 'ONLY_ACTUATION' or only 'ONLY_FRICTION')

3- N: number of times that the execution of the variable constraints search is called

4- yaw_angle: desired yaw angle in which we wish the robot to move (in radiants)
"""

import numpy as np

from context import jet_leg 
from jet_leg.sequential_iterative_projection import SequentialIterativeProjection

'''User parameters'''
comWF = np.array([0.2, 0.05, 0.0])
constraint_mode = 'ONLY_ACTUATION'
N = 1;
yaw_angle = 0.04



'''Implementation '''
print "angle is: ", yaw_angle
dir_x = np.cos(yaw_angle)
dir_y = np.sin(yaw_angle)
desired_direction = np.array([dir_x, dir_y, 0.0])

numberOfContactPoints = 3;
""" contact points """
LF_foot = np.array([0.4, 0.3, -0.5])
RF_foot = np.array([0.4, -0.3, -0.5])
LH_foot = np.array([-0.4, 0.3, -0.5])      
RH_foot = np.array([-0.4, -0.3, -0.5])

contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts = contactsToStack[0:numberOfContactPoints, :]

sequentialIP = SequentialIterativeProjection()
for iter in range(0,N):
    intersection_point = sequentialIP.optimize_direction_variable_constraints(constraint_mode, desired_direction, contacts, comWF)
    print "intersection points: ", intersection_point