#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  3 13:38:45 2018

@author: Romeo Orsolino
"""

import copy
import numpy as np
import os

import rospy as ros
import sys
import time
import threading

from copy import deepcopy

from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Vector3, Wrench
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point
from dls_msgs.msg import SimpleDoubleArray, StringDoubleArray, Polygon3D, LegsPolygons
from dwl_msgs.msg import WholeBodyState, WholeBodyTrajectory, JointState, ContactState, BaseState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Header
from std_srvs.srv import Empty
from termcolor import colored

#from context import jet_leg 
from computational_dynamics import ComputationalDynamics
from computational_geometry import ComputationalGeometry
from foothold_planning_interface import FootholdPlanningInterface
from math_tools import Math
from dog_interface import DogInterface
from rigid_body_dynamics import RigidBodyDynamics


stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class FootHoldPlanning:
    def __init__(self):      
        self.compGeo = ComputationalGeometry()
        self.compDyn = ComputationalDynamics()
        self.footPlanning = FootholdPlanningInterface()
        self.option_index = 0
        self.ack_optimization_done = False
        self.math = Math()
        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()
        
    def selectMaximumFeasibleArea(self, footPlanningParams, params):        

        #foothold planning
        #overwite the position for the actual swing, then the future polygon of stance will be evaluated with that point
#        print 'foothold options',params.footOptions
                    
        footPlanningParams.sample_contacts = deepcopy(params.contactsWF )
        params.setCoMPosWF(footPlanningParams.com_position_to_validateW)
        
#        print "com pos to validate" , params.com_position_to_validateW
#        print "sample contacts" , params.sample_contacts
        
        footPlanningParams.numberOfFeetOptions = np.size(footPlanningParams.footOptions,0)
#        print numberOfFeetOptions
        feasible_regions = []
        area = []
        for i in range(0, footPlanningParams.numberOfFeetOptions):
            params.contactsWF  =  deepcopy(footPlanningParams.sample_contacts)
            
            
            #overwrite the future swing foot
            params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[i]
            #print params.footOptions[i]
 
            IAR, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
#            print 'IAR', IAR
            d = self.math.find_residual_radius(IAR, footPlanningParams.com_position_to_validateW)
            print 'residual radius', d
            feasible_regions.append(IAR)
#            print 'FR', feasible_regions
            area.append( self.compGeo.computePolygonArea(IAR))
        
        print 'area ', area
        print 'max arg ',np.argmax(np.array(area), axis=0)
        return np.argmax(np.array(area), axis=0), feasible_regions
        
    def selectMinumumRequiredFeasibleAreaResidualRadius(self, requiredMinimumResidualRadius, footPlanningParams, params, max_iter = 10):        

        #foothold planning
        #overwite the position for the actual swing, then the future polygon of stance will be evaluated with that point
#        print 'foothold options',params.footOptions
                    
        footPlanningParams.sample_contacts = deepcopy(params.contactsWF)
        params.setCoMPosWF(footPlanningParams.com_position_to_validateW)
        params.contactsWF  =  deepcopy(footPlanningParams.sample_contacts)
        
#        print "com pos to validate" , params.com_position_to_validateW
#        print "sample contacts" , params.sample_contacts
        
#        print numberOfFeetOptions
        feasible_regions = []
        residualRadiusToStack = []
        footOptions = []
        area = []

        counter = 0
        params.contactsWF[params.actual_swing] = footPlanningParams.heuristicFoothold
        print 'contacts before while',footPlanningParams.heuristicFoothold, params.contactsWF
        IAR, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
#            print 'IAR', IAR
        residualRadius = deepcopy(self.math.find_residual_radius(IAR, footPlanningParams.com_position_to_validateW))
        area.append( self.compGeo.computePolygonArea(IAR))
        footOptions.append(deepcopy(params.contactsWF[params.actual_swing] ))
        print 'residual radius', residualRadius
        feasible_regions.append(IAR)
        residualRadiusToStack.append(residualRadius)
        if residualRadius < requiredMinimumResidualRadius:
            gridResolution = 0.04   
        
            params.contactsWF[params.actual_swing][self.rbd.LX] = footPlanningParams.heuristicFoothold[self.rbd.LX] + gridResolution
            IAR1, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
            print 'contacts before while',footPlanningParams.heuristicFoothold, params.contactsWF
            newResidualRadius1 = deepcopy(self.math.find_residual_radius(IAR1, footPlanningParams.com_position_to_validateW))
            searchDirection1 = +1
            area.append( self.compGeo.computePolygonArea(IAR1))
            footOptions.append(deepcopy(params.contactsWF[params.actual_swing]))
            feasible_regions.append(IAR1)
            residualRadiusToStack.append(newResidualRadius1)
            
            params.contactsWF[params.actual_swing][self.rbd.LX] = footPlanningParams.heuristicFoothold[self.rbd.LX] - gridResolution
            IAR2, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
            print 'contacts before while',footPlanningParams.heuristicFoothold, params.contactsWF
            newResidualRadius2 = deepcopy(self.math.find_residual_radius(IAR2, footPlanningParams.com_position_to_validateW))
            searchDirection2 = -1
            print 'tested radiuses', newResidualRadius1, newResidualRadius2
            area.append( self.compGeo.computePolygonArea(IAR2))
            footOptions.append(deepcopy(params.contactsWF[params.actual_swing]))
            feasible_regions.append(IAR2)
            residualRadiusToStack.append(newResidualRadius2)
        
            if newResidualRadius1 > newResidualRadius2: 
                searchDirection = searchDirection1
            else: searchDirection = searchDirection2
        
            params.contactsWF[params.actual_swing][self.rbd.LX] = footPlanningParams.heuristicFoothold[self.rbd.LX] + searchDirection*gridResolution
        
            while (residualRadius < requiredMinimumResidualRadius) and (counter<max_iter):
#        for i in range(0, footPlanningParams.numberOfFeetOptions):           
            
                #overwrite the future swing foot
#                params.contactsWF[params.actual_swing] = np.subtract(params.contactsWF[params.actual_swing], [0.02, 0.0, 0.0])
                footPlanningParams.heuristicFoothold[self.rbd.LX] += searchDirection*gridResolution
                params.contactsWF[params.actual_swing][self.rbd.LX] = footPlanningParams.heuristicFoothold[self.rbd.LX] + searchDirection*gridResolution
                print 'contacts',params.contactsWF
                IAR, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
                residualRadius = deepcopy(self.math.find_residual_radius(IAR, footPlanningParams.com_position_to_validateW))
                
#                gradient = (residualRadius - newResidualRadius)/gridResolution
                area.append( self.compGeo.computePolygonArea(IAR))
                footOptions.append(deepcopy(params.contactsWF[params.actual_swing] ))
                print 'residual radius', residualRadius
                feasible_regions.append(IAR)
#                print 'FR', feasible_regions
                residualRadiusToStack.append(residualRadius)
                
                counter += 1
        
        print 'residualRadiusToStack ', residualRadiusToStack
        return residualRadiusToStack, feasible_regions, footOptions
        