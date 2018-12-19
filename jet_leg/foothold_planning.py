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
        self.math = Math()
        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()
        
    def selectMaximumFeasibleArea(self, footPlanningParams, params):        

        #foothold planning
        #overwite the position for the actual swing, then the future polygon of stance will be evaluated with that point
#        print 'foothold options',params.footOptions
                    
   
        params.setCoMPosWF(footPlanningParams.com_position_to_validateW)
        
#        print "com pos to validate" , params.com_position_to_validateW
#        print "sample contacts" , params.sample_contacts
        
        footPlanningParams.numberOfFeetOptions = np.size(footPlanningParams.footOptions,0)
#        print numberOfFeetOptions
        feasible_regions = []
        area = []
        for i in range(0, footPlanningParams.numberOfFeetOptions):
            #these two lines go together to overwrite the future swing foot
                     
            params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[i]
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
        
    def selectMinumumRequiredFeasibleAreaResidualRadius(self,  footPlanningParams, params):        

        #foothold planning
        #overwite the position for the actual swing, then the future polygon of stance will be evaluated with that point
#        print 'foothold options',params.footOptions
                        
       
        ng = 4
        constraint_mode_IP = 'FRICTION_AND_ACTUATION'
        params.setConstraintModes([constraint_mode_IP,
                           constraint_mode_IP,
                           constraint_mode_IP,
                           constraint_mode_IP])
        params.setNumberOfFrictionConesEdges(ng)

            
        params.setCoMPosWF(footPlanningParams.com_position_to_validateW)
    
        
#        print "com pos to validate" , params.com_position_to_validateW
#        print "sample contacts" , params.sample_contacts
        
#        print numberOfFeetOptions
        feasible_regions = []
        residualRadiusToStack = []
#        print 'empty res radii', residualRadiusToStack
#        footOptions = []
        area = []
        mapFootHoldIdxToPolygonIdx = []
        
        counter = 0
        numberOfOptions = np.size(footPlanningParams.footOptions,0)
        
        #check the prediction point at the beginning
        foothold_index  = int((numberOfOptions -1)/2.0) #assumes numberOfOptions is odd
#        print 'initial foothold index', foothold_index
      
    
        
        #these two lines go together to overwrite the future swing foot
        params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[foothold_index]
#        print 'contacts WF', params.contactsWF
#        print 'com pos WF', params.getCoMPosWF()
        IAR, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
#        print 'IAR', IAR
        residualRadius = deepcopy(self.math.find_residual_radius(IAR, footPlanningParams.com_position_to_validateW))
        area.append( self.compGeo.computePolygonArea(IAR))
        mapFootHoldIdxToPolygonIdx.append(foothold_index)
#        footOptions.append(deepcopy(params.contactsWF[params.actual_swing] ))
     
        feasible_regions.append(IAR)
        residualRadiusToStack.append(residualRadius)
        if residualRadius < footPlanningParams.minRadius:
            
            #check the fist point after and before the heuristic one along the direction

            #these two lines go together to overwrite the future swing foot
            params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[foothold_index+1] 
            IAR1, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
           
            newResidualRadius1 = deepcopy(self.math.find_residual_radius(IAR1, footPlanningParams.com_position_to_validateW))
            searchDirection1 = +1
            area.append( self.compGeo.computePolygonArea(IAR1))
#            footOptions.append(deepcopy(params.contactsWF[params.actual_swing]))
            feasible_regions.append(IAR1)
            residualRadiusToStack.append(newResidualRadius1)
            mapFootHoldIdxToPolygonIdx.append(foothold_index+1)
            
            #these two lines go together to overwrite the future swing foot
            params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[foothold_index-1] 
            IAR2, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
         
            newResidualRadius2 = deepcopy(self.math.find_residual_radius(IAR2, footPlanningParams.com_position_to_validateW))
            searchDirection2 = -1
           
            area.append( self.compGeo.computePolygonArea(IAR2))
            mapFootHoldIdxToPolygonIdx.append(foothold_index-1)
#            footOptions.append(deepcopy(params.contactsWF[params.actual_swing]))
            feasible_regions.append(IAR2)
            residualRadiusToStack.append(newResidualRadius2)
#            print 'RADS', residualRadius, newResidualRadius1, newResidualRadius2
            if (newResidualRadius1 > (residualRadius+footPlanningParams.TOL)) and (newResidualRadius1 > (newResidualRadius2+footPlanningParams.TOL)) : 
                searchDirection = searchDirection1
                residualRadius = newResidualRadius1
            elif (newResidualRadius2 > (residualRadius+footPlanningParams.TOL))  and (newResidualRadius2 > (newResidualRadius1+footPlanningParams.TOL)) :
                searchDirection = searchDirection2
                residualRadius = newResidualRadius2
            else: #you are already in the max
                searchDirection = 0
#                print 'final foothold index', foothold_index
                print 'RETURN before entering while loop'
                return foothold_index, residualRadiusToStack, feasible_regions, mapFootHoldIdxToPolygonIdx
                            
                
            
            foothold_index += searchDirection
            params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[foothold_index] 
            print 'number of option',numberOfOptions  
            #move along the grid to find the feasible point 
            while ((residualRadius < footPlanningParams.minRadius) and (foothold_index > 0) and (foothold_index < numberOfOptions-1)):
#        for i in range(0, footPlanningParams.numberOfFeetOptions):           
               
                #overwrite the future swing foot
#                params.contactsWF[params.actual_swing] = np.subtract(params.contactsWF[params.actual_swing], [0.02, 0.0, 0.0])
                foothold_index += searchDirection

                #these two lines go together to overwrite the future swing foot
                params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[foothold_index]  
                IAR, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
                residualRadius = self.math.find_residual_radius(IAR, footPlanningParams.com_position_to_validateW)
                
#                gradient = (residualRadius - newResidualRadius)/gridResolution
                area.append( self.compGeo.computePolygonArea(IAR))
                mapFootHoldIdxToPolygonIdx.append(foothold_index)
#                footOptions.append(deepcopy(params.contactsWF[params.actual_swing] ))
            
                print 'min radis ', footPlanningParams.minRadius
                feasible_regions.append(IAR)
#                print 'FR', feasible_regions
                residualRadiusToStack.append(residualRadius)
                
                counter += 1
        print 'res radii', residualRadiusToStack
        print 'foothold index ', foothold_index
#        footPlanningParams.option_index = foothold_index
        return foothold_index, residualRadiusToStack, feasible_regions, mapFootHoldIdxToPolygonIdx
        