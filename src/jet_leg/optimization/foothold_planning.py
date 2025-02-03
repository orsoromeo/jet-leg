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
from dls_msgs.msg import StringDoubleArray
from feasible_region.msg import Polygon3D, LegsPolygons
from dwl_msgs.msg import WholeBodyState, WholeBodyTrajectory, JointState, ContactState, BaseState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Header
from std_srvs.srv import Empty
from termcolor import colored

#from context import jet_leg 
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.maths.computational_geometry import ComputationalGeometry
from jet_leg.optimization.foothold_planning_interface import FootholdPlanningInterface
from jet_leg.maths.math_tools import Math
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics


stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class FootHoldPlanning:
    def __init__(self, robot_name):
        self.robotName = robot_name
        self.compGeo = ComputationalGeometry()
        self.compDyn = ComputationalDynamics(self.robotName)
        self.footPlanning = FootholdPlanningInterface()
        self.math = Math()
        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()
        
    def selectMaximumFeasibleArea(self, footPlanningParams, params):
   
        params.setCoMPosWF(footPlanningParams.com_position_to_validateW)
        
#        print "com pos to validate" , params.com_position_to_validateW
#        print "sample contacts" , params.sample_contacts
        
#        footPlanningParams.numberOfFeetOptions = np.size(footPlanningParams.footOptions,0)
        print 'number of feet options ',footPlanningParams.numberOfFeetOptions
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
        ng = 4
        params.setConstraintModes(['FRICTION_AND_ACTUATION',
                           'FRICTION_AND_ACTUATION',
                           'FRICTION_AND_ACTUATION',
                           'FRICTION_AND_ACTUATION'])
        params.setNumberOfFrictionConesEdges(ng)

        params.setCoMPosWF(footPlanningParams.com_position_to_validateW)

#        print numberOfFeetOptions
        feasible_regions = []
        residualRadiusToStack = []
#        print 'empty res radii', residualRadiusToStack
#        footOptions = []
        area = []
        mapFootHoldIdxToPolygonIdx = []
        
#        counter = 0
        print 'number of feet options ',footPlanningParams.numberOfFeetOptions
        numberOfOptions = footPlanningParams.numberOfFeetOptions
        
        #check the prediction point at the beginning
        if numberOfOptions > 0:
            foothold_index  = int((numberOfOptions -1)/2.0) #assumes numberOfOptions is odd
            #        print 'initial foothold index', foothold_index
            #these two lines go together to overwrite the future swing foot
            params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[foothold_index]
            #        print 'contacts WF', params.contactsWF
            #        print 'com pos WF', params.getCoMPosWF()
            IAR, actuation_polygons_array, computation_time = self.compDyn.try_iterative_projection_bretl(params)
            #        print 'IAR', IAR
            if IAR is False:
                return False, False, False, False 
            else:
                residualRadius = deepcopy(self.math.find_residual_radius(IAR, footPlanningParams.com_position_to_validateW))
                area.append( self.compGeo.computePolygonArea(IAR))
                mapFootHoldIdxToPolygonIdx.append(foothold_index)
                #            footOptions.append(deepcopy(params.contactsWF[params.actual_swing] ))
                feasible_regions.append(IAR)
                residualRadiusToStack.append(residualRadius)
                if residualRadius < footPlanningParams.minRadius:
                    gradient, searchDirection, residualRadius, foothold_index, residualRadiusToStack, feasible_regions, mapFootHoldIdxToPolygonIdx = self.compute_search_direciton(params, footPlanningParams, residualRadius, foothold_index, area,
                                             feasible_regions, mapFootHoldIdxToPolygonIdx, residualRadiusToStack)
                else:
                    gradient = False
                            
                if gradient is not False:
                   # print 'gradient before while', gradient
                    foothold_index += searchDirection
                    params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[foothold_index] 
                
            #            print 'number of option',numberOfOptions  
                #move along the grid to find the feasible point 
                    while ((gradient > 0.0) and (residualRadius < footPlanningParams.minRadius) and (foothold_index > 0) and (foothold_index < numberOfOptions-1)):
                        
                        #these two lines go together to overwrite the future swing foot
                        params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[foothold_index+searchDirection]  
                        IAR, actuation_polygons_array, computation_time = self.compDyn.try_iterative_projection_bretl(params)
                        if IAR is False:
                            residualRadius = 0.0
                            newArea = 0.0
                        else:
                            residualRadius = self.math.find_residual_radius(IAR, footPlanningParams.com_position_to_validateW)
                            newArea = self.compGeo.computePolygonArea(IAR)
                        oldArea = area[-1]
                        oldResidualRadius = residualRadiusToStack[-1]
                        #                   print 'old residual radius', oldResidualRadius
                        #                   gradient = residualRadius - oldResidualRadius
                        gradient = newArea - oldArea
                        #                   print 'area gradient ', gradient
                        #                   gradient = (residualRadius - newResidualRadius)/gridResolution
                        if gradient > 0:                
                            foothold_index += searchDirection
                            mapFootHoldIdxToPolygonIdx.append(foothold_index)
                            feasible_regions.append(IAR)
                            residualRadiusToStack.append(residualRadius)
                            area.append(newArea)
            print 'area ', area
            
        else:
            foothold_index = -1
#            feasible_regions = false
                    
                
#        print 'res radii', residualRadiusToStack
    
#            print 'foothold index ', foothold_index
#            footPlanningParams.option_index = foothold_index
        return foothold_index, residualRadiusToStack, feasible_regions, mapFootHoldIdxToPolygonIdx

    def compute_search_direciton(self, params, footPlanningParams, residualRadius, foothold_index, area, feasible_regions, mapFootHoldIdxToPolygonIdx, residualRadiusToStack):
        # check the fist point after and before the heuristic one along the direction
        # these two lines go together to overwrite the future swing foot
        if foothold_index < footPlanningParams.numberOfFeetOptions:
            params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[foothold_index + 1]
            #print "residualRadius, params.actual_swing, foothold_index, params.contactsWF", residualRadius, params.actual_swing, foothold_index, params.contactsWF
            IAR1, actuation_polygons_array, computation_time = self.compDyn.try_iterative_projection_bretl(params)
            newResidualRadius1 = deepcopy(
                self.math.find_residual_radius(IAR1, footPlanningParams.com_position_to_validateW))
            searchDirection1 = +1
            area.append(self.compGeo.computePolygonArea(IAR1))
            #            footOptions.append(deepcopy(params.contactsWF[params.actual_swing]))
            feasible_regions.append(IAR1)
            mapFootHoldIdxToPolygonIdx.append(foothold_index + 1)
            residualRadiusToStack.append(newResidualRadius1)
        else:
            newResidualRadius1 = 0.0
            
        # these two lines go together to overwrite the future swing foot
        params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[foothold_index - 1]
        IAR2, actuation_polygons_array, computation_time = self.compDyn.try_iterative_projection_bretl(params)
        if IAR2 is not False:
            if foothold_index > 0:
                newResidualRadius2 = deepcopy(
                    self.math.find_residual_radius(IAR2, footPlanningParams.com_position_to_validateW))
                searchDirection2 = -1

                area.append(self.compGeo.computePolygonArea(IAR2))
                mapFootHoldIdxToPolygonIdx.append(foothold_index - 1)
                #                footOptions.append(deepcopy(params.contactsWF[params.actual_swing]))
                feasible_regions.append(IAR2)
                residualRadiusToStack.append(newResidualRadius2)
            else:
                newResidualRadius2 = 0.0
        else:
            newResidualRadius2 = 0.0
        #            print 'RADS', residualRadius, newResidualRadius1, newResidualRadius2
        if (newResidualRadius1 > (residualRadius + footPlanningParams.TOL)) and (
                newResidualRadius1 > (newResidualRadius2 + footPlanningParams.TOL)):
            searchDirection = searchDirection1
            gradient = newResidualRadius1 - residualRadius
            residualRadius = newResidualRadius1

        elif (newResidualRadius2 > (residualRadius + footPlanningParams.TOL)) and (
                newResidualRadius2 > (newResidualRadius1 + footPlanningParams.TOL)):
            searchDirection = searchDirection2
            gradient = newResidualRadius2 - residualRadius
            residualRadius = newResidualRadius2
        else:  # you are already in the max
            searchDirection = 0
            #                    print 'final foothold index', foothold_index
            print 'RETURN before entering while loop'
            gradient = False
            return gradient, searchDirection, residualRadius, foothold_index, residualRadiusToStack, feasible_regions, mapFootHoldIdxToPolygonIdx

        return gradient, searchDirection, residualRadius, foothold_index, residualRadiusToStack, feasible_regions, mapFootHoldIdxToPolygonIdx

    def selectMaximumFeasibleArea(self, footPlanningParams, params):
        ng = 4
        params.setConstraintModes(['FRICTION_AND_ACTUATION',
                                   'FRICTION_AND_ACTUATION',
                                   'FRICTION_AND_ACTUATION',
                                   'FRICTION_AND_ACTUATION'])
        
        params.setNumberOfFrictionConesEdges(ng)

        params.setCoMPosWF(footPlanningParams.com_position_to_validateW)

        #        print numberOfFeetOptions
        feasible_regions = []
        residualRadiusToStack = []
        #        print 'empty res radii', residualRadiusToStack
        #        footOptions = []
        area = []
        mapFootHoldIdxToPolygonIdx = []

        #        counter = 0
        print 'number of feet options ', footPlanningParams.numberOfFeetOptions
        numberOfOptions = footPlanningParams.numberOfFeetOptions
        print footPlanningParams.footOptions

        # check the prediction point at the beginning
        if numberOfOptions > 0:
            for footIndex in range(0, int(numberOfOptions)):
                # these two lines go together to overwrite the future swing foot
                params.contactsWF[params.actual_swing] = footPlanningParams.footOptions[footIndex]
                IAR, actuation_polygons_array, computation_time = self.compDyn.try_iterative_projection_bretl(params)
                if IAR is False:
                    residualRadius = 0.0
                    newArea = 0.0
                else:
                    residualRadius = self.math.find_residual_radius(IAR,
                                                                    footPlanningParams.com_position_to_validateW)
                    newArea = self.compGeo.computePolygonArea(IAR)

                    mapFootHoldIdxToPolygonIdx.append(footIndex)
                    feasible_regions.append(IAR)
                    residualRadiusToStack.append(residualRadius)
                    area.append(newArea)
            print 'area ', area
            if np.size(area, 0) > 0:
                maxFootIndex = np.argmax(area)
            else:
                maxFootIndex = -1
            print 'max foothold: ', maxFootIndex

        else:
            maxFootIndex = -1
        #            feasible_regions = false

        #        print 'res radii', residualRadiusToStack

        #            print 'foothold index ', foothold_index
        #            footPlanningParams.option_index = foothold_index
        return maxFootIndex, residualRadiusToStack, feasible_regions, mapFootHoldIdxToPolygonIdx