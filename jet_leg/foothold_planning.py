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
        
    def optimizeFootHoldAndBaseOrient(self, params):        

        #foothold planning
        #overwite the position for the actual swing, then the future polygon of stance will be evaluated with that point
        print 'foothold options',params.footOptions
                    
        params.sample_contacts = params.contactsWF 
        params.setCoMPosWF(params.com_position_to_validateW)
        
        print "AAA" , params.com_position_to_validateW
        print "BBB" , params.sample_contacts
        
        numberOfFeetOptions = np.size(params.footOptions,0)
        numberOfBaseOrientationOptions = np.size(params.orientationOptions,0)
        
        print numberOfFeetOptions
        for i in range(0, numberOfFeetOptions):
            for j in range(0, numberOfBaseOrientationOptions):
                params.contactsWF  =  deepcopy(params.sample_contacts)
                #overwrite the future swing foot
                params.contactsWF[params.actual_swing] = params.footOptions[i]
                params.roll = params.orientationOptions[j][0]
                print params.roll
            
                IAR, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
                self.area[i] = self.compGeo.computePolygonArea(IAR)
                
        print 'area ',self.area
        print 'max arg ',np.argmax(self.area, axis=0)
        return np.argmax(self.area, axis=0)