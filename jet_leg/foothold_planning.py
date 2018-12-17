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

from context import jet_leg 
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry import ComputationalGeometry
from jet_leg.math_tools import Math


stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class FootHoldPlanning:
    def __init__(self):      
        self.compGeo = ComputationalGeometry()
        self.compDyn = ComputationalDynamics()
        self.area = [0.0,0.0,0.0,0.0,0.0]
        self.option_index = 0
        self.ack_optimization_done = False
        
    def optimizeFootHold(self, params):        

        #foothold planning
        #overwite the position for the actual swing, then the future polygon of stance will be evaluated with that point
        print 'foothold options',params.footOptions
                    
        params.sample_contacts = params.contactsWF 
        params.setCoMPosWF(params.com_position_to_validateW)
        
        print "com pos to validate" , params.com_position_to_validateW
        print "sample contacts" , params.sample_contacts
        
        numberOfFeetOptions = np.size(params.footOptions,0)
        print numberOfFeetOptions
        feasible_regions = []
        for i in range(0, numberOfFeetOptions):
            #overwrite the future swing foot
            params.contactsWF[params.actual_swing] = params.footOptions[i]
            print params.footOptions[i]
            params.setContactsPosWF(params.contactsWF)
            IAR, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
#            print 'IAR', IAR
            feasible_regions.append(IAR)
#            print 'FR', feasible_regions
            self.area[i] = self.compGeo.computePolygonArea(IAR)
        
        print 'area ',self.area
        print 'max arg ',np.argmax(self.area, axis=0)
        return np.argmax(self.area, axis=0), feasible_regions
        
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
                #overwrite the future swing foot
                params.contactsWF[params.actual_swing] = params.footOptions[i]
                params.roll = params.orientationOptions[j][0]
                print params.roll
                params.setContactsPosWF(params.contactsWF)
                IAR, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
                self.area[i] = self.compGeo.computePolygonArea(IAR)
                
        print 'area ',self.area
        print 'max arg ',np.argmax(self.area, axis=0)
        return np.argmax(self.area, axis=0)