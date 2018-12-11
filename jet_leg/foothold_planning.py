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
        
        print "AAA" , params.com_position_to_validateW
        print "BBB" , params.sample_contacts
        
        numberOfFeetOptions = np.size(params.footOptions,0)
        print numberOfFeetOptions
        for i in range(0, numberOfFeetOptions):
            #overwrite the future swing foot
            params.contactsWF[params.actual_swing] = params.footOptions[i]
            params.setContactsPosWF(params.contactsWF)
            IAR0, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
            self.area[i] = self.compGeo.computePolygonArea(IAR0)
        
        ######################################à
#        params.contactsWF = params.sample_contacts  
#        #overwrite the future swing foot
#        params.contactsWF[params.actual_swing] = params.footOptions[1]
#        params.setContactsPosWF(params.contactsWF)
#        
#        IAR1, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
#        self.area[1] = self.compGeo.computePolygonArea(IAR1)
#        
#        ##########################################
#        params.contactsWF = params.sample_contacts 
#        #overwrite the future swing foot
#        params.contactsWF[params.actual_swing] = params.footOptions[2]
#        params.setContactsPosWF(params.contactsWF)
#        IAR2, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
#        self.area[2] = self.compGeo.computePolygonArea(IAR2)
#        
#        ##########################################
#        params.contactsWF = params.sample_contacts 
#        #overwrite the future swing foot
#        params.contactsWF[params.actual_swing] = params.footOptions[3]
#        params.setContactsPosWF(params.contactsWF)
#        IAR3, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
#        self.area[3] = self.compGeo.computePolygonArea(IAR3)
#        
#        ##########################################
#        params.contactsWF = params.sample_contacts 
#        #overwrite the future swing foot
#        params.contactsWF[params.actual_swing] = params.footOptions[4]
#        params.setContactsPosWF(params.contactsWF)
#        IAR4, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
#        self.area[4] = self.compGeo.computePolygonArea(IAR4)
        
        print 'area ',self.area
        print 'max arg ',np.argmax(self.area, axis=0)
        return np.argmax(self.area, axis=0)