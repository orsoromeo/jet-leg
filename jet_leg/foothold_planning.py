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
        params.contactsBF[params.actual_swing] = params.footOptions[0]
        IAR0, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
        self.area[0] = self.compGeo.computePolygonArea(IAR0)
        
        #foothold planning
        #overwite the position for the actual swing, then the future polygon of stance will be evaluated with that point
        params.contactsBF[params.actual_swing] = params.footOptions[1]   
        IAR1, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
        self.area[1] = self.compGeo.computePolygonArea(IAR1)
        
        
        #foothold planning
        #overwite the position for the actual swing, then the future polygon of stance will be evaluated with that point
        params.contactsBF[params.actual_swing] = params.footOptions[2]
        IAR2, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
        self.area[2] = self.compGeo.computePolygonArea(IAR2)
        
        
        #foothold planning
        #overwite the position for the actual swing, then the future polygon of stance will be evaluated with that point
        params.contactsBF[params.actual_swing] = params.footOptions[3]
        IAR3, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
        self.area[3] = self.compGeo.computePolygonArea(IAR3)
        
                #foothold planning
        #overwite the position for the actual swing, then the future polygon of stance will be evaluated with that point
        params.contactsBF[params.actual_swing] = params.footOptions[4]
        IAR4, actuation_polygons_array, computation_time = self.compDyn.iterative_projection_bretl(params)
        self.area[4] = self.compGeo.computePolygonArea(IAR4)
        
        print 'area ',self.area
        print 'max arg ',np.argmax(self.area, axis=0)
        return np.argmax(self.area, axis=0)