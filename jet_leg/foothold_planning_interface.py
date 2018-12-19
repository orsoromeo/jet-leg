# -*- coding: utf-8 -*-
"""
Created on Wed Dec 19 09:44:02 2018

@author: rorsolino
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

from context import jet_leg 
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry import ComputationalGeometry
from jet_leg.math_tools import Math


stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class FootholdPlanningInterface:
    def __init__(self):      
        
        self.com_position_to_validateW = [0., 0., 0.] #used only for foothold planning
                
        self.orientation0 = [0., 0., 0.]
        self.orientation1 = [0., 0., 0.]
        self.orientation2 = [0., 0., 0.]
        self.orientation3 = [0., 0., 0.]
        self.orientation4 = [0., 0., 0.]
        self.orientationOptions = np.array([self.orientation0,
                                     self.orientation1,
                                     self.orientation2,
                                     self.orientation3,
                                     self.orientation4])
                                     
        #foothold planning 
        self.footOption0 = [0., 0., 0.]
        self.footOption1 = [0., 0., 0.]
        self.footOption2 = [0., 0., 0.]
        self.footOption3 = [0., 0., 0.]
        self.footOption4 = [0., 0., 0.]
        self.footOptions = np.array([self.footOption0,
                                     self.footOption1,
                                     self.footOption2,
                                     self.footOption3,
                                     self.footOption4])
                                    
        self.numberOfFeetOptions = 0
        self.sample_contacts = np.zeros((4,3))