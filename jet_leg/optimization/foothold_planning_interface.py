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
from dls_msgs.msg import StringDoubleArray
from feasible_region.msg import Polygon3D, LegsPolygons
from dwl_msgs.msg import WholeBodyState, WholeBodyTrajectory, JointState, ContactState, BaseState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Header
from std_srvs.srv import Empty
from termcolor import colored

from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.maths.computational_geometry import ComputationalGeometry
from jet_leg.maths.math_tools import Math


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
        self.footOption5 = [0., 0., 0.]
        self.footOption6 = [0., 0., 0.]
        self.footOption7 = [0., 0., 0.]
        self.footOption8 = [0., 0., 0.]
        self.footOptions = np.array([self.footOption0,
                                     self.footOption1,
                                     self.footOption2,
                                     self.footOption3,
                                     self.footOption4,
                                     self.footOption5,
                                     self.footOption6,
                                     self.footOption7,
                                     self.footOption8])
                                    
        self.numberOfFeetOptions = 0
        self.sample_contacts = np.zeros((4,3))
   
        self.minRadius = 0.
        self.optimization_started = False
        #outputs        
        self.option_index = 0
        self.ack_optimization_done = False
        self.TOL = 0.001
        
    def getParamsFromRosDebugTopic(self, received_data):
        
        num_of_elements = np.size(received_data.data)
#        print 'number of elements: ', num_of_elements
        for j in range(0,num_of_elements):
#            print j, received_data.name[j], str(received_data.name[j]), str("footPosLFx")
   
                
                
                    #foothold planning
            if str(received_data.name[j]) == str("com_position_to_validateWx"):
                self.com_position_to_validateW[0] = received_data.data[j]
            if str(received_data.name[j]) == str("com_position_to_validateWy"):
                self.com_position_to_validateW[1] = received_data.data[j]
            if str(received_data.name[j]) == str("com_position_to_validateWz"):
                self.com_position_to_validateW[2] = received_data.data[j]                
                
                
            if str(received_data.name[j]) == str("foothold_option0x"):  
                self.footOption0[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option0y"):  
                self.footOption0[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option0z"):  
                self.footOption0[2] = received_data.data[j]    
                
            if str(received_data.name[j]) == str("foothold_option1x"):  
                self.footOption1[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option1y"):  
                self.footOption1[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option1z"):  
                self.footOption1[2] = received_data.data[j]   

            if str(received_data.name[j]) == str("foothold_option2x"):  
                self.footOption2[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option2y"):  
                self.footOption2[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option2z"):  
                self.footOption2[2] = received_data.data[j]   

            if str(received_data.name[j]) == str("foothold_option3x"):  
                self.footOption3[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option3y"):  
                self.footOption3[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option3z"):  
                self.footOption3[2] = received_data.data[j]   

            if str(received_data.name[j]) == str("foothold_option4x"):  
                self.footOption4[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option4y"):  
                self.footOption4[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option4z"):  
                self.footOption4[2] = received_data.data[j]  
                
                
            if str(received_data.name[j]) == str("foothold_option5x"):  
                self.footOption5[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option5y"):  
                self.footOption5[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option5z"):  
                self.footOption5[2] = received_data.data[j]  
                
                
            if str(received_data.name[j]) == str("foothold_option6x"):  
                self.footOption6[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option6y"):  
                self.footOption6[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option6z"):  
                self.footOption6[2] = received_data.data[j]  
                
                
            if str(received_data.name[j]) == str("foothold_option7x"):  
                self.footOption7[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option7y"):  
                self.footOption7[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option7z"):  
                self.footOption7[2] = received_data.data[j]  
                          
            
            if str(received_data.name[j]) == str("foothold_option8x"):  
                self.footOption8[0] = received_data.data[j]
            if str(received_data.name[j]) == str("foothold_option8y"):  
                self.footOption8[1] = received_data.data[j]                   
            if str(received_data.name[j]) == str("foothold_option8z"):  
                self.footOption8[2] = received_data.data[j]  

            if str(received_data.name[j]) == str("numberOfFootholdOptions"):  
                self.numberOfFeetOptions = received_data.data[j]  
                          
            self.footOptions = np.array([self.footOption0,
                                     self.footOption1,
                                     self.footOption2,
                                     self.footOption3,
                                     self.footOption4,
                                     self.footOption5,
                                     self.footOption6,
                                     self.footOption7,
                                     self.footOption8])
#            print self.footOptions

            if str(received_data.name[j]) == str("optimization_started"):  
                self.optimization_started = bool(received_data.data[j])    
            
            if str(received_data.name[j]) == str("minRadius"):  
                self.minRadius = received_data.data[j]                
                