#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  3 13:38:45 2018

@author: aradulescu
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
from dls_msgs.msg import SimpleDoubleArray, StringDoubleArray, Polygon3D
from dwl_msgs.msg import WholeBodyState, WholeBodyTrajectory, JointState, ContactState, BaseState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Header
from std_srvs.srv import Empty
from termcolor import colored

from context import jet_leg 
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.math_tools import Math


stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
#import utils
sys.stderr = stderr


class HyQSim(threading.Thread):
    def __init__(self):  
        
        threading.Thread.__init__(self)
        
        self.clock_sub_name = 'clock'
        self.hyq_wbs_sub_name = "/hyq/robot_states"
        self.hyq_actuation_params_sub_name = "/hyq/planner_debug"
#        self.hyq_rcf_params_sub_name = "/hyq/crt_rcf_params"
#        self.hyq_rcf_aux_sub_name = "/hyq/crt_rcf_aux"
#        self.hyq_rcf_debug_sub_name = "/hyq/crt_rcf_debug"
        self.hyq_wbs = dict()
        self.hyq_rcf_debug = StringDoubleArray()  
        self.polygon_topic_name = "/hyq/actuation_polygon"
#        self.debug_topic_name = "/hyq/planner_back"
        self.sim_time  = 0.0
        
    def run(self):
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1)
#        self.sub_wbs = ros.Subscriber(self.hyq_wbs_sub_name, WholeBodyState, callback=self._reg_sim_wbs, queue_size=1)
        self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, StringDoubleArray, callback=self._reg_sim_rcf_debug, queue_size=1)
#        self.sub_rcf_aux = ros.Subscriber(self.hyq_rcf_aux_sub_name, RCFaux, callback=self._reg_sim_rcf_aux, queue_size=1)
#        self.sub_rcf_debug = ros.Subscriber(self.hyq_rcf_debug_sub_name, StringDoubleArray, callback=self._reg_sim_rcf_debug, queue_size=1)
#        self.sub_rcf_params = ros.Subscriber(self.hyq_rcf_params_sub_name, RCFParams, callback=self._reg_sim_rcf_params, queue_size=1)
#        self.pub_rcf_params = ros.Publisher(self.debug_topic_name, SimpleDoubleArray, queue_size=1)
        self.pub_polygon = ros.Publisher(self.polygon_topic_name, Polygon3D, queue_size=1)
#        self.fbs = ros.ServiceProxy('/hyq/freeze_base', Empty)
#        self.startRCF = ros.ServiceProxy('/hyq/start_RCF', Empty)
#        self.stopRCF = ros.ServiceProxy('/hyq/stop_RCF', Empty)   

    def _reg_sim_time(self, time):
        self.sim_time = time.clock.secs + time.clock.nsecs/1000000000.0
#        print("getting time")
        
    def _reg_sim_wbs(self, msg):
        self.hyq_wbs = copy.deepcopy(msg)

    def _reg_sim_rcf_debug(self, msg):
#        print 'new data received'
        self.hyq_rcf_debug = copy.deepcopy(msg)  
        
    def register_node(self):
        ros.init_node('sub_pub_node_python', anonymous=False)

    def deregister_node(self):
        ros.signal_shutdown("manual kill")
        
    def get_sim_time(self):
        return self.sim_time
        
    def get_sim_wbs(self):
        return self.hyq_wbs
    
    def send_polygons(self, name, vertices):
#        self.output = dict()
        output = Polygon3D()
        output.names = name
        output.vertices = vertices
        self.pub_polygon.publish(output)    
    
    def send_simple_array(self, name, data):
#        self.output = dict()
        output = SimpleDoubleArray()
        output.name = name
        output.data = data
        self.pub_rcf_params.publish(output)

class ActuationParameters:
    def __init__(self):
        self.CoMposition = [0., 0., 0.]
        self.footPosLF = [0.3, 0.25, -.5]
        self.footPosRF = [-0.3, 0.25, -.5]
        self.footPosLH = [0.4, -0.25, -.5]
        self.footPosRH = [-0.3, -0.25, -.5]
        self.state_machineLF = True
        self.state_machineRF = True
        self.state_machineLH = True
        self.state_machineRH = True
        self.stanceFeet = [0, 0, 0, 0]
        self.numberOfContacts = 0
        self.feetPos = np.zeros((4,3))
        self.contacts = np.zeros((4,3))
    
    def getParams(self, received_data):
        num_of_elements = np.size(received_data.data)
#        print 'number of elements: ', num_of_elements
        for j in range(0,num_of_elements):
#            print j, received_data.name[j], str(received_data.name[j]), str("footPosLFx")
            if str(received_data.name[j]) == str("footPosLFx"):
                self.footPosLF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosLFy"):
                self.footPosLF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosLFz"):
                self.footPosLF[2] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosRFx"):
                self.footPosRF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosRFy"):
                self.footPosRF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosRFz"):
                self.footPosRF[2] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosLHx"):
                self.footPosLH[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosLHy"):
                self.footPosLH[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosLHz"):
                self.footPosLH[2] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosRHx"):
                self.footPosRH[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosRHy"):
                self.footPosRH[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosRHz"):
                self.footPosRH[2] = received_data.data[j]
            if str(received_data.name[j]) == str("state_machineLF"):
                self.state_machineLF = received_data.data[j]
            if str(received_data.name[j]) == str("state_machineRF"):
                self.state_machineRF = received_data.data[j]
            if str(received_data.name[j]) == str("state_machineLH"):
                self.state_machineLH = received_data.data[j]
            if str(received_data.name[j]) == str("state_machineRH"):
                self.state_machineRH = received_data.data[j]  
                
            self.feetPos = np.array([[self.footPosLF],
                                     [self.footPosRF],
                                        [self.footPosLH],
                                            [self.footPosRH]])
            
            if self.state_machineLF == 0.0 or self.state_machineLF == 1.0:
                self.stanceFeet[0] = 1
            else:
                self.stanceFeet[0] = 0
                
            if self.state_machineRF == 0.0 or self.state_machineRF == 1.0:
                self.stanceFeet[1] = 1
            else:
                self.stanceFeet[1] = 0
                
            if self.state_machineLH == 0.0 or self.state_machineLH == 1.0:
                self.stanceFeet[2] = 1
            else:
                self.stanceFeet[2] = 0
                
            if self.state_machineRH == 0.0 or self.state_machineRH == 1.0:
                self.stanceFeet[3] = 1                
            else:
                self.stanceFeet[3] = 0
                    
            self.contacts = np.zeros((4,3))     
            
            counter = 0
            print self.state_machineLF, self.stanceFeet
            for i in range(0,4):
                self.contacts[i] = self.feetPos[i]
#                print i, self.stanceFeet[i]
                if self.stanceFeet[i] == 1:
#                    self.contacts[counter] = self.feetPos[i]
                    counter += 1
            
            self.numberOfContacts = counter

def talker():
    compDyn = ComputationalDynamics()
    math = Math()
    p=HyQSim()
    p.start()
    p.register_node()
    name = "Actuation_region"
    point = Point()
    
    actuationParams = ActuationParameters()
    i = 0

    while not ros.is_shutdown():
        vertices = [point]
        print("Time: " + str(i*0.004) + "s and Simulation time: " + str(p.get_sim_time()/60))
        p.get_sim_wbs()
        actuationParams.getParams(p.hyq_rcf_debug)
        trunk_mass = 85.
        axisZ= np.array([[0.0], [0.0], [1.0]])
        ''' normals '''    
        n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        normals = np.vstack([n1, n2, n3])

        """ contact points """
        nc = actuationParams.numberOfContacts
        contacts = actuationParams.contacts[0:nc+1, :]
#        print 'contacts: ',contacts
#        print contacts, actuationParams.stanceFeet
        IAR, actuation_polygons, computation_time = compDyn.instantaneous_actuation_region_bretl(actuationParams.stanceFeet, contacts, normals, trunk_mass)
        number_of_vertices = np.size(IAR, 0)
#        print IAR
        for i in range(0, number_of_vertices):
            point = Point()
            point.x = IAR[i][0]
            point.y = IAR[i][1]
            point.z = 0.0
            vertices = np.hstack([vertices, point])
#        print'vertices', vertices
        
        p.send_polygons(name, vertices)
        
        time.sleep(1.0/5.0)
        i+=1
        
    print 'de registering...'
    p.deregister_node()
        

if __name__ == '__main__':
    
    try:
        talker()
    except ros.ROSInterruptException:
        pass
    
        