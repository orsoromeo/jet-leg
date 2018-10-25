#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  3 13:38:45 2018

@author: aradulescu
"""

import copy
import numpy as np
import os
#import pexpect
#import psutil
import random
import rospy as ros
import rosgraph
import sys
import time
import threading
import traceback

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
        self.debug_topic_name = "/hyq/planner_back"
        self.sim_time  = 0.0
                
        
    def run(self):
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1)
        self.sub_wbs = ros.Subscriber(self.hyq_wbs_sub_name, WholeBodyState, callback=self._reg_sim_wbs, queue_size=1)
        self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, StringDoubleArray, callback=self._reg_sim_rcf_debug, queue_size=1)
#        self.sub_rcf_aux = ros.Subscriber(self.hyq_rcf_aux_sub_name, RCFaux, callback=self._reg_sim_rcf_aux, queue_size=1)
#        self.sub_rcf_debug = ros.Subscriber(self.hyq_rcf_debug_sub_name, StringDoubleArray, callback=self._reg_sim_rcf_debug, queue_size=1)
#        self.sub_rcf_params = ros.Subscriber(self.hyq_rcf_params_sub_name, RCFParams, callback=self._reg_sim_rcf_params, queue_size=1)
        self.pub_rcf_params = ros.Publisher(self.debug_topic_name, SimpleDoubleArray, queue_size=1)
        self.pub_polygon = ros.Publisher(self.polygon_topic_name, Polygon3D, queue_size=1)
#        self.fbs = ros.ServiceProxy('/hyq/freeze_base', Empty)
#        self.startRCF = ros.ServiceProxy('/hyq/start_RCF', Empty)
#        self.stopRCF = ros.ServiceProxy('/hyq/stop_RCF', Empty)



#    def call_freezeBaseService(self):
#        ros.wait_for_service('/hyq/freeze_base')
#        try:
#            print("Calling Service")
#            return self.fbs()            
#        except ros.ServiceException, e:
#            print("Service call failed: %s"%e)    
#
#    def call_startRCFService(self):
#        ros.wait_for_service('/hyq/start_RCF')
#        try:
#            print("Calling Service start_RCF")
#            return self.startRCF()            
#        except ros.ServiceException, e:
#            print("Service call failed: %s"%e)    
#
#    def call_stopRCFService(self):
#        ros.wait_for_service('/hyq/stop_RCF')
#        try:
#            print("Calling Service stop_RCF")
#            return self.stopRCF()            
#        except ros.ServiceException, e:
#            print("Service call failed: %s"%e)    

        
    def _reg_sim_time(self, time):
        self.sim_time = time.clock.secs + time.clock.nsecs/1000000000.0
#        print("getting time")
        
    def _reg_sim_wbs(self, msg):
        self.hyq_wbs = copy.deepcopy(msg)
#        print(colored('getting wbs', 'green'))
        
#    def _reg_sim_rcf_params(self, msg):
#        self.hyq_rcf_params = copy.deepcopy(msg)
#        
#    def _reg_sim_rcf_aux(self, msg):
#        self.hyq_rcf_aux = copy.deepcopy(msg)
#        
    def _reg_sim_rcf_debug(self, msg):
        self.hyq_rcf_debug = copy.deepcopy(msg)  
        
    def register_node(self):
        ros.init_node('sub_pub_node_python', anonymous=False)

    def deregister_node(self):
        ros.signal_shutdown("manual kill")
        
    def get_sim_time(self):
        return self.sim_time
        
    def get_sim_wbs(self):
        return self.hyq_wbs
    
#    def get_sim_rcf_params(self):
#        return self.hyq_rcf_params
#    
#    def get_sim_rcf_aux(self):
#        return self.hyq_rcf_aux
#    
#    def get_sim_rcf_debug(self):
#        return self.hyq_rcf_debug
#   

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
        self.footPosLF = [0., 0., 0.]
        self.footPosRF = [0., 0., 0.]
        self.footPosLH = [0., 0., 0.]
        self.footPosRH = [0., 0., 0.]
    
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

if __name__ == '__main__':
    
    compDyn = ComputationalDynamics()
    math = Math()
    p=HyQSim()
    p.start()
    p.register_node()
    name = "Miki"
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    point = Point()
#    point.x = 1.0
#    point.y = 1.5
#    point.z = 2.0
#    vertex = [point, point]
    
    actuationParams = ActuationParameters()
    for i in range(100000):
        vertices = [point]
        print("Time: " + str(i*0.004) + "s and Simulation time: " + str(p.get_sim_time()/60))
        crt_wbs = p.get_sim_wbs()
#        crt_rcf = p.get_sim_rcf_params()
        crt_wbs = p.get_sim_wbs()
#        crt_aux = p.get_sim_rcf_aux()
#        crt_debug = p.get_sim_rcf_debug()
#        print '-------------------------------->names of the received parameters are: ', p.hyq_rcf_debug.name[2]
#        print '-------------------------------->value of the received parameters are: ', p.hyq_rcf_debug.data[2]
        actuationParams.getParams(p.hyq_rcf_debug)
        print '-------------------------------->CoM position is: ', actuationParams.CoMposition
        trunk_mass = 85.
        axisZ= np.array([[0.0], [0.0], [1.0]])
        ''' normals '''    
        n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        normals = np.vstack([n1, n2, n3])
        nc = 3
        """ contact points """
        LF_foot = np.array([0.3, 0.3, -0.5])
        RF_foot = np.array([0.3, -0.2, -0.5])
        LH_foot = np.array([-0.2, 0.0, -0.5])
        RH_foot = np.array([-0.3, -0.2, -0.5])

        contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        contacts = contactsToStack[0:nc, :]
        
        IAR, actuation_polygons, computation_time = compDyn.instantaneous_actuation_region_bretl(contacts, normals, trunk_mass)
        number_of_vertices = np.size(IAR, 0)
        print IAR
        for i in range(0, number_of_vertices):
            point = Point()
            point.x = IAR[i][0]
            point.y = IAR[i][1]
            point.z = 0.0
            vertices = np.hstack([vertices, point])
        print'vertices', vertices
            
#        out_rcf_params = crt_rcf
#        out_rcf_params.KP_lin = list(out_rcf_params.KP_lin)
#        out_rcf_params.KP_lin[0] = out_rcf_params.KP_lin[0] + 10
#        out_rcf_params.KP_lin = tuple(out_rcf_params.KP_lin)
#        print("PCrt robot state: " + str(crt_wbs)) 
#        if i == 59 :
#            p.call_freezeBaseService()

#        output = ['miki',1]

        p.send_simple_array(name, data)
        
        p.send_polygons(name, vertices)
        
        time.sleep(1/250)
        
    p.deregister_node()
        
    
        
            
        