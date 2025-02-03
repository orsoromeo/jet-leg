# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

#!/usr/bin/env python

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
        self.hyq_wbs = dict()
        self.hyq_rcf_debug = StringDoubleArray()  
        self.actuation_polygon_topic_pub_name = "/hyq/actuation_polygon"
        self.sim_time  = 0.0
        self.numberOfReceivedMessages = 0
        
    def run(self):
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1)
        self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, StringDoubleArray, callback=self._reg_sim_rcf_debug, queue_size=1)
        self.pub_polygon = ros.Publisher(self.actuation_polygon_topic_pub_name, Polygon3D, queue_size=1)

    def _reg_sim_time(self, time):
        self.sim_time = time.clock.secs + time.clock.nsecs/1000000000.0
#        print("getting time")
        
    def _reg_sim_wbs(self, msg):
        self.hyq_wbs = copy.deepcopy(msg)

    def _reg_sim_rcf_debug(self, msg):
        self.numberOfReceivedMessages += 1
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
        self.footPosLF = [0.3, 0.2, -.5]
        self.footPosRF = [0.3, -0.2, -.5]
        self.footPosLH = [-0.3, 0.2, -.5]
        self.footPosRH = [-0.3, -0.2, -.5]
        self.state_machineLF = True
        self.state_machineRF = True
        self.state_machineLH = True
        self.state_machineRH = True
        self.stanceFeet = [1, 1, 1, 0]
        self.numberOfContacts = 3
        contacts = np.vstack((self.footPosLF,self.footPosRF,self.footPosLH,self.footPosRH))
        self.feetPos = contacts
        self.contacts = contacts
        self.numberOfPublishedMessages = 0
    
    def getParams(self, received_data):
        num_of_elements = np.size(received_data.data)
        self.numberOfPublishedMessages +=1
#        print self.numberOfPublishedMessages

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

    start_t_IP = time.time()
    
    for j in range (0,1000):
        vertices = [point]
#        print("Time: " + str(i*0.004) + "s and Simulation time: " + str(p.get_sim_time()/60))
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
#        number_of_vertices = 10
#        print IAR
        for i in range(0, number_of_vertices):
            point = Point()
            point.x = IAR[i][0]
            point.y = IAR[i][1]
            point.z = 0.0
            vertices = np.hstack([vertices, point])
#        print'vertices', vertices
        
        p.send_polygons(name, vertices)
        
#        time.sleep(1.0/50.0)
        i+=1
        
    print 'de registering...'
    p.deregister_node()
    
    computation_time = (time.time() - start_t_IP)
    print("Total time: --- %s seconds ---" % computation_time)
    print 'number of published messages ', actuationParams.numberOfPublishedMessages
    avgTime = computation_time/actuationParams.numberOfPublishedMessages    
    print 'average publishing time [ms]', avgTime
    print 'average publishing frequency [Hz]', 1.0/avgTime        

    print 'number of received messages ', p.numberOfReceivedMessages
    avgTime = computation_time/p.numberOfReceivedMessages    
    print 'average subscription time [ms]', avgTime
    print 'average subscription frequency [Hz]', 1.0/avgTime 
    
if __name__ == '__main__':
    
    try:
        talker()
    except ros.ROSInterruptException:
        pass
    
        