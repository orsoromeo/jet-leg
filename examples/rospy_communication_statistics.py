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


#from gazebo_msgs.srv import ApplyBodyWrench
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
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters
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
        self.hyq_actuation_params_sub_name = "/hyq/actuation_polygon"
#        self.hyq_rcf_params_sub_name = "/hyq/crt_rcf_params"
#        self.hyq_rcf_aux_sub_name = "/hyq/crt_rcf_aux"
#        self.hyq_rcf_debug_sub_name = "/hyq/crt_rcf_debug"
        self.hyq_wbs = dict()
        self.hyq_rcf_debug = Polygon3D()  
        self.actuation_polygon_topic_pub_name = "/hyq/actuation_polygon"
#        self.debug_topic_name = "/hyq/planner_back"
        self.sim_time  = 0.0
        self.numberOfReceivedMessages = 0
        
    def run(self):
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1)
#        self.sub_wbs = ros.Subscriber(self.hyq_wbs_sub_name, WholeBodyState, callback=self._reg_sim_wbs, queue_size=1)
        self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, Polygon3D, callback=self._reg_sim_rcf_debug, queue_size=1)
#        self.sub_rcf_aux = ros.Subscriber(self.hyq_rcf_aux_sub_name, RCFaux, callback=self._reg_sim_rcf_aux, queue_size=1)
#        self.sub_rcf_debug = ros.Subscriber(self.hyq_rcf_debug_sub_name, StringDoubleArray, callback=self._reg_sim_rcf_debug, queue_size=1)
#        self.sub_rcf_params = ros.Subscriber(self.hyq_rcf_params_sub_name, RCFParams, callback=self._reg_sim_rcf_params, queue_size=1)
#        self.pub_rcf_params = ros.Publisher(self.debug_topic_name, SimpleDoubleArray, queue_size=1)
        self.pub_polygon = ros.Publisher(self.actuation_polygon_topic_pub_name, Polygon3D, queue_size=1)
#        self.fbs = ros.ServiceProxy('/hyq/freeze_base', Empty)
#        self.startRCF = ros.ServiceProxy('/hyq/start_RCF', Empty)
#        self.stopRCF = ros.ServiceProxy('/hyq/stop_RCF', Empty)   

    def _reg_sim_time(self, time):
        self.sim_time = time.clock.secs + time.clock.nsecs/1000000000.0
#        print("getting time")
        
    def _reg_sim_wbs(self, msg):
        self.hyq_wbs = copy.deepcopy(msg)

    def _reg_sim_rcf_debug(self, msg):
        self.numberOfReceivedMessages += 1
        print 'number of received messages: ', self.numberOfReceivedMessages
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
#        output.names = name
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
        self.stanceFeet = [1, 1, 1, 0]
        self.numberOfContacts = 3
        LF_foot = np.array([0.3, 0.2, -0.5])
        RF_foot = np.array([0.3, -0.2, -0.5])
        LH_foot = np.array([-0.3, 0.2, -0.5])
        RH_foot = np.array([-0.3, -0.2, -0.5])
        contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        self.feetPos = contacts
        self.contacts = contacts
        self.numberOfPublishedMessages = 0
    
    def getParams(self, received_data):
        num_of_elements = np.size(received_data.vertices)
        self.numberOfPublishedMessages += 1
        print 'main for loop ', num_of_elements, self.numberOfPublishedMessages

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
    
    for j in range (0,100):
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
        LF_tau_lim = [50.0, 100.0, 100.0]
        RF_tau_lim = [50.0, 100.0, 100.0]
        LH_tau_lim = [50.0, 100.0, 100.0]
        RH_tau_lim = [50.0, 100.0, 100.0]
        torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
        comWF = np.array([0.0, 0.0, 0.0])
        extForceW = np.array([0.0,0.0, 0.0])
        constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION',
                      'FRICTION_AND_ACTUATION']
        mu = 0.8
        ng = 4
#        print 'contacts: ',contacts
#        print contacts, actuationParams.stanceFeet
        params = IterativeProjectionParameters()
        stanceFeet = [1,1,1,1]
        params.setContactsPosWF(contacts)
        params.setCoMPosWF(comWF)
        params.setTorqueLims(torque_limits)
        params.setActiveContacts(stanceFeet)
        params.setConstraintModes(constraint_mode_IP)
        params.setContactNormals(normals)
        params.setFrictionCoefficient(mu)
        params.setNumberOfFrictionConesEdges(ng)
        params.setTotalMass(trunk_mass + extForceW[2]/9.81)
        params.externalForceWF = extForceW
        ''' compute iterative projection '''
        IAR, actuation_polygons, computation_time = compDyn.iterative_projection_bretl(params)
        

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
        
#        time.sleep(1.0/5.0)
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
    
        