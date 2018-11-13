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
        self.hyq_actuation_params_sub_name = "/hyq/debug"

        self.hyq_wbs = dict()
        self.hyq_rcf_debug = StringDoubleArray()  
        self.actuation_polygon_topic_name = "/hyq/actuation_polygon"
        self.support_region_topic_name = "/hyq/support_region"
        self.force_polygons_topic_name = "/hyq/force_polygons"


        self.sim_time  = 0.0
        
    def run(self):
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1)
#        self.sub_wbs = ros.Subscriber(self.hyq_wbs_sub_name, WholeBodyState, callback=self._reg_sim_wbs, queue_size=1)
        self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, StringDoubleArray, callback=self._reg_sim_rcf_debug, queue_size=1)
#        self.sub_rcf_aux = ros.Subscriber(self.hyq_rcf_aux_sub_name, RCFaux, callback=self._reg_sim_rcf_aux, queue_size=1)
#        self.sub_rcf_debug = ros.Subscriber(self.hyq_rcf_debug_sub_name, StringDoubleArray, callback=self._reg_sim_rcf_debug, queue_size=1)
#        self.sub_rcf_params = ros.Subscriber(self.hyq_rcf_params_sub_name, RCFParams, callback=self._reg_sim_rcf_params, queue_size=1)
#        self.pub_rcf_params = ros.Publisher(self.debug_topic_name, SimpleDoubleArray, queue_size=1)
        self.pub_polygon = ros.Publisher(self.actuation_polygon_topic_name, Polygon3D, queue_size=1)
        self.pub_support_region = ros.Publisher(self.support_region_topic_name, Polygon3D, queue_size=1)
        self.pub_force_polygons = ros.Publisher(self.force_polygons_topic_name, LegsPolygons, queue_size=1)
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

    def send_force_polygons(self, name, polygons):
#        self.output = dict()
        output = LegsPolygons()
        output.names = name
        output.polygons = polygons
        self.pub_force_polygons.publish(output) 
        
    def send_support_region(self, name, vertices):
#        self.output = dict()
        output = Polygon3D()
        output.names = name
        output.vertices = vertices
        self.pub_support_region.publish(output) 
        
    def send_actuation_polygons(self, name, vertices):
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

        self.LF_tau_lim = [50.0, 50.0, 50.0]
        self.RF_tau_lim = [50.0, 50.0, 50.0]
        self.LH_tau_lim = [50.0, 50.0, 50.0]
        self.RH_tau_lim = [50.0, 50.0, 50.0]
        self.torque_limits = np.array([self.LF_tau_lim, self.RF_tau_lim, self.LH_tau_lim, self.RH_tau_lim, ])
        
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
            if str(received_data.name[j]) == str("LF_HAAmin"):
                self.LF_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("LF_HFEmin"):
                self.LF_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("LF_KFEmin"):
                self.LF_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_HAAmin"):
                self.RF_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("RF_HFEmin"):
                self.RF_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_KFEmin"):
                self.RF_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_HAAmin"):
                self.LH_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("LH_HFEmin"):
                self.LH_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_KFEmin"):
                self.LH_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_HAAmin"):
                self.RH_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("RH_HFEmin"):
                self.RH_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_KFEmin"):
                self.RH_tau_lim[2] = received_data.data[j]
                
            self.torque_limits = np.array([self.LF_tau_lim, 
                                           self.RF_tau_lim, 
                                           self.LH_tau_lim, 
                                           self.RH_tau_lim, ])
                                                                                    
#            print 'torque lims',self.torque_limits
            
            if str(received_data.name[j]) == str("footPosLFx"):
                self.footPosLF[0] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosLFy"):
                self.footPosLF[1] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosLFz"):
                self.footPosLF[2] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRFx"):
                self.footPosRF[0] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRFy"):
                self.footPosRF[1] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRFz"):
                self.footPosRF[2] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosLHx"):
                self.footPosLH[0] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosLHy"):
                self.footPosLH[1] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosLHz"):
                self.footPosLH[2] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRHx"):
                self.footPosRH[0] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRHy"):
                self.footPosRH[1] = int(received_data.data[j]*100.0)/100.0
            if str(received_data.name[j]) == str("footPosRHz"):
                self.footPosRH[2] = int(received_data.data[j]*100.0)/100.0
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
            
            if self.state_machineLF == 0.0 or self.state_machineLF == 3.0:
                self.stanceFeet[0] = 1
            else:
                self.stanceFeet[0] = 0
                
            if self.state_machineRF == 0.0 or self.state_machineRF == 3.0:
                self.stanceFeet[1] = 1
            else:
                self.stanceFeet[1] = 0
                
            if self.state_machineLH == 0.0 or self.state_machineLH == 3.0:
                self.stanceFeet[2] = 1
            else:
                self.stanceFeet[2] = 0
                
            if self.state_machineRH == 0.0 or self.state_machineRH == 3.0:
                self.stanceFeet[3] = 1                
            else:
                self.stanceFeet[3] = 0
                    
#            self.contacts = np.zeros((4,3))     
            
            counter = 0
#            print self.state_machineLF, self.stanceFeet
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
    polygonVertex = Point()
    actPolygon = Polygon3D()
    actuationParams = ActuationParameters()
    i = 0

    while not ros.is_shutdown():
        vertices1 = [point]
        actPolygons = [polygonVertex]
#        poly = []
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
        normals = np.vstack([n1, n2, n3, n4])

        """ contact points """
        nc = actuationParams.numberOfContacts
        contacts = actuationParams.contacts[0:nc+1, :]
#        print 'contacts: ',contacts
#        print contacts, actuationParams.stanceFeet
#        print actuationParams.LF_tau_lim
        comWF = np.array([0.0,0.0,0.0])
        # ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
        constraint_mode = 'FRICTION_AND_ACTUATION'
        IAR, actuation_polygons_array, computation_time = compDyn.iterative_projection_bretl(constraint_mode, actuationParams.stanceFeet, contacts, normals, trunk_mass, 4, 1.0, comWF, actuationParams.torque_limits)
#        IAR, actuation_polygons, computation_time = compDyn.instantaneous_actuation_region_bretl(actuationParams.stanceFeet, contacts, normals, trunk_mass)
        num_actuation_vertices = np.size(IAR, 0)
        
        point.x = IAR[0][0]
        point.y = IAR[0][1]
        point.z =  -0.55
        vertices1 = [point]
        
        for i in range(0, num_actuation_vertices):
            point = Point()
            point.x = IAR[i][0]
            point.y = IAR[i][1]
            point.z = -0.55
            vertices1 = np.hstack([vertices1, point])

        poly = [actPolygon]
        for i in range(0, nc):
            actPolygons = [polygonVertex]            
            for j in range(0,7):
                vx = Point()
#                print 'act', actuation_polygons_array[i][0][j]
                vx.x = actuation_polygons_array[i][0][j]
                vx.y = actuation_polygons_array[i][1][j]
                vx.z = actuation_polygons_array[i][2][j]
#                print vx
                actPolygons = np.hstack([actPolygons, vx])          
            poly = np.hstack([poly, actPolygons])  
            
        p.send_actuation_polygons(name, vertices1)
        
#        p.send_force_polygons(name, poly)

        # ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
        constraint_mode = 'ONLY_FRICTION'
        IAR, actuation_polygons, computation_time = compDyn.iterative_projection_bretl(constraint_mode, actuationParams.stanceFeet, contacts, normals, trunk_mass, 4, 1.0, comWF, actuationParams.torque_limits)
#        IAR, actuation_polygons, computation_time = compDyn.instantaneous_actuation_region_bretl(actuationParams.stanceFeet, contacts, normals, trunk_mass)
        num_support_vertices = np.size(IAR, 0)
        point.x = IAR[0][0]
        point.y = IAR[0][1]
        point.z =  -0.55
        vertices2 = [point]
        for i in range(0, num_support_vertices):
            point = Point()
            point.x = IAR[i][0]
            point.y = IAR[i][1]
            point.z = -0.55
            vertices2 = np.hstack([vertices2, point])
            
#        print'vertices', vertices2        
        p.send_support_region(name, vertices2)
        
        
        time.sleep(1.0/35.0)
        i+=1
        
    print 'de registering...'
    p.deregister_node()
        

if __name__ == '__main__':
    
    try:
        talker()
    except ros.ROSInterruptException:
        pass
    
        