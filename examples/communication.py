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
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters


stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
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
        self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, StringDoubleArray, callback=self._reg_sim_rcf_debug, queue_size=1)
        self.pub_polygon = ros.Publisher(self.actuation_polygon_topic_name, Polygon3D, queue_size=1)
        self.pub_support_region = ros.Publisher(self.support_region_topic_name, Polygon3D, queue_size=1)
        self.pub_force_polygons = ros.Publisher(self.force_polygons_topic_name, LegsPolygons, queue_size=1)

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
        output = LegsPolygons()
        output.names = name
        output.polygons = polygons
        self.pub_force_polygons.publish(output) 
        
    def send_support_region(self, name, vertices):
        output = Polygon3D()
        output.names = name
        output.vertices = vertices
        self.pub_support_region.publish(output) 
        
    def send_actuation_polygons(self, name, vertices):
        output = Polygon3D()
        output.names = name
        output.vertices = vertices
        self.pub_polygon.publish(output)    
    
    def send_simple_array(self, name, data):
        output = SimpleDoubleArray()
        output.name = name
        output.data = data
        self.pub_rcf_params.publish(output)

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
    params = IterativeProjectionParameters()
    i = 0

    while not ros.is_shutdown():
        vertices1 = [point]
        actPolygons = [polygonVertex]
#        poly = []
#        print("Time: " + str(i*0.004) + "s and Simulation time: " + str(p.get_sim_time()/60))
        p.get_sim_wbs()
        params.getParamsFromRosDebugTopic(p.hyq_rcf_debug)
        params.getFutureStanceFeet(p.hyq_rcf_debug)
#        params.getCurrentStanceFeet(p.hyq_rcf_debug)
#        total_mass = 85.
#        mu = 0.8
        #friction cone edges 
        ng = 4
#        axisZ= np.array([[0.0], [0.0], [1.0]])
        ''' normals '''    
#        n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
#        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
#        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
#        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
#        normals = np.vstack([n1, n2, n3, n4])

        """ contact points """
        nc = params.numberOfContacts
        # ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
        constraint_mode_IP = 'FRICTION_AND_ACTUATION'
        params.setConstraintModes([constraint_mode_IP,
                           constraint_mode_IP,
                           constraint_mode_IP,
                           constraint_mode_IP])
        #params.setContactNormals(normals)
        #params.setFrictionCoefficient(mu)
        params.setNumberOfFrictionConesEdges(ng)
        #params.setTotalMass(total_mass)
        #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
        IAR, actuation_polygons_array, computation_time = compDyn.iterative_projection_bretl(params)
        num_actuation_vertices = np.size(IAR, 0)
        
        point.x = IAR[0][0]
        point.y = IAR[0][1]
        point.z =  0.0 #is the centroidal frame
        vertices1 = [point]
        
        for i in range(0, num_actuation_vertices):
            point = Point()
            point.x = IAR[i][0]
            point.y = IAR[i][1]
            point.z = 0.0 #is the centroidal frame
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
        constraint_mode_IP = 'ONLY_FRICTION'
        params.setConstraintModes([constraint_mode_IP,
                           constraint_mode_IP,
                           constraint_mode_IP,
                           constraint_mode_IP])
        #params.setContactNormals(normals)
        #params.setFrictionCoefficient(mu)
        params.setNumberOfFrictionConesEdges(ng)
#        params.setTrunkMass(trunk_mass)
        #    IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)
        IAR, actuation_polygons, computation_time = compDyn.iterative_projection_bretl(params)
        num_support_vertices = np.size(IAR, 0)
        point.x = IAR[0][0]
        point.y = IAR[0][1]
        point.z =  0.0
        vertices2 = [point]
        for i in range(0, num_support_vertices):
            point = Point()
            point.x = IAR[i][0]
            point.y = IAR[i][1]
            point.z = 0.0
            vertices2 = np.hstack([vertices2, point])
            
#        print'vertices', vertices2        
        p.send_support_region(name, vertices2)
        
        

        time.sleep(0.2)
        i+=1
        
    print 'de registering...'
    p.deregister_node()
        

if __name__ == '__main__':
    
    try:
        talker()
    except ros.ROSInterruptException:
        pass
    
        