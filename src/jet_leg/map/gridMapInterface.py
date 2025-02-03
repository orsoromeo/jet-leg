# -*- coding: utf-8 -*-
"""
Created on Sat Nov 24 15:09:35 2018

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
from grid_map_msgs.msg import GridMap
from dls_msgs.msg import SimpleDoubleArray, StringDoubleArray, Polygon3D, LegsPolygons, HeightMap
from dwl_msgs.msg import WholeBodyState, WholeBodyTrajectory, JointState, ContactState, BaseState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Header
from std_srvs.srv import Empty
from termcolor import colored

#from context import jet_leg 
from computational_dynamics import ComputationalDynamics
from math_tools import Math
from grid_map_converter import gridMapConverter


stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
#import utils
sys.stderr = stderr


class HyQSim(threading.Thread):
    def __init__(self):  
        
        threading.Thread.__init__(self)
        
        self.clock_sub_name = 'clock'
        self.hyq_wbs_sub_name = "/hyq/robot_states"
        self.gridMapTopicName = "/local_gridmap"

        self.hyq_wbs = dict()
        self.hyq_rcf_debug = GridMap()  
        self.actuation_polygon_topic_name = "/hyq/actuation_polygon"
        self.support_region_topic_name = "/hyq/support_region"
        self.force_polygons_topic_name = "/hyq/force_polygons"


        self.sim_time  = 0.0
        
    def run(self):
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1)
#        self.sub_wbs = ros.Subscriber(self.hyq_wbs_sub_name, WholeBodyState, callback=self._reg_sim_wbs, queue_size=1)
        self.sub_actuation_params = ros.Subscriber(self.gridMapTopicName, GridMap, callback=self._reg_sim_rcf_debug, queue_size=1)

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
        print 'new data received'
        self.hyq_rcf_debug = copy.deepcopy(msg)  
        
    def register_node(self):
        ros.init_node('sub_pub_node_python', anonymous=False)

    def deregister_node(self):
        ros.signal_shutdown("manual kill")
        
    def get_sim_time(self):
        return self.sim_time
        
    def get_sim_wbs(self):
        return self.hyq_wbs

def talker():

    gridmap = GridMap()
    math = Math()
    p=HyQSim()
    p.start()
    p.register_node()
    name = "Actuation_region"
    point = Point()
    polygonVertex = Point()
    actPolygon = Polygon3D()
    converter = gridMapConverter()
    i = 0

    for j in range(0,50):
        vertices1 = [point]
        actPolygons = [polygonVertex]
#        poly = []
#        print("Time: " + str(i*0.004) + "s and Simulation time: " + str(p.get_sim_time()/60))
        p.get_sim_wbs()
        converter.getParamsFromRosDebugTopic(p.hyq_rcf_debug)
        trunk_mass = 85.
        mu = 0.8
        ng = 4
        axisZ= np.array([[0.0], [0.0], [1.0]])
        ''' normals '''    
        n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        normals = np.vstack([n1, n2, n3, n4])

        print 'contacts: ',1
            
        
        time.sleep(1.0/25.0)
        i+=1
        
    print 'de registering...'
    p.deregister_node()
        

if __name__ == '__main__':
    
    try:
        talker()
    except ros.ROSInterruptException:
        pass
    
        