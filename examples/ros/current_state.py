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

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point
from dls_msgs.msg import StringDoubleArray
from feasible_region.msg import Polygon3D
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.foothold_planning_interface import FootholdPlanningInterface

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
        self.support_region_topic_name = "/hyq/support_region"


        self.sim_time = 0.0

    def run(self):
        print "Run!"
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1000)
        self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, StringDoubleArray,
                                                   callback=self._reg_sim_rcf_debug, queue_size=1000)

        self.pub_support_region = ros.Publisher(self.support_region_topic_name, Polygon3D, queue_size=1000)

    def _reg_sim_time(self, time):
        self.sim_time = time.clock.secs + time.clock.nsecs / 1000000000.0

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

    def send_support_region(self, name, vertices):
        output = Polygon3D()
        #        output.names = name
        output.vertices = vertices
        self.pub_support_region.publish(output)

    def fillPolygon(self, polygon):
        num_actuation_vertices = np.size(polygon, 0)
        vertices = []

        for i in range(0, num_actuation_vertices):
            point = Point()
            point.x = polygon[i][0]
            point.y = polygon[i][1]
            point.z = 0.0  # is the centroidal frame
            vertices = np.hstack([vertices, point])
        return vertices


def talker():
    compDyn = ComputationalDynamics()

    p = HyQSim()
    p.start()
    p.register_node()
    name = "Support region"

    params = IterativeProjectionParameters()
    foothold_params = FootholdPlanningInterface()
    i = 0

    p.get_sim_wbs()
    params.getParamsFromRosDebugTopic(p.hyq_rcf_debug)
    foothold_params.getParamsFromRosDebugTopic(p.hyq_rcf_debug)
    params.getCurrentStanceFeetFlags(p.hyq_rcf_debug)
    params.getCurrentFeetPos(p.hyq_rcf_debug)

    """ contact points """
    ng = 4
    params.setNumberOfFrictionConesEdges(ng)

    while not ros.is_shutdown():

        p.get_sim_wbs()
        params.getParamsFromRosDebugTopic(p.hyq_rcf_debug)
        foothold_params.getParamsFromRosDebugTopic(p.hyq_rcf_debug)
        params.getCurrentStanceFeetFlags(p.hyq_rcf_debug)
        # params.getCurrentFeetPosFlags(p.hyq_rcf_debug)

        #         ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
        #        3 - FRICTION REGION
        constraint_mode_IP = 'ONLY_FRICTION'
        params.setConstraintModes([constraint_mode_IP,
                               constraint_mode_IP,
                               constraint_mode_IP,
                               constraint_mode_IP])
        params.setNumberOfFrictionConesEdges(ng)
        
        frictionRegionWF, actuation_polygons, computation_time = compDyn.try_iterative_projection_bretl(params)
        print frictionRegionWF
        p.send_support_region(name, p.fillPolygon(frictionRegionWF))

        time.sleep(0.05)
        i += 1

    print 'de registering...'
    p.deregister_node()


if __name__ == '__main__':

    try:
        talker()
    except ros.ROSInterruptException:
        pass

