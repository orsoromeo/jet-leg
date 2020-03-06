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

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Point
from xpp_msgs.msg import RobotStateCartesian
import std_msgs
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.computational_geometry.robotStateCartesianInterface import RobotStateCartesianInterface

stderr = sys.stderr
sys.stderr = open(os.devnull, 'w')
sys.stderr = stderr


class HyQSim(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

        self.clock_sub_name = 'clock'
        self.hyq_actuation_params_sub_name = "/xpp/state_des"
        self.hyq_wbs = dict()
        self.hyq_debug_msg = RobotStateCartesian()
        print ros.get_namespace()
        self.sim_time = 0.0
        #self.params = IterativeProjectionParameters("anymal")
        self.cartesianStateParams = RobotStateCartesianInterface("anymal")
        self.compDyn = ComputationalDynamics("anymal");
        self.compGeom = ComputationalGeometry()

    def run(self):
        print "Run!"
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1000)
        self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, RobotStateCartesian,
                                                   callback=self.callback_hyq_debug, queue_size=1000)
        self.margin_pub = ros.Publisher('analytic_margin', std_msgs.msg.Float64, queue_size=10)

        print "callbacks created"

    def _reg_sim_time(self, time):
        self.sim_time = time.clock.secs + time.clock.nsecs / 1000000000.0

    #        print("getting time")

    def _reg_sim_wbs(self, msg):
        self.hyq_wbs = copy.deepcopy(msg)

    def callback_hyq_debug(self, msg):
        print 'new robot state cartesian data received'
        #self.cartesianStateParams.getStateFromRobotStateCartesian(copy.deepcopy(msg))
        self.cartesianStateParams.getStateFromRobotStateCartesianWrtBaseFrame(copy.deepcopy(msg))
        #print "com position ", self.cartesianStateParams.getCoMPosWF()
        start_time = time.time()
        IP_points, force_polytopes, IP_computation_time = self.compDyn.iterative_projection_bretl(self.cartesianStateParams)
        rows, cols = np.shape(IP_points)
        print "number of IP points", rows
        if rows>2:
            facets = self.compGeom.compute_halfspaces_convex_hull(IP_points)
            print IP_points
            self.cartesianStateParams.useInstantaneousCapturePoint = True
            #point2check = self.compDyn.getReferencePointWrtBaseFrame(self.cartesianStateParams)
            #print "ICP is ", point2check
            ref = [0.0, 0.0]
            print "reference point is ", ref
            #isPointFeasible, icp_margin = self.compGeom.isPointRedundant(facets, point2check)
            isPointFeasible, com_margin = self.compGeom.isPointRedundant(facets, ref)
            margin = com_margin
            #print "icp margin ", icp_margin
            print "com margin ", com_margin
            print "margin is: ", margin
        else:
            print "==========> Feasible region has zero volume"
            margin = -100.0

        print "total time needed to compute the margin ", time.time() - start_time
        self.margin_pub.publish(margin)
        #self.hyq_debug_msg = copy.deepcopy(msg)

    def register_node(self):
        ros.init_node('sub_pub_node_python', anonymous=False)

    def deregister_node(self):
        ros.signal_shutdown("manual kill")

    def get_sim_time(self):
        return self.sim_time

    def get_sim_wbs(self):
        return self.hyq_wbs

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


def talker(robotName):
    print "talker"

    p = HyQSim()

    p.start()
    p.register_node()

    i = 0
    p.get_sim_wbs()

    """ contact points """
    ng = 4
    #p.params.setNumberOfFrictionConesEdges(ng)

    while not ros.is_shutdown():

        p.get_sim_wbs()

        #cartesianStateInt.getStateFromRobotStateCartesian(p.hyq_debug_msg)

        # USE THIS ONLY TO PLOT THE ACTUAL REGION FOR A VIDEO FOR THE PAPER DO NOT USE FOR COM PLANNING
        #p.params.setConstraintModes(['FRICTION_AND_ACTUATION',
        #                           'FRICTION_AND_ACTUATION',
        #                           'FRICTION_AND_ACTUATION',
        #                           'FRICTION_AND_ACTUATION'])


        time.sleep(0.1)
        i += 1

    print 'de registering...'
    p.deregister_node()


if __name__ == '__main__':

    try:
        robot_name = 'hyq'
        talker(robot_name)
    except ros.ROSInterruptException:
        pass

