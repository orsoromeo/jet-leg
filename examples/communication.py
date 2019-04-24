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
from dls_msgs.msg import  StringDoubleArray
from feasible_region.msg import  Polygon3D, LegsPolygons


from context import jet_leg 
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.math_tools import Math
from jet_leg.iterative_projection_parameters import IterativeProjectionParameters
from jet_leg.foothold_planning_interface import FootholdPlanningInterface

from jet_leg.foothold_planning import FootHoldPlanning

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
        self.hyq_debug_msg = StringDoubleArray()
        self.actuation_polygon_topic_name = "/feasible_region/actuation_polygon"
        self.support_region_topic_name = "/feasible_region/support_region"
        self.force_polygons_topic_name = "/feasible_region/force_polygons"
        print ros.get_namespace()
        self.sim_time  = 0.0

    def run(self):
        print "Run!"
        self.sub_clock = ros.Subscriber(self.clock_sub_name, Clock, callback=self._reg_sim_time, queue_size=1000)
        self.sub_actuation_params = ros.Subscriber(self.hyq_actuation_params_sub_name, StringDoubleArray,
                                                   callback=self.callback_hyq_debug, queue_size=1000)
        self.pub_polygon = ros.Publisher(self.actuation_polygon_topic_name, Polygon3D, queue_size=10000)
        self.pub_support_region = ros.Publisher(self.support_region_topic_name, Polygon3D, queue_size=1000)
        self.pub_force_polygons = ros.Publisher(self.force_polygons_topic_name, LegsPolygons, queue_size=1000)

    def _reg_sim_time(self, time):
        self.sim_time = time.clock.secs + time.clock.nsecs/1000000000.0
#        print("getting time")

    def _reg_sim_wbs(self, msg):
        self.hyq_wbs = copy.deepcopy(msg)

    def callback_hyq_debug(self, msg):
#        print 'new data received'
        self.hyq_debug_msg = copy.deepcopy(msg)

    def register_node(self):
        ros.init_node('sub_pub_node_python', anonymous=False)

    def deregister_node(self):
        ros.signal_shutdown("manual kill")

    def get_sim_time(self):
        return self.sim_time

    def get_sim_wbs(self):
        return self.hyq_wbs

    def send_force_polytopes(self, name, polygons):
        output = LegsPolygons()
        output.polygons = polygons
        self.pub_force_polygons.publish(output)

    def send_support_region(self, name, vertices):
        output = Polygon3D()
        output.vertices = vertices
        self.pub_support_region.publish(output)

    def send_actuation_polygons(self, name, vertices, option_index, ack_optimization_done):
        output = Polygon3D()
        output.vertices = vertices
        output.option_index = option_index
        output.ack_optimization_done = ack_optimization_done
        self.pub_polygon.publish(output)

    def fillPolygon(self, polygon):

        num_actuation_vertices = np.size(polygon, 0)
        vertices = []

        for i in range(0, num_actuation_vertices):
            point = Point()
            point.x = polygon[i][0]
            point.y = polygon[i][1]
            point.z = 0.0 #is the centroidal frame
            vertices = np.hstack([vertices, point])
        return vertices

def talker():
    compDyn = ComputationalDynamics()
    footHoldPlanning = FootHoldPlanning()
    math = Math()
    p=HyQSim()
    p.start()
    p.register_node()
    name = "Actuation_region"
    force_polytopes_name = "force_polytopes"

    params = IterativeProjectionParameters()
    foothold_params = FootholdPlanningInterface()
    i = 0

    p.get_sim_wbs()
    params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
    foothold_params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
    params.getFutureStanceFeet(p.hyq_debug_msg)
   

    """ contact points """
    ng = 4
    params.setNumberOfFrictionConesEdges(ng)    




    while not ros.is_shutdown():


        p.get_sim_wbs()

        params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
        foothold_params.getParamsFromRosDebugTopic(p.hyq_debug_msg)
        #params.getFutureStanceFeet(p.hyq_debug_msg)
        params.getCurrentStanceFeet(p.hyq_debug_msg)
        
        # USE THIS ONLY TO PLOT THE ACTUAL REGION FOR A VIDEO FOR THE PAPER DO NOT USE FOR COM PLANNING
        params.setConstraintModes(['FRICTION_AND_ACTUATION',
                           'FRICTION_AND_ACTUATION',
                           'FRICTION_AND_ACTUATION',
                           'FRICTION_AND_ACTUATION'])
        IAR, actuation_polygons_array, computation_time = compDyn.try_iterative_projection_bretl(params)
        # print 'feasible region', IAR
#        if IAR is not False:
#            p.send_actuation_polygons(name, p.fillPolygon(IAR), foothold_params.option_index, foothold_params.ack_optimization_done)
#            old_IAR = IAR
#        else:
#            print 'Could not compute the feasible region'
#            p.send_actuation_polygons(name, p.fillPolygon(old_IAR), foothold_params.option_index,
#                                      foothold_params.ack_optimization_done)
##   
        p.send_actuation_polygons(name, p.fillPolygon(IAR), foothold_params.option_index, foothold_params.ack_optimization_done)
                                     
        constraint_mode_IP = 'ONLY_FRICTION'
        params.setConstraintModes([constraint_mode_IP,
                                       constraint_mode_IP,
                                       constraint_mode_IP,
                                       constraint_mode_IP])
        params.setNumberOfFrictionConesEdges(ng)
            
        params.contactsWF[params.actual_swing] = foothold_params.footOptions[foothold_params.option_index]

            #        uncomment this if you dont want to use the vars read in iterative_proJ_params
            #        params.setContactNormals(normals)
            #        params.setFrictionCoefficient(mu)
            #        params.setTrunkMass(trunk_mass)
            #        IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)

        frictionRegion, actuation_polygons, computation_time = compDyn.iterative_projection_bretl(params)
        p.send_support_region(name, p.fillPolygon(frictionRegion))         
       

        
        #print "AA"
        
        #1 - INSTANTANEOUS FEASIBLE REGION    
        # ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION


        #IAR, actuation_polygons_array, computation_time = compDyn.iterative_projection_bretl(params)
        #print 'feasible region', IAR, 
        #p.send_actuation_polygons(name, p.fillPolygon(IAR), foothold_params.option_index, foothold_params.ack_optimization_done)
         

         
         
         
        #2 - FORCE POLYGONS
         #point = Point()
         #polygonVertex = Point()
         #polygon = Polygon3D()
#        point.x = actuation_polygons_array[0][0][0]/1000.0
#        point.y = actuation_polygons_array[0][1][0]/1000.0
#        point.z = actuation_polygons_array[0][2][0]/1000.0
                
#        forcePolygons = []
#        for i in range(0,4):
#            singlePolygon = Polygon3D()
##            print actuation_polygons_array[i]
#            vertices = []
#            for j in range(0,8):    
#                vx = Point()
#                vx.x = actuation_polygons_array[i][0][j]/1000.0
#                vx.y = actuation_polygons_array[i][1][j]/1000.0
#                vx.z = actuation_polygons_array[i][2][j]/1000.0
#                vertices = np.hstack([vertices, vx])       
#            singlePolygon.vertices = vertices      
#            forcePolygons = np.hstack([forcePolygons, singlePolygon])
#        p.send_force_polytopes(force_polytopes_name, forcePolygons)


        #4 - FOOTHOLD PLANNING
   
        
        #print 'opt started?', foothold_params.optimization_started
        #print 'ack opt done', foothold_params.ack_optimization_done
#        foothold_params.ack_optimization_done = True 
        actuationRegions = []
#        print 'robot mass', params.robotMass
        if (foothold_params.optimization_started == False):
            foothold_params.ack_optimization_done = False
        # print 'optimization done',foothold_params.ack_optimization_done, ' ... ', foothold_params.optimization_started
        if foothold_params.optimization_started and not foothold_params.ack_optimization_done :
            print '============================================================'
            print 'current swing ', params.actual_swing            
            print '============================================================'

            #print foothold_params.footOptions
            
            #chosen_foothold, actuationRegions = footHoldPlanning.selectMaximumFeasibleArea(foothold_params, params)
#            print 'current swing ',params.actual_swing
            foothold_params.option_index, stackedResidualRadius, actuationRegions, mapFootHoldIdxToPolygonIdx = footHoldPlanning.selectMaximumFeasibleArea( foothold_params, params)

            if actuationRegions is False:
                foothold_params.option_index = -1
            else:
                print 'min radius ', foothold_params.minRadius, 'residual radius ', stackedResidualRadius
                #print 'feet options', foothold_params.footOptions
                print 'final index', foothold_params.option_index, 'index list', mapFootHoldIdxToPolygonIdx
                
            foothold_params.ack_optimization_done = 1

            #         ONLY_ACTUATION, ONLY_FRICTION or FRICTION_AND_ACTUATION
            #        3 - FRICTION REGION
            constraint_mode_IP = 'ONLY_FRICTION'
            params.setConstraintModes([constraint_mode_IP,
                                       constraint_mode_IP,
                                       constraint_mode_IP,
                                       constraint_mode_IP])
            params.setNumberOfFrictionConesEdges(ng)
            
            params.contactsWF[params.actual_swing] = foothold_params.footOptions[foothold_params.option_index]

            #        uncomment this if you dont want to use the vars read in iterative_proJ_params
            #        params.setContactNormals(normals)
            #        params.setFrictionCoefficient(mu)
            #        params.setTrunkMass(trunk_mass)
            #        IP_points, actuation_polygons, comp_time = comp_dyn.support_region_bretl(stanceLegs, contacts, normals, trunk_mass)

            frictionRegion, actuation_polygons, computation_time = compDyn.iterative_projection_bretl(params)
            p.send_support_region(name, p.fillPolygon(frictionRegion))

            #this sends the data back to ros that contains the foot hold choice (used for stepping) and the corrspondent region (that will be used for com planning TODO update with the real footholds)
            if (actuationRegions is not False) and (np.size(actuationRegions) is not 0):
                print 'sending actuation region'
                p.send_actuation_polygons(name, p.fillPolygon(actuationRegions[-1]), foothold_params.option_index, foothold_params.ack_optimization_done)
                # print actuationRegions[-1]
            else:
                #if it cannot compute anything it will return the frictin region
                p.send_actuation_polygons(name, p.fillPolygon(frictionRegion), foothold_params.option_index, foothold_params.ack_optimization_done)


        time.sleep(0.1)
        i+=1
        
    print 'de registering...'
    p.deregister_node()
        

if __name__ == '__main__':
    
    try:
        talker()
    except ros.ROSInterruptException:
        pass
    
        