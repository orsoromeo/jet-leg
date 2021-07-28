# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry

from copy import copy

class FeasibilityAnalysis():
    def test_trajectory(self, params, comp_dyn, step_height, des_height, pitch):
        print('current step height', step_height)
        print('current desired robot height', des_height)
        print('current desired pitch', pitch)
        start_point = [0.0, 0.0, des_height]
        mid_height = step_height/2.0 + 0.33
        step_distance = 0.5
        mid_point = [step_distance, 0.0, mid_height]
        dist_from_goal = 2.0*step_distance
        goal_point = [dist_from_goal, 0.0, des_height + step_height]

        avg_pitch = np.arcsin(des_height/dist_from_goal)
        mid_pitch = pitch

        tot_time = 20.0
        N = 50
        T = tot_time/float(N)
        base_lin_traj = np.vstack([np.linspace(start_point , mid_point, num=N/2), np.linspace(mid_point , goal_point, num=N/2)])

        start_orient = [ 0.0, 0.0, 0.0]
        mid_orient = [0.0, mid_pitch, 0.0]
        goal_orient = start_orient
        base_orient_traj = np.vstack([np.linspace(start_orient , mid_orient, num=N/2), np.linspace(mid_orient , goal_orient, num=N/2)])
        current_footholds = copy(params.getContactsPosWF())
        in_stance = [True, True, True, True]

        phase_offsets = np.array([0.0, 0.5, 0.5, 0.0])
        duty_factor = 0.75
        swing_duration = 0.4
        gait_duration = swing_duration/(1.0 - duty_factor)
        tot_time = N*T
        avg_speed = dist_from_goal/tot_time
        # print 'gait duration', gait_duration
        # print 'avg speed', avg_speed
        step_lenght = avg_speed*gait_duration
        # print 'step length', step_lenght
        step_idx = [0, 0, 0, 0]
        next_liftoff_time = phase_offsets*gait_duration
        next_touchdown_time = next_liftoff_time+swing_duration

        margin_list = list()
        time_list = list()
        for i in range(0, N):
            t = i*T
            print '====================> iter',i, 'time', t
            com_des = np.array([base_lin_traj[i, 0], base_lin_traj[i, 1], base_lin_traj[i, 2]])
            for leg in range(0, 4):
                if(t > next_liftoff_time[leg]):
                    current_footholds[leg][0] += step_lenght
                    step_idx[leg] += 1
                    next_touchdown_time[leg] = next_liftoff_time[leg] + swing_duration
                    next_liftoff_time[leg] += gait_duration
                    in_stance[leg] = False

                    if(current_footholds[leg][0] >= step_distance):
                        current_footholds[leg][2] = step_height

                if (t >= next_touchdown_time[leg]):
                    in_stance[leg] = True

            # print 'stance legs number:',np.sum(np.array(in_stance))
            # print 'step times', next_liftoff_time
            # print 'touch down times', next_touchdown_time
            # print 'step_idx', step_idx
            # print 'current footholds', current_footholds
            # print 'in stance', in_stance

            params.setCoMPosWF(com_des)
            # print 'current CoM pos', com_des
            euler_angles = np.array([base_orient_traj[i, 0], base_orient_traj[i, 1], base_orient_traj[i, 2]])
            params.setEulerAngles(euler_angles)
            # print 'current base orient', euler_angles
            LF_foot = current_footholds[0]
            RF_foot = current_footholds[1]
            LH_foot = current_footholds[2]
            RH_foot = current_footholds[3]

            contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
            # print 'current contacts wrt WF', contactsWF
            params.setContactsPosWF(contactsWF)
            params.setActiveContacts(in_stance)
            ''' compute iterative projection
            Outputs of "iterative_projection_bretl" are:
            IP_points = resulting 2D vertices
            actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
            computation_time = how long it took to compute the iterative projection
            '''
            try:
                IP_points, force_polytopes, IP_computation_time, joints_pos, knee_pos, hips_pos = comp_dyn.iterative_projection_bretl(params)
                comp_geom = ComputationalGeometry()
                facets = comp_geom.compute_halfspaces_convex_hull(IP_points)
                point2check = comp_dyn.getReferencePoint(params, "ZMP")
                isPointFeasible, margin = comp_geom.isPointRedundant(facets, point2check)
                # print "isPointFeasible: ", isPointFeasible
                # print "Margin is: ", margin
                margin_list.append(margin)
                time_list.append(t)
            except:
                print('exception caught! Terminate trajectory')
                return False

        return True