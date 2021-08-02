# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from copy import copy
import os
import pinocchio
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper


class FeasibilityAnalysis():

    def test_pitch_and_height(self, params, comp_dyn, step_height, des_height, pitch):
        print('current step height', step_height)
        print('current desired robot height', des_height)
        print('current desired pitch', pitch)
        start_point = [0.0, 0.0, des_height]
        mid_height = step_height/2.0 + des_height
        step_distance = 0.5
        mid_point = [step_distance, 0.0, mid_height]
        dist_from_goal = 2.0*step_distance
        goal_point = [dist_from_goal, 0.0, des_height + step_height]

        avg_pitch = np.arcsin(step_height/dist_from_goal)
        mid_pitch = pitch

        return self.test_trajectory(params, comp_dyn, des_height, mid_pitch, start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height)

    def setup_body_parameters(self, pin, body_length, links_length):
        urdf_hip_id_lf = 1
        urdf_hip_id_rf = 4
        urdf_hip_id_lh = 7
        urdf_hip_id_rh = 10

        pin.model.jointPlacements[urdf_hip_id_lf].translation[0] = body_length
        pin.model.jointPlacements[urdf_hip_id_rf].translation[0] = body_length
        pin.model.jointPlacements[urdf_hip_id_lh].translation[0] = -body_length
        pin.model.jointPlacements[urdf_hip_id_rh].translation[0] = -body_length

        FL_foot_frame_id = pin.model.getFrameId('FL_foot')
        HL_foot_frame_id = pin.model.getFrameId('HL_foot')
        FR_foot_frame_id = pin.model.getFrameId('FR_foot')
        HR_foot_frame_id = pin.model.getFrameId('HR_foot')
        FL_foot_frame = pin.model.frames[FL_foot_frame_id]
        HL_foot_frame = pin.model.frames[HL_foot_frame_id]
        FR_foot_frame = pin.model.frames[FR_foot_frame_id]
        HR_foot_frame = pin.model.frames[HR_foot_frame_id]
        FL_foot_frame.placement.translation[2] = -links_length
        HL_foot_frame.placement.translation[2] = -links_length
        FR_foot_frame.placement.translation[2] = -links_length
        HR_foot_frame.placement.translation[2] = -links_length
        # print(FL_foot_frame.placement.translation)
        # print(HL_foot_frame.placement.translation)
        # print(FR_foot_frame.placement.translation)
        # print(HR_foot_frame.placement.translation)

        FL_calf_frame = pin.model.jointPlacements[urdf_hip_id_lf+2]
        HL_calf_frame = pin.model.jointPlacements[urdf_hip_id_rf+2]
        FR_calf_frame = pin.model.jointPlacements[urdf_hip_id_lh+2]
        HR_calf_frame = pin.model.jointPlacements[urdf_hip_id_rh+2]
        FL_calf_frame.translation[2] = -links_length
        HL_calf_frame.translation[2] = -links_length
        FR_calf_frame.translation[2] = -links_length
        HR_calf_frame.translation[2] = -links_length
        # print(FL_calf_frame.translation)
        # print(HL_calf_frame.translation)
        # print(FR_calf_frame.translation)
        # print(HR_calf_frame.translation)
        return True

    def isInCollision(self, point, plane_eq):
        dist = np.dot(point, plane_eq[0:2]) - plane_eq[2]
        print('distance from point', dist)
        return dist > 0.0

    def test_body_vs_links_length(self, optimize_height_and_pitch, params, robot_name, step_height, body_length, links_length):
        print('current step height', step_height)
        print('current desired body length', body_length)
        print('current desired links length', links_length)

        PKG = os.path.dirname(os.path.abspath(
            __file__)) + '/../../resources/urdfs/lemo_EP0/'
        URDF = PKG + 'urdf/lemo_EP0.urdf'
        if PKG is None:
            pin = RobotWrapper.BuildFromURDF(URDF)
        else:
            pin = RobotWrapper.BuildFromURDF(URDF, [PKG])

        self.setup_body_parameters(pin, body_length, links_length)

        FL_foot_frame_id = pin.model.getFrameId('FL_foot')
        FL_foot_frame = pin.model.frames[FL_foot_frame_id]
        links_length = -FL_foot_frame.placement.translation[2]
        print('links lenght', links_length)
        des_height = links_length
        print('des height', des_height)
        start_point = [0.0, 0.0, des_height]
        mid_height = step_height/2.0 + des_height
        step_distance = 0.5
        mid_point = [step_distance, 0.0, mid_height]
        dist_from_goal = 2.0*step_distance
        goal_point = [dist_from_goal, 0.0, des_height + step_height]
        mid_pitch = -np.arcsin(step_height/dist_from_goal)

        comp_dyn = ComputationalDynamics(robot_name, pin)
        if optimize_height_and_pitch:
            return self.test_trajectory_with_variable_pitch_and_height(params, pin, comp_dyn, des_height, mid_pitch, start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height)
        else:
            params.setDefaultValuesWrtWorld(pin)
            return self.test_trajectory(params, comp_dyn, des_height, mid_pitch,
                                        start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height)

    def test_trajectory_with_variable_pitch_and_height(self, params, pin, comp_dyn, des_height, mid_pitch, start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height):
        pitch_min = mid_pitch*1.5
        pitch_max = mid_pitch*2.5
        N_pitch = 10
        pitch_range = np.linspace(pitch_min, pitch_max, num=N_pitch)
        des_height_min = des_height*0.75
        des_height_max = des_height*1.25
        N_height = 10
        height_range = np.linspace(
            des_height_min, des_height_max, num=N_height)
        for p in pitch_range:
            for h in height_range:
                print('pitch & height', p, h)
                params.setDefaultValuesWrtWorld(pin)
                res = self.test_trajectory(params, comp_dyn, h, p, start_point,
                                           mid_point, goal_point, dist_from_goal, step_distance, step_height)
                if res:
                    print('Feasible pitch and height values found!', p, h)
                    return True
        print('No feasible value of height and pitch found!', p, h, mid_pitch)
        return False

    def test_trajectory(self, params, comp_dyn, des_height, mid_pitch, start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height):
        tot_time = 20.0
        N = 50
        T = tot_time/float(N)
        base_lin_traj = np.vstack([np.linspace(
            start_point, mid_point, num=int(N/2)), np.linspace(mid_point, goal_point, num=int(N/2))])
        start_orient = [0.0, 0.0, 0.0]
        mid_orient = [0.0, mid_pitch, 0.0]
        goal_orient = start_orient
        base_orient_traj = np.vstack([np.linspace(
            start_orient, mid_orient, num=int(N/2)), np.linspace(mid_orient, goal_orient, num=int(N/2))])
        current_footholds = copy(params.getContactsPosWF())
        print('Initial footholds', current_footholds)
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
            print('====================> iter', i, 'time', t)
            com_des = np.array(
                [base_lin_traj[i, 0], base_lin_traj[i, 1], base_lin_traj[i, 2]])
            for leg in range(0, 4):
                if(t > next_liftoff_time[leg]):
                    current_footholds[leg][0] += step_lenght
                    step_idx[leg] += 1
                    next_touchdown_time[leg] = next_liftoff_time[leg] + \
                        swing_duration
                    next_liftoff_time[leg] += gait_duration
                    in_stance[leg] = False
                    if(current_footholds[leg][0] >= step_distance):
                        current_footholds[leg][2] = step_height
                if (t >= next_touchdown_time[leg]):
                    in_stance[leg] = True
            # print('stance legs number:', np.sum(np.array(in_stance)))
            # print('step times', next_liftoff_time)
            # print('touch down times', next_touchdown_time)
            # print('step_idx', step_idx)
            # print('current footholds', current_footholds)
            # print('in stance', in_stance)
            params.setCoMPosWF(com_des)
            print('current CoM pos', com_des)
            euler_angles = np.array(
                [base_orient_traj[i, 0], base_orient_traj[i, 1], base_orient_traj[i, 2]])
            params.setEulerAngles(euler_angles)
            # print('current base orient', euler_angles)
            LF_foot = current_footholds[0]
            RF_foot = current_footholds[1]
            LH_foot = current_footholds[2]
            RH_foot = current_footholds[3]
            contactsWF = np.vstack(
                (LF_foot, RF_foot, LH_foot, RH_foot))
            print('current contacts wrt WF', contactsWF)
            params.setContactsPosWF(contactsWF)
            params.setActiveContacts(in_stance)
            ''' compute iterative projection
            Outputs of "iterative_projection_bretl" are:
            IP_points = resulting 2D vertices
            actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
            computation_time = how long it took to compute the iterative projection
            '''
            try:
                IP_points, force_polytopes, IP_computation_time, joints_pos, knee_pos, hips_pos = comp_dyn.iterative_projection_bretl(
                    params)
                comp_geom = ComputationalGeometry()
                facets = comp_geom.compute_halfspaces_convex_hull(
                    IP_points)
                point2check = comp_dyn.getReferencePoint(params, "ZMP")
                isPointFeasible, margin = comp_geom.isPointRedundant(
                    facets, point2check)
                margin_list.append(margin)
                time_list.append(t)
            except:
                print('exception caught! Terminate trajectory')
                return False

            check_collision = True
            collision_margin = 0.05
            corner_point = [step_distance, step_height]
            current_pitch = euler_angles[1]
            base_plane_eq = [np.sin(current_pitch), np.cos(current_pitch)]
            hip_plane_eq = [-np.cos(current_pitch), -np.sin(current_pitch)]
            com_xz = [com_des[0], com_des[2]]
            point2test = com_xz - collision_margin*np.array(base_plane_eq)
            print('point 2 test', point2test)
            offset_term = np.dot(point2test, base_plane_eq)
            print('hips', hips_pos[0][0])
            hip_offset_term = np.dot(
                [hips_pos[0][0], hips_pos[0][2]], hip_plane_eq)
            print('offset', offset_term, hip_offset_term)
            base_plane_eq = [base_plane_eq[0], base_plane_eq[1], offset_term]
            hip_plane_eq = [hip_plane_eq[0], hip_plane_eq[1], hip_offset_term]
            if check_collision and self.isInCollision(corner_point, base_plane_eq) and self.isInCollision(corner_point, hip_plane_eq):
                print('Collision detected!')
                return False

        return True
