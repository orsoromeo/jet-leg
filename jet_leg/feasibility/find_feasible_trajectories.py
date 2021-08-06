# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 10:54:31 2018

@author: Romeo Orsolino
"""

import numpy as np
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from copy import copy
from copy import deepcopy
import os
import pinocchio
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
from scipy.spatial.transform import Rotation as Rot


class FeasibilityAnalysis():
    def __init__(self, pinocchio):
        self.pin = pinocchio

    def setup_kinematic_lims(self, hip_x_min, hip_y_min, knee_min):
        hip_x_range = self.pin.model.upperPositionLimit[0] - \
            self.pin.model.lowerPositionLimit[0]
        hip_x_max = hip_x_min + hip_x_range

        hip_y_range = self.pin.model.upperPositionLimit[1] - \
            self.pin.model.lowerPositionLimit[1]
        hip_y_max = hip_y_min + hip_y_range
        knee_range = self.pin.model.upperPositionLimit[2] - \
            self.pin.model.lowerPositionLimit[2]
        knee_max = knee_min + knee_range

        for leg in range(0, 4):
            self.pin.model.lowerPositionLimit[leg*3] = hip_x_min
            self.pin.model.lowerPositionLimit[1+leg*3] = hip_y_min
            self.pin.model.lowerPositionLimit[2+leg*3] = knee_min

            self.pin.model.upperPositionLimit[leg*3] = hip_x_max
            self.pin.model.upperPositionLimit[1+leg*3] = hip_y_max
            self.pin.model.upperPositionLimit[2+leg*3] = knee_max
        return True

    def setup_torque_lims(self, hip_x_lim, hip_y_lim, knee_lim):
        for leg in range(0, 4):
            self.pin.model.effortLimit[leg*3] = hip_x_lim
            self.pin.model.effortLimit[1+leg*3] = hip_y_lim
            self.pin.model.effortLimit[2+leg*3] = knee_lim
        return True

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

    def kneeStepCollision(self, step_distance, step_height, knee_pos):
        e_v = [0, -1]
        e_h = [1, 0]

        if knee_pos[0] >= step_distance:
            step_edge = [step_distance, step_height]
            h_offset = np.dot(step_edge, e_v)
            h_plane_eq = [e_v[0], e_v[1], h_offset]
            v_offset = np.dot(step_edge, e_h)
            v_plane_eq = [e_h[0], e_h[1], v_offset]
            knee_pos_xz = [knee_pos[0], knee_pos[2]]
            return self.isInCollision(knee_pos_xz, h_plane_eq) and self.isInCollision(knee_pos_xz, v_plane_eq)
        else:
            step_edge = [step_distance, 0.0]
            h_offset = np.dot(step_edge, e_v)
            h_plane_eq = [e_v[0], e_v[1], h_offset]
            v_offset = np.dot(step_edge, -np.array(e_h))
            v_plane_eq = [e_h[0], e_h[1], v_offset]
            knee_pos_xz = [knee_pos[0], knee_pos[2]]
            return self.isInCollision(knee_pos_xz, h_plane_eq) and self.isInCollision(knee_pos_xz, v_plane_eq)

    def isInCollision(self, point, plane_eq):
        dist = np.dot(point, plane_eq[0:2]) - plane_eq[2]
        print('distance from point', dist)
        return dist > 0.0

    def test_trajectory_with_variable_pitch_and_height(self, params, pin, des_height, mid_pitch, start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height):
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
                res = self.test_trajectory(params, params.compDyn, h, p, start_point,
                                           mid_point, goal_point, dist_from_goal, step_distance, step_height)
                if res:
                    print('Feasible pitch and height values found!', p, h)
                    return True
        print('No feasible value of height and pitch found!', p, h, mid_pitch)
        return False

    def make_footholds(self, N, T, params, dist_from_goal, step_distance, step_height, is_trot):
        footholds = np.array([[[0.0]*3]*N]*4)
        in_stance = np.array([[True]*N]*4)
        if is_trot:
            phase_offsets = np.array([0.0, 0.5, 0.5, 0.0])
            duty_factor = 0.75
        else:
            phase_offsets = np.array([0.0, 0.5, 0.25, 0.75])
            duty_factor = 0.75
        swing_duration = 0.4
        gait_duration = swing_duration/(1.0 - duty_factor)
        tot_time = N*T
        avg_speed = dist_from_goal/tot_time
        step_lenght = avg_speed*gait_duration
        step_idx = [0, 0, 0, 0]
        next_liftoff_time = phase_offsets*gait_duration
        next_touchdown_time = next_liftoff_time+swing_duration
        current_footholds = copy(params.getContactsPosWF())
        for i in range(0, N):
            t = i*T
            for leg in range(0, 4):
                footholds[leg][i] = copy(current_footholds[leg])
                if(t > next_liftoff_time[leg]):
                    current_footholds[leg][0] += step_lenght
                    step_idx[leg] += 1
                    next_touchdown_time[leg] = next_liftoff_time[leg] + \
                        swing_duration
                    next_liftoff_time[leg] += gait_duration
                    in_stance[leg][i] = False
                    if(current_footholds[leg][0] >= step_distance):
                        current_footholds[leg][2] = step_height
                if (t >= next_touchdown_time[leg]):
                    in_stance[leg][i] = True
        return footholds, in_stance

    def make_base_trot_traj(self, N, start_point, mid_point, goal_point):
        base_lin_traj = np.vstack([np.linspace(
            start_point, mid_point, num=int(N/2)), np.linspace(mid_point, goal_point, num=int(N/2))])
        return base_lin_traj

    def make_base_walk_traj(self, footholds_traj, stance_traj, des_height):
        N = len(footholds_traj[0])
        base_lin_traj = [[0.0]*3]*N
        print('des height', des_height)
        for i in range(0, N):
            n_stance_legs = 0
            for leg in range(0, 4):
                if stance_traj[leg][i]:
                    n_stance_legs += 1
                    base_lin_traj[i] += footholds_traj[leg][i]
            base_lin_traj[i] = base_lin_traj[i]/n_stance_legs
            base_lin_traj[i][2] += des_height

        return base_lin_traj

    def test_trajectory(self, params, des_height, mid_pitch, start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height):
        tot_time = 20.0
        N = 50
        T = tot_time/float(N)

        min_margin = -0.01
        is_trot = False

        footholds_traj, in_stance_traj = self.make_footholds(
            N, T, params, dist_from_goal, step_distance, step_height, is_trot)

        if is_trot:
            base_lin_traj = self.make_base_trot_traj(
                N, start_point, mid_point, goal_point)
        else:
            base_lin_traj = self.make_base_walk_traj(
                footholds_traj, in_stance_traj, des_height)

        print('base linear traj', base_lin_traj)

        start_orient = [0.0, 0.0, 0.0]
        mid_orient = [0.0, mid_pitch, 0.0]
        goal_orient = start_orient
        base_orient_traj = np.vstack([np.linspace(
            start_orient, mid_orient, num=int(N/2)), np.linspace(mid_orient, goal_orient, num=int(N/2))])

        for i in range(0, N):
            t = i*T
            print('====================> iter', i, 'time', t)
            com_des = base_lin_traj[i]
            params.setCoMPosWF(com_des)
            print('current CoM pos', com_des)
            euler_angles = np.array(
                [base_orient_traj[i, 0], base_orient_traj[i, 1], base_orient_traj[i, 2]])
            params.setEulerAngles(euler_angles)
            print('current base orient', euler_angles)
            LF_foot = footholds_traj[0][i]
            RF_foot = footholds_traj[1][i]
            LH_foot = footholds_traj[2][i]
            RH_foot = footholds_traj[3][i]
            contactsWF = np.vstack(
                (LF_foot, RF_foot, LH_foot, RH_foot))
            print('current contacts wrt WF', contactsWF)
            params.setContactsPosWF(contactsWF)
            in_stance = [in_stance_traj[leg][i] for leg in range(0, 4)]
            params.setActiveContacts(in_stance)
            ''' compute iterative projection
            Outputs of "iterative_projection_bretl" are:
            IP_points = resulting 2D vertices
            actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
            computation_time = how long it took to compute the iterative projection
            '''
            try:
                IP_points, force_polytopes, IP_computation_time, joints_pos, knee_pos, hips_pos = params.compDyn.iterative_projection_bretl(
                    params)
                comp_geom = ComputationalGeometry()
                facets = comp_geom.compute_halfspaces_convex_hull(
                    IP_points)
                point2check = params.compDyn.getReferencePoint(params, "ZMP")
                isPointFeasible, margin = comp_geom.isPointRedundant(
                    facets, point2check)
                print('Margin is', margin)
                if margin < min_margin:
                    print('Margin is too low!')
                    return False
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
            offset_term = np.dot(point2test, base_plane_eq)
            hip_offset_term = np.dot(
                [hips_pos[0][0], hips_pos[0][2]], hip_plane_eq)
            base_plane_eq = [base_plane_eq[0], base_plane_eq[1], offset_term]
            hip_plane_eq = [hip_plane_eq[0], hip_plane_eq[1], hip_offset_term]
            if check_collision and self.isInCollision(corner_point, base_plane_eq) and self.isInCollision(corner_point, hip_plane_eq):
                print('Base collision detected!')
                return False

            rot = Rot.from_euler('xyz', euler_angles, degrees=False)
            W_R_B = rot.as_matrix()
            if check_collision:
                for leg in range(0, 4):
                    knee_pos_WF = W_R_B.dot(knee_pos[leg]) + com_des
                    col = self.kneeStepCollision(
                        step_distance, step_height, knee_pos_WF)
                    if col:
                        print('Knee collision detected on leg', leg)
                        return False

        return True
