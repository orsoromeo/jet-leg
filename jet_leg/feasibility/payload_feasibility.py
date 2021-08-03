import numpy as np
from jet_leg.feasibility.find_feasible_trajectories import FeasibilityAnalysis
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics


class PayloadFeasibility(FeasibilityAnalysis):
    def test_payload_amplitude_vs_payload_pos(self, optimize_height_and_pitch, params, robot_name, step_height, vertical_force, horizontal_location):
        print('current step height', step_height)

        des_height = 0.38
        start_point = [0.0, 0.0, des_height]
        mid_height = step_height/2.0 + des_height
        step_distance = 0.5
        mid_point = [step_distance, 0.0, mid_height]
        dist_from_goal = 2.0*step_distance
        goal_point = [dist_from_goal, 0.0, des_height + step_height]
        mid_pitch = -np.arcsin(step_height/dist_from_goal)
        comp_dyn = ComputationalDynamics(robot_name, self.pin)
        ext_force = [0.0, 0.0, vertical_force]
        lever = [horizontal_location, 0.0, 0.0]
        ext_torque = np.cross(lever, ext_force)
        print('centroidal wrench', params.externalCentroidalWrench)
        if optimize_height_and_pitch:
            return self.test_trajectory_with_variable_pitch_and_height(params, self.pin, comp_dyn, des_height, mid_pitch, start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height)
        else:
            params.setDefaultValuesWrtWorld(self.pin)
            params.externalCentroidalWrench = np.hstack(
                [ext_force, ext_torque])
            return self.test_trajectory(params, comp_dyn, des_height, mid_pitch,
                                        start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height)
