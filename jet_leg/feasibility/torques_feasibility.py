import numpy as np
from jet_leg.feasibility.find_feasible_trajectories import FeasibilityAnalysis
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics


class TorquesFeasibility(FeasibilityAnalysis):
    def test_hipy_vs_knee_torque(self, optimize_height_and_pitch, params, robot_name, step_height, hip_y_lim, knee_lim):
        print('current step height', step_height)

        hip_x_lim = self.pin.model.effortLimit[0]
        self.setup_torque_lims(hip_x_lim, hip_y_lim, knee_lim)

        print('effort limits', self.pin.model.effortLimit)
        des_height = 0.38
        start_point = [0.0, 0.0, des_height]
        mid_height = step_height/2.0 + des_height
        step_distance = 0.5
        mid_point = [step_distance, 0.0, mid_height]
        dist_from_goal = 2.0*step_distance
        goal_point = [dist_from_goal, 0.0, des_height + step_height]
        mid_pitch = -np.arcsin(step_height/dist_from_goal)
        if optimize_height_and_pitch:
            return self.test_trajectory_with_variable_pitch_and_height(params, self.pin, des_height, mid_pitch, start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height)
        else:
            params.setDefaultValuesWrtWorld(self.pin)
            return self.test_trajectory(params, des_height, mid_pitch,
                                        start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height)
