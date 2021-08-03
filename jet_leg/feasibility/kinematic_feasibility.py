import numpy as np
from jet_leg.feasibility.find_feasible_trajectories import FeasibilityAnalysis
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics


class KinematicFeasibility(FeasibilityAnalysis):
    def test_hipy_vs_knee_position_limits(self, optimize_height_and_pitch, params, robot_name, step_height, hip_y_min, knee_min):
        print('current step height', step_height)

        hip_x_min = self.pin.model.lowerPositionLimit[0]
        hip_x_max = self.pin.model.upperPositionLimit[0]

        hip_y_range = self.pin.model.upperPositionLimit[1] - \
            self.pin.model.lowerPositionLimit[1]
        hip_y_max = hip_y_min + hip_y_range
        knee_range = self.pin.model.upperPositionLimit[2] - \
            self.pin.model.lowerPositionLimit[2]
        knee_max = knee_min + knee_range

        print('Lower position limits', self.pin.model.lowerPositionLimit)
        print('Upper position limits', self.pin.model.upperPositionLimit)

        for leg in range(0, 4):
            self.pin.model.lowerPositionLimit[leg*3] = hip_x_min
            self.pin.model.lowerPositionLimit[1+leg*3] = hip_y_min
            self.pin.model.lowerPositionLimit[2+leg*3] = knee_min

            self.pin.model.upperPositionLimit[leg*3] = hip_x_max
            self.pin.model.upperPositionLimit[1+leg*3] = hip_y_max
            self.pin.model.upperPositionLimit[2+leg*3] = knee_max

        print('Lower position limits', self.pin.model.lowerPositionLimit)
        print('Upper position limits', self.pin.model.upperPositionLimit)

        des_height = 0.38
        start_point = [0.0, 0.0, des_height]
        mid_height = step_height/2.0 + des_height
        step_distance = 0.5
        mid_point = [step_distance, 0.0, mid_height]
        dist_from_goal = 2.0*step_distance
        goal_point = [dist_from_goal, 0.0, des_height + step_height]
        mid_pitch = -np.arcsin(step_height/dist_from_goal)
        comp_dyn = ComputationalDynamics(robot_name, self.pin)
        if optimize_height_and_pitch:
            return self.test_trajectory_with_variable_pitch_and_height(params, self.pin, comp_dyn, des_height, mid_pitch, start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height)
        else:
            params.setDefaultValuesWrtWorld(self.pin)
            return self.test_trajectory(params, comp_dyn, des_height, mid_pitch,
                                        start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height)
