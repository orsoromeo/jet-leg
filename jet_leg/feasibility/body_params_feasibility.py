import numpy as np
from jet_leg.feasibility.find_feasible_trajectories import FeasibilityAnalysis
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics


class BodyParamsFeasibility(FeasibilityAnalysis):
    def test_body_vs_links_length(self, optimize_height_and_pitch, params, robot_name, step_height, body_length, links_length):
        print('current step height', step_height)
        print('current desired body length', body_length)
        print('current desired links length', links_length)

        self.setup_body_parameters(self.pin, body_length, links_length)

        FL_foot_frame_id = self.pin.model.getFrameId('FL_foot')
        FL_foot_frame = self.pin.model.frames[FL_foot_frame_id]
        links_length = -FL_foot_frame.placement.translation[2]
        print('links lenght', links_length)
        des_height = links_length*1.3
        print('des height', des_height)
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
