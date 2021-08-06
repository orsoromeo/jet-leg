import numpy as np
from jet_leg.feasibility.find_feasible_trajectories import FeasibilityAnalysis
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics


class HeightPitchFeasibility(FeasibilityAnalysis):
    def test_pitch_and_height(self, params, step_height, des_height, pitch):
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

        return self.test_trajectory(params, des_height, mid_pitch, start_point, mid_point, goal_point, dist_from_goal, step_distance, step_height)
