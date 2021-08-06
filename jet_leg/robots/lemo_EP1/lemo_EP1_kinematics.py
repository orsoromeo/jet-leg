import pinocchio
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
from jet_leg.kinematics.kinematics_base import KinematicsBase

import os


class LemoEP0Kinematics(KinematicsBase):
    def __init__(self, pinocchio_model=False):
        super(LemoEP0Kinematics, self).__init__('lemo_EP1', pinocchio_model)

        self.urdf_foot_name_lf = 'FL_foot'
        self.urdf_foot_name_lh = 'HL_foot'
        self.urdf_foot_name_rf = 'FR_foot'
        self.urdf_foot_name_rh = 'HR_foot'
        self.urdf_knee_name_lf = 'FL_calf'
        self.urdf_knee_name_lh = 'HL_calf'
        self.urdf_knee_name_rf = 'FR_calf'
        self.urdf_knee_name_rh = 'HR_calf'
        self.urdf_hip_name_lf = 'FL_hip'
        self.urdf_hip_name_lh = 'HL_hip'
        self.urdf_hip_name_rf = 'FR_hip'
        self.urdf_hip_name_rh = 'HR_hip'

        self.q0 = np.vstack([0, 0.65, -1.45, 0, 0.65, -1.45,
                             0, 0.65, -1.45, 0.0, 0.65, -1.45])
