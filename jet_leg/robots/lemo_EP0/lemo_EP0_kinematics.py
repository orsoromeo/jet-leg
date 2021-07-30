import pinocchio
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper

import os


class LemoEP0Kinematics():
    def __init__(self, pinocchio_model=False):
        self.check_joint_kinematic_lims = True
        if pinocchio_model is False:
            self.PKG = os.path.dirname(os.path.abspath(
                __file__)) + '/../../../resources/urdfs/lemo_EP0/'
            self.URDF = self.PKG + 'urdf/lemo_EP0.urdf'
            if self.PKG is None:
                self.robot = RobotWrapper.BuildFromURDF(self.URDF)
            else:
                self.robot = RobotWrapper.BuildFromURDF(self.URDF, [self.PKG])
            self.model = self.robot.model
            self.data = self.robot.data
        else:
            self.model = pinocchio_model.model
            self.data = pinocchio_model.data

        self.LF_foot_jac = None
        self.LH_foot_jac = None
        self.RF_foot_jac = None
        self.RH_foot_jac = None
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

    def getBlockIndex(self, foot_frame_name):
        if foot_frame_name == self.urdf_foot_name_lf:
            idx = 0
        elif foot_frame_name == self.urdf_foot_name_lh:
            idx = 6
        elif foot_frame_name == self.urdf_foot_name_rf:
            idx = 3
        elif foot_frame_name == self.urdf_foot_name_rh:
            idx = 9

        return idx

    def footInverseKinematicsFixedBaseUnsafe(self, foot_pos_des, foot_frame_name, knee_frame_name, hip_frame_name):
        foot_frame_id = self.model.getFrameId(foot_frame_name)
        knee_frame_id = self.model.getFrameId(knee_frame_name)
        hip_frame_id = self.model.getFrameId(hip_frame_name)
        blockIdx = self.getBlockIndex(foot_frame_name)
        q0 = np.vstack([0, 0.65, -1.45, 0, 0.65, -1.45,
                       0, 0.65, -1.45, 0.0, 0.65, -1.45])
        q = q0
        eps = 0.005
        IT_MAX = 200
        DT = 1e-1
        err = np.zeros((3, 1))
        e = np.zeros((3, 1))

        i = 0
        while True:
            pinocchio.forwardKinematics(self.model, self.data, q)
            pinocchio.framesForwardKinematics(self.model, self.data, q)
            foot_pos = self.data.oMf[foot_frame_id].translation
            knee_pos = self.data.oMf[knee_frame_id].translation
            hip_pos = self.data.oMf[hip_frame_id].translation

            e[0] = foot_pos[[0]] - foot_pos_des[0]
            e[1] = foot_pos[[1]] - foot_pos_des[1]
            e[2] = foot_pos[[2]] - foot_pos_des[2]
            if np.linalg.norm(e) < eps:
                break
            if i >= IT_MAX:
                print("Warning: the iterative algorithm has not reached convergence to the desired precision. Error is: ", np.linalg.norm(e))
                raise Exception('FailedConvergence at blockIdx', blockIdx)
            J = pinocchio.computeFrameJacobian(
                self.model, self.data, q, foot_frame_id)
            J_lin = J[:3, :]
            v = - np.dot(np.linalg.pinv(J_lin), e)
            q = pinocchio.integrate(self.model, q, v * DT)
            i += 1

        q_leg = np.transpose(q[blockIdx:blockIdx+3])
        J_leg = J_lin[:, blockIdx:blockIdx+3]
        return q_leg, J_leg, err, knee_pos, hip_pos

    def footInverseKinematicsFixedBase(self, leg_id, foot_pos_des, foot_frame_name, knee_frame_name, hip_frame_name):
        q, foot_jac, err, knee_pos, hip_pos = self.footInverseKinematicsFixedBaseUnsafe(
            foot_pos_des, foot_frame_name, knee_frame_name, hip_frame_name)
        ik_success = True
        print('check kin lims', self.check_joint_kinematic_lims)
        if(self.check_joint_kinematic_lims):
            for j in range(0, 3):
                if(q[j] >= self.model.upperPositionLimit[leg_id*3+j]):
                    ik_success = False
                    raise Exception('Maximum kinematic limit violated at leg', leg_id, 'joint', j, ': q is', float(
                        q[j]), 'and q max is', float(self.model.upperPositionLimit[leg_id*3+j]))
                elif(q[j] <= self.model.lowerPositionLimit[leg_id*3+j]):
                    ik_success = False
                    print('Min kinematic limit violated at leg', leg_id, 'joint', j, ': q is', float(q[
                        j]), 'and q min is', float(self.model.lowerPositionLimit[leg_id * 3 + j]))
                    raise
        return q, foot_jac, err, knee_pos, hip_pos, ik_success

    def fixedBaseInverseKinematics(self, feetPosDes, stance_idx):

        self.LF_foot_jac = []
        self.LH_foot_jac = []
        self.RF_foot_jac = []
        self.RH_foot_jac = []
        q_LF = [0]*3
        q_RF = [0]*3
        q_LH = [0]*3
        q_RH = [0]*3
        knee_pos_LF = [0]*3
        knee_pos_RF = [0]*3
        knee_pos_LH = [0]*3
        knee_pos_RH = [0]*3
        hip_pos_LF = [0]*3
        hip_pos_RF = [0]*3
        hip_pos_LH = [0]*3
        hip_pos_RH = [0]*3
        number_of_stance_feet = np.shape(stance_idx)[0]
        leg_ik_success = [True] * 4
        knees_pos = np.zeros((4, 3))
        for k in np.arange(0, number_of_stance_feet):
            print('k', k)
            stance_leg_id = int(stance_idx[k])
            print('id', stance_leg_id)
            if stance_leg_id == 0:
                f_p_des = np.array(feetPosDes[0, :]).T
                q_LF, self.LF_foot_jac, err, knee_pos_LF, hip_pos_LF, leg_ik_success[0] = self.footInverseKinematicsFixedBase(0, f_p_des,
                                                                                                                              self.urdf_foot_name_lf, self.urdf_knee_name_lf, self.urdf_hip_name_lf)
                knees_pos[stance_leg_id, :] = knee_pos_LF
            if stance_leg_id == 1:
                f_p_des = np.array(feetPosDes[1, :]).T
                q_RF, self.RF_foot_jac, err, knee_pos_RF, hip_pos_RF, leg_ik_success[1] = self.footInverseKinematicsFixedBase(1, f_p_des,
                                                                                                                              self.urdf_foot_name_rf, self.urdf_knee_name_rf, self.urdf_hip_name_rf)
                knees_pos[stance_leg_id, :] = knee_pos_RF
            if stance_leg_id == 2:
                f_p_des = np.array(feetPosDes[2, :]).T
                q_LH, self.LH_foot_jac, err, knee_pos_LH, hip_pos_LH, leg_ik_success[2] = self.footInverseKinematicsFixedBase(2, f_p_des,
                                                                                                                              self.urdf_foot_name_lh, self.urdf_knee_name_lh, self.urdf_hip_name_lh)
                knees_pos[stance_leg_id, :] = knee_pos_LH
            if stance_leg_id == 3:
                f_p_des = np.array(feetPosDes[3, :]).T
                q_RH, self.RH_foot_jac, err, knee_pos_RH, hip_pos_RH, leg_ik_success[3] = self.footInverseKinematicsFixedBase(3, f_p_des,
                                                                                                                              self.urdf_foot_name_rh, self.urdf_knee_name_rh, self.urdf_hip_name_rh)
                knees_pos[stance_leg_id, :] = knee_pos_RH
        for legId in np.arange(0, 4):
            if leg_ik_success[legId] is False:
                print('--- > Warning, IK failed. Jacobian is singular. Leg id is', legId)
                raise Exception('raise exception')
        '''please NOTICE here the alphabetical order of the legs in the vector q: LF -> LH -> RF -> RH '''
        q = np.hstack([q_LF, q_LH, q_RF, q_RH])
        hip_pos = np.vstack([hip_pos_LF, hip_pos_RF, hip_pos_LH, hip_pos_RH])

        print('q', q)
        print('hips', hip_pos)
        print('knees', knees_pos)
        # legJacs = np.vstack([self.LF_foot_jac, self.LH_foot_jac, self.RF_foot_jac, self.RH_foot_jac])
        return q, knees_pos, hip_pos

    def getLegJacobians(self):
        isOutOfWS = False
        return self.LF_foot_jac, self.RF_foot_jac, self.LH_foot_jac, self.RH_foot_jac, isOutOfWS
