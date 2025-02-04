import os

import pinocchio
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *


class anymalKinematics():
    def __init__(self):
        self.PKG = os.path.dirname(os.path.abspath(__file__))
        self.URDF = self.PKG + '/urdf/anymal_boxy.urdf'
        if self.PKG is None:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF)
        else:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF, [self.PKG])
        self.robot = RobotWrapper.BuildFromURDF(self.URDF, [self.PKG])
        self.model = self.robot.model
        self.LF_foot_jac = None
        self.LH_foot_jac = None
        self.RF_foot_jac = None
        self.RH_foot_jac = None
        self.urdf_foot_name_lf = 'LF_FOOT'
        self.urdf_foot_name_lh = 'LH_FOOT'
        self.urdf_foot_name_rf = 'RF_FOOT'
        self.urdf_foot_name_rh = 'RH_FOOT'
        self.ik_success = False

    def getBlockIndex(self, frame_name):
        if frame_name == self.urdf_foot_name_lf:
            idx = 0
        elif frame_name == self.urdf_foot_name_lh:
            idx = 3
        elif frame_name == self.urdf_foot_name_rf:
            idx = 6
        elif frame_name == self.urdf_foot_name_rh:
            idx = 9

        return idx

    def footInverseKinematicsFixedBase(self, foot_pos_des, frame_name):
        frame_id = self.model.getFrameId(frame_name)
        blockIdx = self.getBlockIndex(frame_name)
        anymal_q0 = pinocchio.neutral(self.model)
        data = self.model.createData()
        q = anymal_q0
        eps = 0.005
        IT_MAX = 200
        DT = 1e-1
        err = np.zeros((3, 1))
        e = np.zeros((3, 1))

        i = 0
        while True:
            pinocchio.forwardKinematics(self.model, data, q)
            pinocchio.framesForwardKinematics(self.model, data, q)
            foot_pos = data.oMf[frame_id].translation
            e = foot_pos - foot_pos_des

            pinocchio.computeJointJacobians(self.model, data, q) # compute jacobians
            J = pinocchio.getFrameJacobian(self.model, data, frame_id, pinocchio.LOCAL_WORLD_ALIGNED)
            J_lin = J[:3, :]

            if np.linalg.norm(e) < eps:
                IKsuccess = True
                break
            if i >= IT_MAX:
                print(
                    "\n Warning: the iterative algorithm has not reached convergence to the desired precision. Error is: ",
                    np.linalg.norm(e))
                IKsuccess = False
                break
            v = - np.linalg.pinv(J_lin) @ e
            q = pinocchio.integrate(self.model, q, v * DT)
            i += 1

        q_leg = q[blockIdx:blockIdx + 3]
        J_leg = J_lin[:, blockIdx:blockIdx + 3]
        return q_leg, J_leg, err, IKsuccess

    def fixedBaseInverseKinematics(self, feetPosDes):

        self.LF_foot_jac = []
        self.LH_foot_jac = []
        self.RF_foot_jac = []
        self.RH_foot_jac = []
        leg_ik_success = np.zeros((4))

        f_p_des = np.array(feetPosDes[0, :]).T
        q_LF, self.LF_foot_jac, err, leg_ik_success[0] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_lf)

        f_p_des = np.array(feetPosDes[2, :]).T
        q_LH, self.LH_foot_jac, err, leg_ik_success[2] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_lh)

        f_p_des = np.array(feetPosDes[1, :]).T
        q_RF, self.RF_foot_jac, err, leg_ik_success[1] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_rf)

        f_p_des = np.array(feetPosDes[3, :]).T
        q_RH, self.RH_foot_jac, err, leg_ik_success[3] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_rh)
        print("success", leg_ik_success)
        self.ik_success = bool(leg_ik_success[0] and leg_ik_success[1] and leg_ik_success[2] and leg_ik_success[3])

        if self.ik_success is False:
            print('Warning, IK failed. Jacobian is singular')

        '''please NOTICE here the alphabetical order of the legs in the vector q: LF -> LH -> RF -> RH '''
        q = np.vstack([q_LF, q_LH, q_RF, q_RH])

        return q

    def getLegJacobians(self):
        isOutOfWS = not self.ik_success
        return self.LF_foot_jac, self.RF_foot_jac, self.LH_foot_jac, self.RH_foot_jac, isOutOfWS
