import os

import pinocchio
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *


class anymalKinematics():
    def __init__(self, anymal_type = 'anymal_boxy'):
        self.PKG = os.path.dirname(os.path.abspath(__file__)) + '/../../../resources/urdfs/anymal/'
        self.URDF = self.PKG + 'urdf/' + anymal_type + '.urdf'
        print self.URDF
        if self.PKG is None:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF)
        else:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF, [self.PKG])
        self.robot = RobotWrapper.BuildFromURDF(self.URDF, [self.PKG])
        self.model = self.robot.model
        self.data = self.robot.data
        self.LF_foot_jac = None
        self.LH_foot_jac = None
        self.RF_foot_jac = None
        self.RH_foot_jac = None
        self.urdf_foot_name_lf = 'LF_FOOT'
        self.urdf_foot_name_lh = 'LH_FOOT'
        self.urdf_foot_name_rf = 'RF_FOOT'
        self.urdf_foot_name_rh = 'RH_FOOT'
        self.ik_success = False
        self.leg_ik_success = [False, False, False, False]

    def getBlockIndex(self, frame_name):
        if frame_name == self.urdf_foot_name_lf:
            idx = 0
        elif frame_name == self.urdf_foot_name_lh:
            idx = 3
        elif frame_name == self.urdf_foot_name_rf:
            idx = 6
        elif frame_name == self.urdf_foot_name_rh:
            idx = 9

        # print ('index is',idx)
        return idx

    def footInverseKinematicsFixedBase(self, foot_pos_des, frame_name):
        frame_id = self.model.getFrameId(frame_name)
        blockIdx = self.getBlockIndex(frame_name)
        anymal_q0 = np.vstack([-0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -0.7, 1.])
        q = anymal_q0
        eps = 0.005
        IT_MAX = 200
        DT = 1e-1
        err = np.zeros((3, 1))
        e = np.zeros((3, 1))

        #print "foot pos des", foot_pos_des

        i = 0
        while True:
            pinocchio.forwardKinematics(self.model, self.data, q)
            pinocchio.framesForwardKinematics(self.model, self.data, q)
            foot_pos = self.data.oMf[frame_id].translation
            # err = np.hstack([err, (foot_pos - foot_pos_des)])
            # e = err[:,-1]
            # print foot_pos_des[0], foot_pos[[0]], foot_pos[[0]] - foot_pos_des[0]
            # e = foot_pos - foot_pos_des
            e[0] = foot_pos[[0]] - foot_pos_des[0]
            e[1] = foot_pos[[1]] - foot_pos_des[1]
            e[2] = foot_pos[[2]] - foot_pos_des[2]

            J = pinocchio.frameJacobian(self.model, self.data, q, frame_id)
            J_lin = J[:3, :]

            if np.linalg.norm(e) < eps:
                # print("IK Convergence achieved!")
                IKsuccess = True
                break
            if i >= IT_MAX:
                print(
                    "\n Warning: the iterative algorithm has not reached convergence to the desired precision. Error is: ",
                    np.linalg.norm(e))
                IKsuccess = False
                break
            # print J_lin
            v = - np.linalg.pinv(J_lin) * e
            q = pinocchio.integrate(self.model, q, v * DT)
            i += 1
            # print i

        q_leg = q[blockIdx:blockIdx + 3]
        J_leg = J_lin[:, blockIdx:blockIdx + 3]
        return q_leg, J_leg, err, IKsuccess

    def fixedBaseInverseKinematics(self, feetPosDes, stance_idx):

        self.LF_foot_jac = []
        self.LH_foot_jac = []
        self.RF_foot_jac = []
        self.RH_foot_jac = []
        self.leg_ik_success = [False, False, False, False]

        f_p_des = np.array(feetPosDes[0, :]).T
        q_LF, self.LF_foot_jac, err, self.leg_ik_success[0] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_lf)

        f_p_des = np.array(feetPosDes[1, :]).T
        q_RF, self.RF_foot_jac, err, self.leg_ik_success[1] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_rf)

        f_p_des = np.array(feetPosDes[2, :]).T
        q_LH, self.LH_foot_jac, err, self.leg_ik_success[2] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_lh)

        f_p_des = np.array(feetPosDes[3, :]).T
        q_RH, self.RH_foot_jac, err, self.leg_ik_success[3] = self.footInverseKinematicsFixedBase(f_p_des,
                                                                                             self.urdf_foot_name_rh)

        #self.ik_success = bool(leg_ik_success[0] and leg_ik_success[1] and leg_ik_success[2] and leg_ik_success[3])

        #print "stance index is ", stance_idx
        for legId in np.arange(0,4):
            #print "leg id ", legId, self.leg_ik_success[legId]
            if self.leg_ik_success[legId] is False:
                print('Warning, IK failed. Jacobian is singular. Leg id is', legId)

        number_of_stance_feet = np.shape(stance_idx)[0]
        are_stance_feet_in_workspace = np.full((1, number_of_stance_feet), False, dtype=bool)[0]
        previous_flag = True
        for k in np.arange(0, number_of_stance_feet):
            stance_leg_id = stance_idx[k]
            are_stance_feet_in_workspace[k] = self.leg_ik_success[int(stance_leg_id)]
            previous_flag = are_stance_feet_in_workspace[k] and previous_flag

        are_feet_in_workspace_flag = previous_flag
        #print "are_feet_in_ws_flag" , are_feet_in_workspace_flag
        self.ik_success = are_feet_in_workspace_flag
        #print "self.ik_success is ", self.ik_success

        '''please NOTICE here the alphabetical order of the legs in the vector q: LF -> LH -> RF -> RH '''
        q = np.vstack([q_LF, q_LH, q_RF, q_RH])

        return q, self.leg_ik_success

    def getLegJacobians(self):
        isOutOfWS = not self.ik_success
        return self.LF_foot_jac, self.RF_foot_jac, self.LH_foot_jac, self.RH_foot_jac, isOutOfWS, self.leg_ik_success
