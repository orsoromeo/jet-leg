import pinocchio
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
import matplotlib.pyplot as plt
import os


class anymalKinematics():
    def __init__(self):
        self.PKG = os.path.dirname(os.path.abspath(__file__)) + '/../../resources/urdfs/anymal/'
        self.URDF = self.PKG + 'urdf/anymal_boxy.urdf'
        if self.PKG is None:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF)
        else:
            self.robot = RobotWrapper.BuildFromURDF(self.URDF, [self.PKG])
        self.model = self.robot.model
        self.data = self.robot.data
        self.LF_foot_jac = None
        self.LH_foot_jac = None
        self.RF_foot_jac = None
        self.RH_foot_jac = None

    def getBlockIndex(self, frame_name):
        if frame_name == 'LF_FOOT':
            idx = 0
        elif frame_name == 'LH_FOOT':
            idx = 3
        elif frame_name == 'RF_FOOT':
            idx = 6
        elif frame_name == 'RH_FOOT':
            idx = 9

        return idx

    def footInverseKinematicsFixedBase(self, foot_pos_des, frame_name):
        frame_id = self.model.getFrameId(frame_name)
        blockIdx = self.getBlockIndex(frame_name)
        anymal_q0 = np.vstack([-0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7,-1., 0.1,-0.7, 1.])
        q = anymal_q0
        eps = 0.005
        IT_MAX = 200
        DT = 1e-1
        err = np.zeros((3, 1))
        e = np.zeros((3, 1))

        i = 0
        while True:
            pinocchio.forwardKinematics(self.model, self.data, q)
            pinocchio.framesForwardKinematics(self.model, self.data, q)
            foot_pos = self.data.oMf[frame_id].translation
            #err = np.hstack([err, (foot_pos - foot_pos_des)])
            #e = err[:,-1]
            #print foot_pos_des[0], foot_pos[[0]], foot_pos[[0]] - foot_pos_des[0]
            #e = foot_pos - foot_pos_des
            e[0] = foot_pos[[0]] - foot_pos_des[0]
            e[1] = foot_pos[[1]] - foot_pos_des[1]
            e[2] = foot_pos[[2]] - foot_pos_des[2]
            if np.linalg.norm(e) < eps:
                #print("IK Convergence achieved!")
                break
            if i >= IT_MAX:
                print("\n Warning: the iterative algorithm has not reached convergence to the desired precision. Error is: ", np.linalg.norm(e))
                raise Exception('FailedConvergence')
            J = pinocchio.frameJacobian(self.model, self.data, q, frame_id)
            J_lin = J[:3, :]
            #print J_lin
            v = - np.linalg.pinv(J_lin) * e
            q = pinocchio.integrate(self.model, q, v * DT)
            i += 1
            #print i

        q_leg = q[blockIdx:blockIdx+3]
        J_leg = J_lin[:,blockIdx:blockIdx+3]
        return q_leg, J_leg, err

    def anymalFixedBaseInverseKinematics(self, feetPosDes):
        f_p_des = np.array(feetPosDes[0,:]).T
        q_LF, self.LF_foot_jac, err = self.footInverseKinematicsFixedBase(f_p_des, 'LF_FOOT')

        f_p_des = np.array(feetPosDes[2, :]).T
        q_LH, self.LH_foot_jac, err = self.footInverseKinematicsFixedBase(f_p_des, 'LH_FOOT')

        f_p_des = np.array(feetPosDes[1, :]).T
        q_RF, self.RF_foot_jac, err = self.footInverseKinematicsFixedBase(f_p_des, 'RF_FOOT')

        f_p_des = np.array(feetPosDes[3, :]).T
        q_RH, self.RH_foot_jac, err = self.footInverseKinematicsFixedBase(f_p_des, 'RH_FOOT')

        q = np.vstack([q_LF, q_LH, q_RF, q_RH])
        legJacs = np.vstack([self.LF_foot_jac, self.LH_foot_jac, self.RF_foot_jac, self.RH_foot_jac])
        return q, legJacs

    def getLegJacobians(self):
        isOutOfWS = False
        return self.LF_foot_jac, self.RF_foot_jac, self.LH_foot_jac, self.RH_foot_jac, isOutOfWS

