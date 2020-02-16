import numpy as np
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
import copy

class Jacobians:
    def __init__(self,robot_name):
        self.compDyn = ComputationalDynamics(robot_name)
        self.delta = 0.01

    def computeComPosJacobian(self, params):
        jacobian = np.zeros(3)
        initialCoMPos = copy.copy(params.comPositionWF)
        print "initialCoMPos ", initialCoMPos

        for j in np.arange(0, 3):
            params.comPositionWF = initialCoMPos
            params.comPositionWF[j] = initialCoMPos[j] + self.delta / 2.0
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params)
            params.comPositionWF = initialCoMPos
            params.comPositionWF[j] = initialCoMPos[j] - self.delta / 2.0
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params)
            diff = margin1 - margin2
            jacobian[j] = diff / self.delta
            print "[computeComPosJacobian]", margin1, margin2, self.delta, jacobian[j]

        return jacobian

    def computeComEulerAnglesJacobian(self, params):
        jacobian = np.zeros(3)
        initiaPoint = params.getOrientation()

        for j in np.arange(0, 3):
            eulerAngles = initiaPoint
            eulerAngles[j] = initiaPoint[j] + self.delta / 2.0
            params.setEulerAngles(eulerAngles)
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params)

            eulerAngles = initiaPoint
            eulerAngles[j] = initiaPoint[j] - self.delta / 2.0
            params.setEulerAngles(eulerAngles)
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params)
            diff = margin1 - margin2
            jacobian[j] = diff / self.delta

        return jacobian

    def computeComLinVelJacobian(self, params):
        jacobian = np.zeros(3)
        initiaPoint = params.comLinVel

        for j in np.arange(0, 3):
            params.comLinVel = initiaPoint
            params.comLinVel[j] = initiaPoint[j] + self.delta / 2.0
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params)

            params.comLinVel = initiaPoint
            params.comLinVel[j] = initiaPoint[j] - self.delta / 2.0
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params)
            diff = margin1 - margin2
            jacobian[j] = diff / self.delta

        return jacobian

    def computeComLinAccJacobian(self, params):
        jacobian = np.zeros(3)
        initiaPoint = params.comLinAcc

        for j in np.arange(0, 3):
            params.comLinAcc = initiaPoint
            params.comLinAcc[j] = initiaPoint[j] + self.delta / 2.0
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params)

            params.comLinAcc = initiaPoint
            params.comLinAcc[j] = initiaPoint[j] - self.delta / 2.0
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params)
            diff = margin1 - margin2
            jacobian[j] = diff / self.delta

        return jacobian

    def computeFeetJacobian(self, params):
        jacobian = np.zeros(12)
        initialContacts = copy.copy(params.getContactsPosWF())
        for footID in np.arange(0, 4):
            jacobian[footID*3: footID*3+3] = self.computeFootJacobian(params, footID)
        return jacobian

    def computeFootJacobian(self, params, footID):
        jacobian = np.zeros(3)
        initialContacts = copy.copy(params.getContactsPosWF())
        initiaPoint = copy.copy(initialContacts[footID])
        for j in np.arange(0, 3):
            contacts = initialContacts
            contacts[footID,j] = initiaPoint[j] + self.delta / 2.0
            params.setContactsPosWF(contacts)
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params)
            contacts = initialContacts
            contacts[footID, j] = initiaPoint[j] - self.delta / 2.0
            params.setContactsPosWF(contacts)
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params)
            diff = margin1 - margin2
            jacobian[j] = diff / self.delta

        return jacobian
