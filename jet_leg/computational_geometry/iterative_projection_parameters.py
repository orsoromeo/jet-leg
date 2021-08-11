# -*- coding: utf-8 -*-
"""
Created on Wed Nov 14 15:07:45 2018

@author: Romeo Orsolino
"""
import os
import numpy as np
from jet_leg.computational_geometry.math_tools import Math
import random
from copy import copy
from scipy.spatial.transform import Rotation as Rot
from jet_leg.robots.robot_model_interface import RobotModelInterface
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
import pinocchio
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper


class IterativeProjectionParameters:
    def __init__(self, robot_name):

        PKG = os.path.dirname(os.path.abspath(__file__)) + \
            '/../../resources/urdfs/'+robot_name+'/'
        URDF = PKG + robot_name + '.urdf'
        if PKG is None:
            self.pin = RobotWrapper.BuildFromURDF(URDF)
        else:
            self.pin = RobotWrapper.BuildFromURDF(URDF, [PKG])

        self.robotName = robot_name
        self.robotModel = RobotModelInterface(self.robotName)
        self.compDyn = ComputationalDynamics(self.robotName, self.pin)
        self.math = Math()
        self.q = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        # var used only for IK inside constraints.py
        self.comPositionBF = [0., 0., 0.]
        self.comPositionWF = [0., 0., 0.]
        self.comLinVel = [0., 0., 0.]
        self.comLinAcc = [0., 0., 0.]
        self.comAngAcc = [0., 0., 0.]
        self.footPosWLF = [0.3, 0.2, -.0]
        self.footPosWRF = [0.3, -0.2, -.0]
        self.footPosWLH = [-0.3, 0.2, -.0]
        self.footPosWRH = [-0.3, -0.2, -.0]
        self.externalForce = [0., 0., 0.]
        self.externalCentroidalTorque = [0., 0., 0.]
        self.externalCentroidalWrench = np.hstack(
            [self.externalForce, self.externalCentroidalTorque])
        self.instantaneousCapturePoint = [0.0, 0.0]

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.eurlerAngles = [self.roll, self.pitch, self.yaw]

        self.LF_tau_lim = [50.0, 50.0, 50.0]
        self.RF_tau_lim = [50.0, 50.0, 50.0]
        self.LH_tau_lim = [50.0, 50.0, 50.0]
        self.RH_tau_lim = [50.0, 50.0, 50.0]
        self.torque_limits = np.array(
            [self.LF_tau_lim, self.RF_tau_lim, self.LH_tau_lim, self.RH_tau_lim])

        self.state_machineLF = True
        self.state_machineRF = True
        self.state_machineLH = True
        self.state_machineRH = True
        self.stanceFeet = [0, 0, 0, 0]
        self.numberOfContacts = 0
#        self.contactsHF = np.zeros((4,3))
        self.contactsBF = np.zeros((4, 3))
        self.contactsWF = np.zeros((4, 3))

        axisZ = np.array([[0.0], [0.0], [1.0]])
        n1 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))
        # %% Cell 2
        self.normals = np.vstack([n1, n2, n3, n4])
        self.constraintMode = ['FRICTION_AND_ACTUATION',
                               'FRICTION_AND_ACTUATION',
                               'FRICTION_AND_ACTUATION',
                               'FRICTION_AND_ACTUATION']

        self.friction = 0.8
        self.robotMass = self.robotModel.trunkMass  # Kg
        self.numberOfGenerators = 4
        self.useContactTorque = True
        self.useInstantaneousCapturePoint = True
        self.actual_swing = 0

    def computeContactsPosBF(self):
        self.contactsBF = np.zeros((4, 3))
        rpy = self.getOrientation()
        rot = Rot.from_euler('xyz', [rpy[0], rpy[1], rpy[2]], degrees=False)
        B_R_W = rot.inv().as_matrix()
        for j in np.arange(0, 4):
            j = int(j)
            self.contactsBF[j, :] = np.add(
                np.dot(B_R_W, (self.contactsWF[j, :] - self.comPositionWF)), self.comPositionBF)
        return self.contactsBF

    def setContactsPosBF(self, contactsBF):
        self.contactsBF = contactsBF

    def setContactsPosWF(self, contactsWF):
        self.contactsWF = contactsWF

    def setCoMPosWF(self, comWF):
        self.comPositionWF = comWF

    def setCoMLinAcc(self, comLinAcc):
        self.comLinAcc = comLinAcc

    def setCoMAngAcc(self, comAngAcc):
        self.comAngAcc = comAngAcc

    def setTorqueLims(self, torqueLims):
        self.torque_limits = torqueLims

    def setActiveContacts(self, activeContacts):
        self.stanceFeet = activeContacts

    def setContactNormals(self, normals):
        self.normals = normals

    def setConstraintModes(self, constraintMode):
        self.constraintMode = constraintMode

    def setFrictionCoefficient(self, mu):
        self.friction = mu

    def setNumberOfFrictionConesEdges(self, ng):
        self.numberOfGenerators = ng

    def setTotalMass(self, mass):
        self.robotMass = mass

    def setEulerAngles(self, eurlerAngles):
        self.roll = eurlerAngles[0]
        self.pitch = eurlerAngles[1]
        self.yaw = eurlerAngles[2]
        self.eurlerAngles = [self.roll, self.pitch, self.yaw]

    def setInstantaneousCapturePoint(self, ICP):
        self.instantaneousCapturePoint = ICP

    def getInstantaneousCapturePoint(self):
        return self.instantaneousCapturePoint

    def getContactsPosWF(self):
        return self.contactsWF

    def getContactsPosBF(self):  # used only for IK inside constraints.py
        return self.contactsBF

    def getCoMPosWF(self):
        return self.comPositionWF

    def getCoMPosBF(self):
        return self.comPositionBF

    def getCoMLinAcc(self):
        return self.comLinAcc

    def getCoMAngAcc(self):
        return self.comAngAcc

    def getTorqueLims(self):
        return self.torque_limits

    def getStanceFeet(self):
        return self.stanceFeet

    def getNormals(self):
        return self.normals

    def getOrientation(self):
        return self.eurlerAngles

    def getConstraintModes(self):
        return self.constraintMode

    def getFrictionCoefficient(self):
        return self.friction

    def getNumberOfFrictionConesEdges(self):
        return self.numberOfGenerators

    def getTotalMass(self):
        return self.robotMass

    def getStanceIndex(self, stanceLegs):
        stanceIdx = []
        for iter in range(0, 4):
            if stanceLegs[iter] == 1:
                stanceIdx = np.hstack([stanceIdx, iter])
        return stanceIdx

    def getCurrentFeetPos(self, received_data):
        num_of_elements = np.size(received_data.data)
        for j in range(0, num_of_elements):
            if str(received_data.name[j]) == str("footPosDesLFx"):
                self.footPosWLF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesLFy"):
                self.footPosWLF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesLFz"):
                self.footPosWLF[2] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRFx"):
                self.footPosWRF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRFy"):
                self.footPosWRF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRFz"):
                self.footPosWRF[2] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesLHx"):
                self.footPosWLH[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesLHy"):
                self.footPosWLH[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesLHz"):
                self.footPosWLH[2] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRHx"):
                self.footPosWRH[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRHy"):
                self.footPosWRH[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRHz"):
                self.footPosWRH[2] = received_data.data[j]

        self.contactsWF = np.array(
            [self.footPosWLF, self.footPosWRF, self.footPosWLH, self.footPosWRH])

    def getParamsFromRosDebugTopic(self, received_data):

        num_of_elements = np.size(received_data.data)
        for j in range(0, num_of_elements):
            if str(received_data.name[j]) == str("LF_HAAmaxVar"):
                self.LF_tau_lim[0] = received_data.data[j]
            if str(received_data.name[j]) == str("LF_HFEmaxVar"):
                self.LF_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("LF_KFEmaxVar"):
                self.LF_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_HAAmaxVar"):
                self.RF_tau_lim[0] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_HFEmaxVar"):
                self.RF_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_KFEmaxVar"):
                self.RF_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_HAAmaxVar"):
                self.LH_tau_lim[0] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_HFEmaxVar"):
                self.LH_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_KFEmaxVar"):
                self.LH_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_HAAmaxVar"):
                self.RH_tau_lim[0] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_HFEmaxVar"):
                self.RH_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_KFEmaxVar"):
                self.RH_tau_lim[2] = received_data.data[j]

            self.torque_limits = np.array([self.LF_tau_lim,
                                           self.RF_tau_lim,
                                           self.LH_tau_lim,
                                           self.RH_tau_lim, ])

# the inputs are all in the WF this way we can compute generic regions for generic contact sets and generic com position

            if str(received_data.name[j]) == str("contact_setWLFx"):
                self.footPosWLF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWLFy"):
                self.footPosWLF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWLFz"):
                self.footPosWLF[2] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRFx"):
                self.footPosWRF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRFy"):
                self.footPosWRF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRFz"):
                self.footPosWRF[2] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWLHx"):
                self.footPosWLH[0] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWLHy"):
                self.footPosWLH[1] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWLHz"):
                self.footPosWLH[2] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRHx"):
                self.footPosWRH[0] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRHy"):
                self.footPosWRH[1] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRHz"):
                self.footPosWRH[2] = received_data.data[j]

            self.contactsWF = np.array(
                [self.footPosWLF, self.footPosWRF, self.footPosWLH, self.footPosWRH])

            if str(received_data.name[j]) == str("actual_CoMX"):
                self.comPositionWF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("actual_CoMY"):
                self.comPositionWF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("actual_CoMZ"):
                self.comPositionWF[2] = received_data.data[j]

            if str(received_data.name[j]) == str("offCoMX"):
                self.comPositionBF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("offCoMY"):
                self.comPositionBF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("offCoMZ"):
                self.comPositionBF[2] = received_data.data[j]

            # external wrench
            if str(received_data.name[j]) == str("extPerturbForceX"):
                self.externalForceWF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("extPerturbForceY"):
                self.externalForceWF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("extPerturbForceZ"):
                self.externalForceWF[2] = received_data.data[j]

            if str(received_data.name[j]) == str("LF_HAA_th"):
                self.q[0] = received_data.data[j]
            if str(received_data.name[j]) == str("LF_HFE_th"):
                self.q[1] = received_data.data[j]
            if str(received_data.name[j]) == str("LF_KFE_th"):
                self.q[2] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_HAA_th"):
                self.q[3] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_HFE_th"):
                self.q[4] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_KFE_th"):
                self.q[5] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_HAA_th"):
                self.q[6] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_HFE_th"):
                self.q[7] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_KFE_th"):
                self.q[8] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_HAA_th"):
                self.q[9] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_HFE_th"):
                self.q[10] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_KFE_th"):
                self.q[11] = received_data.data[j]

            # they are in WF
            if str(received_data.name[j]) == str("normalLFx"):
                self.normals[0, 0] = received_data.data[j]
            if str(received_data.name[j]) == str("normalLFy"):
                self.normals[0, 1] = received_data.data[j]
            if str(received_data.name[j]) == str("normalLFz"):
                self.normals[0, 2] = received_data.data[j]

            if str(received_data.name[j]) == str("normalRFx"):
                self.normals[1, 0] = received_data.data[j]
            if str(received_data.name[j]) == str("normalRFy"):
                self.normals[1, 1] = received_data.data[j]
            if str(received_data.name[j]) == str("normalRFz"):
                self.normals[1, 2] = received_data.data[j]

            if str(received_data.name[j]) == str("normalLHx"):
                self.normals[2, 0] = received_data.data[j]
            if str(received_data.name[j]) == str("normalLHy"):
                self.normals[2, 1] = received_data.data[j]
            if str(received_data.name[j]) == str("normalLHz"):
                self.normals[2, 2] = received_data.data[j]

            if str(received_data.name[j]) == str("normalRHx"):
                self.normals[3, 0] = received_data.data[j]
            if str(received_data.name[j]) == str("normalRHy"):
                self.normals[3, 1] = received_data.data[j]
            if str(received_data.name[j]) == str("normalRHz"):
                self.normals[3, 2] = received_data.data[j]

            if str(received_data.name[j]) == str("robotMass"):
                self.robotMass = received_data.data[j]

            if str(received_data.name[j]) == str("muEstimate"):
                self.friction = received_data.data[j]

            if str(received_data.name[j]) == str("roll"):
                self.roll = received_data.data[j]
            if str(received_data.name[j]) == str("pitch"):
                self.pitch = received_data.data[j]
            if str(received_data.name[j]) == str("yaw"):
                self.yaw = received_data.data[j]

            if str(received_data.name[j]) == str("actual_swing"):
                self.actual_swing = int(received_data.data[j])

        self.robotMass -= self.externalForceWF[2]/9.81

    def getFutureStanceFeetFlags(self, received_data):

        num_of_elements = np.size(received_data.data)
        for j in range(0, num_of_elements):
            if str(received_data.name[j]) == str("future_stance_LF"):
                self.stanceFeet[0] = int(received_data.data[j])
            if str(received_data.name[j]) == str("future_stance_RF"):
                self.stanceFeet[1] = int(received_data.data[j])
            if str(received_data.name[j]) == str("future_stance_LH"):
                self.stanceFeet[2] = int(received_data.data[j])
            if str(received_data.name[j]) == str("future_stance_RH"):
                self.stanceFeet[3] = int(received_data.data[j])
        self.numberOfContacts = np.sum(self.stanceFeet)

    def getCurrentStanceFeetFlags(self, received_data):

        num_of_elements = np.size(received_data.data)
        for j in range(0, num_of_elements):
            if str(received_data.name[j]) == str("state_machineLF"):
                self.state_machineLF = int(received_data.data[j])
            if str(received_data.name[j]) == str("state_machineRF"):
                self.state_machineRF = int(received_data.data[j])
            if str(received_data.name[j]) == str("state_machineLH"):
                self.state_machineLH = int(received_data.data[j])
            if str(received_data.name[j]) == str("state_machineRH"):
                self.state_machineRH = int(received_data.data[j])

            if self.state_machineLF < 4.0:
                self.stanceFeet[0] = 1
            else:
                self.stanceFeet[0] = 0

            if self.state_machineRF < 4.0:
                self.stanceFeet[1] = 1
            else:
                self.stanceFeet[1] = 0

            if self.state_machineLH < 4.0:
                self.stanceFeet[2] = 1
            else:
                self.stanceFeet[2] = 0

            if self.state_machineRH < 4.0:
                self.stanceFeet[3] = 1
            else:
                self.stanceFeet[3] = 0

        self.numberOfContacts = np.sum(self.stanceFeet)

    def setDefaultValuesWrtBase(self):
        '''
        possible constraints for each foot:
         ONLY_ACTUATION = only joint-torque limits are enforces
         ONLY_FRICTION = only friction cone constraints are enforced
         FRICTION_AND_ACTUATION = both friction cone constraints and joint-torque limits
        '''
        constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION']

        comWF = np.array([0.0, 0.0, 0.0])
        comWF_lin_acc = np.array([0.0, .0, .0])
        comWF_ang_acc = np.array([.0, .0, .0])

        ''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
        extForceW = np.array([0., .0, 0.0 * 9.81])  # units are N
        extCentroidalTorque = np.array([.0, .0, .0])  # units are Nm
        extCentroidalWrench = np.hstack([extForceW, extCentroidalTorque])

        ''' parameters to be tuned'''
        mu = 0.5

        ''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
        stanceFeet = [1, 1, 1, 1]

        randomSwingLeg = random.randint(0, 3)
        tripleStance = False  # if you want you can define a swing leg using this variable
        if tripleStance:
            stanceFeet[randomSwingLeg] = 0
        ''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
        axisZ = np.array([[0.0], [0.0], [1.0]])

        n1 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LF
        n2 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RF
        n3 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LH
        n4 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RH
        normals = np.vstack([n1, n2, n3, n4])

        ''' Roll Pitch Yaw angles of the base link'''
        rpy_base = np.array([0., 0., 0.0])  # units are rads
        rot = Rot.from_euler(
            'xyz', [rpy_base[0], rpy_base[1], rpy_base[2]], degrees=False)
        W_R_B = rot.as_matrix()

        '''You now need to fill the 'params' object with all the relevant
            informations needed for the computation of the IP'''

        """ contact points in the World Frame"""
        model = RobotModelInterface(self.robotName)
        LF_foot = model.nominal_stance_LF
        RF_foot = model.nominal_stance_RF
        LH_foot = model.nominal_stance_LH
        RH_foot = model.nominal_stance_RH

        LF_foot = [0.3, 0.2, -0.59]
        RF_foot = [0.4, -0.2, -0.19]
        LH_foot = [-0.3, 0.2, -0.59]
        RH_foot = [-0.3, -0.2, -0.59]

        contactsBF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
        contactsWF = copy(contactsBF)
        self.setContactsPosWF(contactsWF)
        self.setEulerAngles(rpy_base)
        self.useContactTorque = True
        self.useInstantaneousCapturePoint = True
        self.externalCentroidalWrench = extCentroidalWrench
        self.setCoMPosWF(comWF)
        self.comLinVel = [0., 0.0, 0.0]
        self.setCoMLinAcc(comWF_lin_acc)
        self.setTorqueLims(
            self.compDyn.robotModel.robotModel.joint_torque_limits)
        self.setActiveContacts(stanceFeet)
        self.setConstraintModes(constraint_mode_IP)
        self.setContactNormals(normals)
        self.setFrictionCoefficient(mu)
        self.setTotalMass(self.compDyn.robotModel.robotModel.trunkMass)

        # params.externalForceWF is actually used anywhere at the moment
        self.externalForceWF = extForceW

    def setDefaultValuesWrtWorld(self, pinocchio=False):
        '''
        possible constraints for each foot:
         ONLY_ACTUATION = only joint-torque limits are enforces
         ONLY_FRICTION = only friction cone constraints are enforced
         FRICTION_AND_ACTUATION = both friction cone constraints and joint-torque limits
        '''
        constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION']

        comWF = np.array([0.0, 0.0, 0.5])
        # comWF = np.array([0.58333333, 0.03066667, 0.45666667])

        comWF_lin_acc = np.array([0.0, .0, .0])
        comWF_ang_acc = np.array([.0, .0, .0])

        ''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
        extForceW = np.array([0., .0, 0.0 * 9.81])  # units are N
        extCentroidalTorque = np.array([.0, .0, .0])  # units are Nm
        extCentroidalWrench = np.hstack([extForceW, extCentroidalTorque])

        ''' parameters to be tuned'''
        mu = 0.5

        ''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
        stanceFeet = [1, 1, 1, 1]

        randomSwingLeg = random.randint(0, 3)
        tripleStance = False  # if you want you can define a swing leg using this variable
        if tripleStance:
            stanceFeet[randomSwingLeg] = 0

        ''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
        axisZ = np.array([[0.0], [0.0], [1.0]])

        n1 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LF
        n2 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RF
        n3 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LH
        n4 = np.transpose(np.transpose(
            self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RH
        normals = np.vstack([n1, n2, n3, n4])

        ''' Roll Pitch Yaw angles of the base link'''
        rpy_base = np.array([0., 0.0, 0.0])  # units are rads
        # rpy_base = np.array([0., -0.11745879, 0.])  # units are rads
        rot = Rot.from_euler(
            'xyz', [rpy_base[0], rpy_base[1], rpy_base[2]], degrees=False)
        W_R_B = rot.as_matrix()

        '''You now need to fill the 'params' object with all the relevant 
            informations needed for the computation of the IP'''

        """ contact points in the World Frame"""
        if pinocchio:
            LF_foot = pinocchio.model.jointPlacements[1].translation
            RF_foot = pinocchio.model.jointPlacements[4].translation
            LH_foot = pinocchio.model.jointPlacements[7].translation
            RH_foot = pinocchio.model.jointPlacements[10].translation
        else:
            model = RobotModelInterface(self.robotName)
            LF_foot = model.nominal_stance_LF
            RF_foot = model.nominal_stance_RF
            LH_foot = model.nominal_stance_LH
            RH_foot = model.nominal_stance_RH

        LF_foot[2] = 0.0
        RF_foot[2] = 0.0
        LH_foot[2] = 0.0
        RH_foot[2] = 0.0

        # LF_foot = [0.3, 0.2, 0.0]
        # RF_foot = [0.3, -0.2, 0.0]
        # LH_foot = [-0.3, 0.2, 0.0]
        # RH_foot = [-0.3, -0.2, 0.0]

        # LF_foot = [1.13,  0.092, 0.2]
        # RF_foot = [0.97, -0.092, 0.2]
        # LH_foot = [0.31,  0.092, 0.]
        # RH_foot = [0.31, -0.092, 0.]
        contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))

        self.setContactsPosWF(contactsWF)
        self.setEulerAngles(rpy_base)
        self.useContactTorque = True
        self.useInstantaneousCapturePoint = True
        self.externalCentroidalWrench = extCentroidalWrench
        self.setCoMPosWF(comWF)
        self.comLinVel = [0., 0.0, 0.0]
        self.setCoMLinAcc(comWF_lin_acc)
        self.setTorqueLims(
            self.compDyn.robotModel.robotModel.joint_torque_limits)
        self.setActiveContacts(stanceFeet)
        self.setConstraintModes(constraint_mode_IP)
        self.setContactNormals(normals)
        self.setFrictionCoefficient(mu)
        self.setTotalMass(self.compDyn.robotModel.robotModel.trunkMass)
        self.externalForceWF = extForceW
