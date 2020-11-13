from jet_leg.computational_geometry.iterative_projection_parameters import IterativeProjectionParameters
import numpy as np
from scipy.spatial.transform import Rotation as Rot

class RobotStateCartesianInterface(IterativeProjectionParameters):
    pass

    def getStateFromRobotStateCartesian(self, received_data):
        num_of_elements = np.size(received_data)
        # print 'number of elements: ', num_of_elements
        #print received_data
        self.setCoMPosWF([received_data.base.pose.position.x,
                          received_data.base.pose.position.y,
                          received_data.base.pose.position.z])
        contactsLF = [received_data.ee_motion[0].pos.x,
                      received_data.ee_motion[0].pos.y,
                      received_data.ee_motion[0].pos.z]
        contactsRF = [received_data.ee_motion[1].pos.x,
                      received_data.ee_motion[1].pos.y,
                      received_data.ee_motion[1].pos.z]
        contactsLH = [received_data.ee_motion[2].pos.x,
                      received_data.ee_motion[2].pos.y,
                      received_data.ee_motion[2].pos.z]
        contactsRH = [received_data.ee_motion[3].pos.x,
                      received_data.ee_motion[3].pos.y,
                      received_data.ee_motion[3].pos.z]
        contactsWF = np.vstack([contactsLF, contactsRF, contactsLH, contactsRH])
        self.setContactsPosWF(contactsWF)
        print contactsWF
        stanceFlags = [received_data.ee_contact[0],
                       received_data.ee_contact[1],
                       received_data.ee_contact[2],
                       received_data.ee_contact[3]]
        self.setActiveContacts(stanceFlags)
        quaternions = [received_data.base.pose.orientation.w,
                received_data.base.pose.orientation.x,
                received_data.base.pose.orientation.y,
                received_data.base.pose.orientation.z]
        rotation = Rot.from_quat(quaternions)
        print "received rotation base to world",rotation.as_dcm()
        print "received rotation world to base",np.transpose(rotation.as_dcm())
        r,p,y = rotation.as_euler('zyx', degrees=False)
        #r,p,y = rotation.as_euler('xyz', degrees=False)
        euler = [-r, p, np.pi - y]
        print "euler", euler
        #euler = [r, p, y]
        self.setEulerAngles(euler)
        self.useContactTorque = True
        #params.externalCentroidalWrench = extCentroidalWrench
        comLinVelWrtWF = np.array([received_data.base.twist.linear.x,
           received_data.base.twist.linear.y,
           received_data.base.twist.linear.z])

        w_R_b = self.math.rpyToRot(0.0,0.0,0.0);
        comLinVelWrtBF = w_R_b.dot(comLinVelWrtWF)
        comAngAccBF = [0.0, 0.0, 0.0]
        self.setCoMAngAcc(comAngAccBF)

        self.comLinVel = comLinVelWrtBF
        acc = [received_data.base.accel.linear.x,
                           received_data.base.accel.linear.y,
                           received_data.base.accel.linear.z]
        self.setCoMLinAcc(acc)

        #self.setTorqueLims(self..robotModel.robotModel.joint_torque_limits)

        constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION']

        self.setConstraintModes(constraint_mode_IP)
        self.externalCentroidalWrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        axisZ = np.array([[0.0], [0.0], [1.0]])
        n1 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LF
        n2 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RF
        n3 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LH
        n4 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RH
        normals = np.vstack([n1, n2, n3, n4])
        self.setContactNormals(normals)
        self.setFrictionCoefficient(0.8)


    def getStateFromRobotStateCartesianWrtBaseFrame(self, received_data):
        num_of_elements = np.size(received_data)
        # print 'number of elements: ', num_of_elements
        #print received_data
        self.setCoMPosWF([0.0, 0.0, 0.0])
        contactsLF = [received_data.ee_motion[0].pos.x,
                      received_data.ee_motion[0].pos.y,
                      received_data.ee_motion[0].pos.z]
        contactsRF = [received_data.ee_motion[1].pos.x,
                      received_data.ee_motion[1].pos.y,
                      received_data.ee_motion[1].pos.z]
        contactsLH = [received_data.ee_motion[2].pos.x,
                      received_data.ee_motion[2].pos.y,
                      received_data.ee_motion[2].pos.z]
        contactsRH = [received_data.ee_motion[3].pos.x,
                      received_data.ee_motion[3].pos.y,
                      received_data.ee_motion[3].pos.z]
        contactsBF = np.vstack([contactsLF, contactsRF, contactsLH, contactsRH])
        self.setContactsPosWF(contactsBF)
        print contactsBF
        stanceFlags = [received_data.ee_contact[0],
                       received_data.ee_contact[1],
                       received_data.ee_contact[2],
                       received_data.ee_contact[3]]
        self.setActiveContacts(stanceFlags)
        quaternions = [received_data.base.pose.orientation.w,
                received_data.base.pose.orientation.x,
                received_data.base.pose.orientation.y,
                received_data.base.pose.orientation.z]
        rotation = Rot.from_quat(quaternions)
        r,p,y = rotation.as_euler('zyx', degrees=False)
        euler = [0.0, 0.0, 0.0]
        self.setEulerAngles(euler)
        self.useContactTorque = True
        self.externalCentroidalWrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        comLinVelWrtWF = 0.0*np.array([received_data.base.twist.linear.x,
           received_data.base.twist.linear.y,
           received_data.base.twist.linear.z])

        w_R_b = self.math.rpyToRot(0.0,0.0,0.0);
        comLinVelWrtBF = 0.0*w_R_b.dot(comLinVelWrtWF)
        comAngAccBF = [0.0, 0.0, 0.0]
        self.setCoMAngAcc(comAngAccBF)

        self.comLinVel = comLinVelWrtBF
        #acc = [received_data.base.accel.linear.x,
        #                   received_data.base.accel.linear.y,
        #                   received_data.base.accel.linear.z]
        acc = [0.0, 0.0, 0.0]
        self.setCoMLinAcc(acc)

        #self.setTorqueLims(self..robotModel.robotModel.joint_torque_limits)

        constraint_mode_IP = ['FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION',
                              'FRICTION_AND_ACTUATION']

        self.setConstraintModes(constraint_mode_IP)

        axisZ = np.array([[0.0], [0.0], [1.0]])
        n1 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LF
        n2 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RF
        n3 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LH
        n4 = np.transpose(np.transpose(self.math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RH
        normals = np.vstack([n1, n2, n3, n4])
        self.setContactNormals(normals)
        self.setFrictionCoefficient(0.8)
        self.externalForceWF = [0.0, 0.0, 0.0]