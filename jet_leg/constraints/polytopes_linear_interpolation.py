import numpy as np
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.kinematics.kinematics_interface import KinematicsInterface
from scipy.linalg import block_diag
from jet_leg.robots.dog_interface import DogInterface
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics

class polytopesLinearInterpolation:
    def __init__(self, robot_kinematics, comp_dynamics):
        self.kin = robot_kinematics
        self.compDyn = comp_dynamics
        self.math = Math()
        self.dog = DogInterface()
        self.rbd = RigidBodyDynamics()
        self.maxRetractionFeetPos = np.zeros((4,4))
        self.maxExtensionFeetPos = np.zeros((4,4))
        LF_foot = np.array([0.3, 0.2, -0.4])
        RF_foot = np.array([0.3, -0.2, -0.4])
        LH_foot = np.array([-0.3, 0.2, -0.4])
        RH_foot = np.array([-0.3, -0.2, -0.4])
        self.maxRetractionFeetPos = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
        LF_foot = np.array([0.3, 0.2, -0.4])
        RF_foot = np.array([0.3, -0.2, -0.4])
        LH_foot = np.array([-0.3, 0.2, -0.4])
        RH_foot = np.array([-0.3, -0.2, -0.4])
        self.maxExtensionFeetPos = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
        trunk_mass = 85.
        mu = 0.5

        ''' stanceFeet vector contains 1 is the foot is on the ground and 0 if it is in the air'''
        stanceFeet = [1, 1, 1, 1]

        randomSwingLeg = random.randint(0, 3)
        tripleStance = False  # if you want you can define a swing leg using this variable
        if tripleStance:
            print 'Swing leg', randomSwingLeg
            stanceFeet[randomSwingLeg] = 0
        print 'stanceLegs ', stanceFeet

        ''' now I define the normals to the surface of the contact points. By default they are all vertical now'''
        axisZ = array([[0.0], [0.0], [1.0]])

        n1 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LF
        n2 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RF
        n3 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # LH
        n4 = np.transpose(np.transpose(math.rpyToRot(0.0, 0.0, 0.0)).dot(axisZ))  # RH
        normals = np.vstack([n1, n2, n3, n4])

        ''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
        HAA = Hip Abduction Adduction
        HFE = Hip Flextion Extension
        KFE = Knee Flextion Extension
        '''
        LF_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        RF_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        LH_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        RH_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])

        ''' extForceW is an optional external pure force (no external torque for now) applied on the CoM of the robot.'''
        extForceW = np.array([0.0, 0.0, 0.0])  # units are Nm

        '''You now need to fill the 'params' object with all the relevant 
            informations needed for the computation of the IP'''
        params = IterativeProjectionParameters()
        params.setContactsPosWF(contacts)
        params.setCoMPosWF(comWF)
        params.setTorqueLims(torque_limits)
        params.setActiveContacts(stanceFeet)
        params.setConstraintModes(constraint_mode_IP)
        params.setContactNormals(normals)
        params.setFrictionCoefficient(mu)
        params.setNumberOfFrictionConesEdges(ng)
        params.setTotalMass(trunk_mass)
        params.externalForceWF = extForceW
        C, d, isIKoutOfWorkSpace, forcePolytopes = comp_dyn.constr.getInequalities(params)
        self.maxRetractionPolytope = np.zeros((8,3))
        self.maxExtensionPolytope = np.zeros((8,3))

    def interpolate(self, footPos):