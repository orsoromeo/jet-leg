import numpy as np
from jet_leg.dynamics.computational_dynamics import ComputationalDynamics
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
import copy


class Jacobians:
    def __init__(self, robot_name):
        self.compDyn = ComputationalDynamics(robot_name)
        self.compGeom = ComputationalGeometry()
        self.delta = 0.001

    def computeComPosJacobianSecondOrderCentral(self, params):
        jacobian = np.zeros(3)
        initialCoMPos = copy.copy(params.comPositionWF)
        print "initialCoMPos ", initialCoMPos

        for j in np.arange(0, 3):
            params.comPositionWF = initialCoMPos
            params.comPositionWF[j] = initialCoMPos[j] + self.delta / 2.0
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params, "COM")

            params.comPositionWF = initialCoMPos
            isPointFeasible, margin0 = self.compDyn.compute_IP_margin(params, "COM")
            diff = margin1 - margin0
            jac_left = diff / self.delta

            params.comPositionWF = initialCoMPos
            params.comPositionWF[j] = initialCoMPos[j] - self.delta / 2.0
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params, "COM")
            diff = margin0 - margin2
            jac_right = diff / self.delta

            diff = jac_left - jac_right
            jacobian[j] = diff / self.delta
            #print "[computeComPosJacobian]", margin1, margin2, self.delta, jacobian[j]

        return jacobian

    def computeComPosJacobian(self, params, contactsWF):
        jacobian = np.zeros(3)
        default = copy.deepcopy(contactsWF)

        for j in np.arange(0, 3):
            feet_pos_dim = copy.deepcopy(default[:, j])
            feet_pos_dim -= self.delta
            feet_pos_WF_plus = copy.deepcopy(default)
            feet_pos_WF_plus[:, j] = feet_pos_dim
            isPointFeasible, margin1 = self.computeComMargin(params, feet_pos_WF_plus)
            feet_pos_dim = copy.deepcopy(default[:, j])
            feet_pos_dim += self.delta
            feet_pos_WF_minus = copy.deepcopy(default)
            feet_pos_WF_minus[:, j] = feet_pos_dim
            isPointFeasible, margin2 = self.computeComMargin(params, feet_pos_WF_minus)
            diff = margin1 - margin2
            jacobian[j] = diff / (2.0*self.delta)

        return jacobian

    def computeBaseOrientationJacobian(self, params):
        jacobian = np.zeros(3)
        initialBaseOrient = copy.copy(params.eurlerAngles)
        #print "initialBaseOrient ", initialBaseOrient

        for j in np.arange(0, 3):
            params.eurlerAngles = initialBaseOrient
            params.eurlerAngles[j] = initialBaseOrient[j] + self.delta / 2.0
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params, "COM")
            params.eurlerAngles = initialBaseOrient
            params.eurlerAngles[j] = initialBaseOrient[j] - self.delta / 2.0
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params, "COM")
            diff = margin1 - margin2
            jacobian[j] = diff / self.delta
            #print "[computeBaseOrientationJacobian]", margin1, margin2, self.delta, jacobian[j]

        return jacobian

    def computeComEulerAnglesJacobian(self, params):
        jacobian = np.zeros(3)
        initiaPoint = params.getOrientation()

        for j in np.arange(0, 3):
            eulerAngles = initiaPoint
            eulerAngles[j] = initiaPoint[j] + self.delta / 2.0
            params.setEulerAngles(eulerAngles)
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params, "COM")

            eulerAngles = initiaPoint
            eulerAngles[j] = initiaPoint[j] - self.delta / 2.0
            params.setEulerAngles(eulerAngles)
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params, "COM")
            diff = margin1 - margin2
            jacobian[j] = diff / self.delta

        return jacobian

    def computeComLinVelJacobian(self, params):
        jacobian = np.zeros(3)
        initiaPoint = params.comLinVel

        for j in np.arange(0, 3):
            params.comLinVel = initiaPoint
            params.comLinVel[j] = initiaPoint[j] + self.delta / 2.0
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params, "COM")

            params.comLinVel = initiaPoint
            params.comLinVel[j] = initiaPoint[j] - self.delta / 2.0
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params, "COM")
            diff = margin1 - margin2
            jacobian[j] = diff / self.delta

        return jacobian

    def computeComLinAccJacobian(self, params):
        jacobian = np.zeros(3)
        initiaPoint = params.comLinAcc

        for j in np.arange(0, 3):
            params.comLinAcc = initiaPoint
            params.comLinAcc[j] = initiaPoint[j] + self.delta / 2.0
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params, "COM")

            params.comLinAcc = initiaPoint
            params.comLinAcc[j] = initiaPoint[j] - self.delta / 2.0
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params, "COM")
            diff = margin1 - margin2
            jacobian[j] = diff / self.delta

        return jacobian

    def computeFeetJacobian(self, params):
        jacobian = np.zeros(12)
        for footID in np.arange(0, 4):
            jacobian[footID * 3: footID * 3 + 3] = self.computeFootJacobian(params, footID)
        return jacobian

    def computeFootJacobian(self, params, footID):
        jacobian = np.zeros(3)
        initialContacts = copy.copy(params.getContactsPosWF())
        initiaPoint = copy.copy(initialContacts[footID])
        for j in np.arange(0, 3):
            contacts = initialContacts
            contacts[footID, j] = initiaPoint[j] + self.delta / 2.0
            params.setContactsPosWF(contacts)
            isPointFeasible, margin1 = self.compDyn.compute_IP_margin(params, "COM")
            contacts = initialContacts
            contacts[footID, j] = initiaPoint[j] - self.delta / 2.0
            params.setContactsPosWF(contacts)
            isPointFeasible, margin2 = self.compDyn.compute_IP_margin(params, "COM")
            diff = margin1 - margin2
            jacobian[j] = diff / self.delta

        return jacobian

    def plotMarginAndJacobianOfMarginWrtComLinAcceleration(self, params, acc_range, dimension):
        num_of_tests = np.shape(acc_range)
        margin = np.zeros(num_of_tests)
        jac_com_lin_acc = np.zeros((3, num_of_tests[0]))
        count = 0
        for delta_acc in acc_range:
            #print "dimension", dimension
            #print "delta_acc", delta_acc
            params.comLinAcc = [0.0, 0.0, 0.0]
            params.comLinAcc[dimension] = delta_acc
            #print "count", count, params.comLinAcc, params.comPositionWF, params.getContactsPosWF()
            ''' compute iterative projection 
            Outputs of "iterative_projection_bretl" are:
            IP_points = resulting 2D vertices
            actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
            computation_time = how long it took to compute the iterative projection
            '''
            IP_points, force_polytopes, IP_computation_time = self.compDyn.try_iterative_projection_bretl(params)

            '''I now check whether the given CoM configuration is stable or not'''
            isCoMStable, contactForces, forcePolytopes = self.compDyn.check_equilibrium(params)
            #print "is CoM stable?", isCoMStable
            # print 'Contact forces:', contactForces
            #print 'IP_points:', IP_points
            #if IP_points is True:
            #    print "ip points shape", np.shape(IP_points)
            #    facets = self.compGeom.compute_halfspaces_convex_hull(IP_points)
            #    point2check = self.compDyn.getReferencePoint(params)
            #    print "point 2 check", point2check
            #    isPointFeasible, margin[count] = self.compGeom.isPointRedundant(facets, point2check)
            isPointFeasible, margin[count] = self.compDyn.compute_IP_margin(params, "COM")
            #
            marginJAcWrtComLinAcc = self.computeComLinAccJacobian(params)
            #    print "marginJAcWrtComLinAcc", marginJAcWrtComLinAcc
            jac_com_lin_acc[:, count] = marginJAcWrtComLinAcc
            #else:
            #    jac_com_lin_acc[:, count] = [-1000] * 3

            # print "com lin vel jacobian", jac_com_lin_vel

            count += 1

        return margin, jac_com_lin_acc

    def plotMarginAndJacobianOfMarginWrtComVelocity(self, params, velocity_range, dimension):
        num_of_tests = np.shape(velocity_range)
        margin = np.zeros(num_of_tests)
        jac_com_lin_vel = np.zeros((3, num_of_tests[0]))
        count = 0
        for delta_vel in velocity_range:
            print "delta_vel", delta_vel
            params.comLinVel = [0.0, 0.0, 0.0]
            params.comLinVel[dimension] = delta_vel
            print "count", count, params.comLinVel, params.comPositionWF, params.getContactsPosWF()
            ''' compute iterative projection 
            Outputs of "iterative_projection_bretl" are:
            IP_points = resulting 2D vertices
            actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
            computation_time = how long it took to compute the iterative projection
            '''
            #IP_points, force_polytopes, IP_computation_time = self.compDyn.iterative_projection_bretl(params)
            #
            #'''I now check whether the given CoM configuration is stable or not'''
            #isCoMStable, contactForces, forcePolytopes = self.compDyn.check_equilibrium(params)
            #print "is CoM stable?", isCoMStable
            ## print 'Contact forces:', contactForces
            #
            #facets = self.compGeom.compute_halfspaces_convex_hull(IP_points)
            #point2check = self.compDyn.getReferencePoint(params)
            #print "point 2 check", point2check
            #isPointFeasible, margin[count] = self.compGeom.isPointRedundant(facets, point2check)
            isPointFeasible, margin[count] = self.compDyn.compute_IP_margin(params, "COM")

            marginJAcWrtComLinVel = self.computeComLinVelJacobian(params)
            #print "marginJAcWrtComLinVel", marginJAcWrtComLinVel
            jac_com_lin_vel[:, count] = marginJAcWrtComLinVel
            # print "com lin vel jacobian", jac_com_lin_vel

            count += 1

        return margin, jac_com_lin_vel

    def computeComMargin(self, params, new_contacts_wf):
        default_feet_pos_WF = copy.deepcopy(params.getContactsPosWF())
        params.setContactsPosWF(new_contacts_wf)
        isPointFeasible, margin = self.compDyn.compute_IP_margin(params, "COM")
        params.setContactsPosWF(default_feet_pos_WF)
        return isPointFeasible, margin

    def plotMarginAndJacobianWrtComPosition(self, params, com_pos_range, dim_to_check):
        num_of_tests = np.shape(com_pos_range)
        margin = np.zeros(num_of_tests)
        jac_com_pos = np.zeros((3, num_of_tests[0]))
        count = 0
        default_feet_pos_WF = copy.deepcopy(params.getContactsPosWF())

        for delta in com_pos_range:
            """ contact points in the World Frame"""
            contactsWF = copy.deepcopy(default_feet_pos_WF)
            contactsWF[:, dim_to_check] -= delta
            isPointFeasible, margin[count] = self.computeComMargin(params, contactsWF)
            jac_com_pos[:, count] = self.computeComPosJacobian(params, contactsWF)

            count += 1

        params.setContactsPosWF(default_feet_pos_WF)
        return margin, jac_com_pos

    '''
    @brief this function computes the jacobian of the feasible region margin wrt to the base orientation 
    for a predefined set of base orienation values
    @params params = iterative projection parameters 
    @params dim_to_check = 0 (roll), dim_to_check = 1 (pitch)
    @params base_orient_range = set of values to use for the roll/pitch test
    '''
    def plotMarginAndJacobianWrtBaseOrientation(self, params, base_orient_range, dim_to_check):
        num_of_tests = np.shape(base_orient_range)
        margin = np.zeros(num_of_tests)
        jac_base_orient = np.zeros((3, num_of_tests[0]))
        count = 0
        default_euler_angles = copy.deepcopy(params.eurlerAngles)
        print "default euler", default_euler_angles

        for delta in base_orient_range:
            eulerAngles = copy.deepcopy(default_euler_angles)
            eulerAngles[dim_to_check] -= delta
            params.setEulerAngles(eulerAngles)

            isPointFeasible, margin[count] = self.compDyn.compute_IP_margin(params, "COM")
            print "params.eurlerAngles ", params.eurlerAngles
            print "margin ", margin[count]
            marginJAcWrtBaseOrient = self.computeBaseOrientationJacobian(params)
            print "margin jac wrt base orient", marginJAcWrtBaseOrient
            jac_base_orient[:, count] = marginJAcWrtBaseOrient

            count += 1

        return margin, jac_base_orient

    def plotMarginAndJacobianWrtFootPosition(self, params, foot_id, foot_pos_range, dimension_to_check):
        num_of_tests = np.shape(foot_pos_range)
        margin = np.zeros(num_of_tests)
        jac_com_pos = np.zeros(num_of_tests)
        count = 0

        for delta in foot_pos_range:
            """ contact points in the World Frame"""
            LF_foot = np.array([0.3, 0.2, -0.4])
            RF_foot = np.array([0.3, -0.2, -0.4])
            LH_foot = np.array([-0.3, 0.2, -0.4])
            RH_foot = np.array([-0.3, -0.2, -0.4])

            contactsWF = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
            contactsWF[foot_id, dimension_to_check] += delta
            params.setContactsPosWF(contactsWF)

            #print "contacts ", params.getContactsPosWF()

            ''' compute iterative projection 
            Outputs of "iterative_projection_bretl" are:
            IP_points = resulting 2D vertices
            actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
            computation_time = how long it took to compute the iterative projection
            '''
            IP_points, force_polytopes, IP_computation_time = self.compDyn.iterative_projection_bretl(params)

            '''I now check whether the given CoM configuration is stable or not'''
            isCoMStable, contactForces, forcePolytopes = self.compDyn.check_equilibrium(params)
            print "is CoM stable?", isCoMStable
            # print 'Contact forces:', contactForces

            facets = self.compGeom.compute_halfspaces_convex_hull(IP_points)
            point2check = self.compDyn.getReferencePoint(params)
            print count, params.comLinVel, params.comPositionWF
            isPointFeasible, margin[count] = self.compGeom.isPointRedundant(facets, point2check)

            marginJacWrtComPos = self.computeComPosJacobian(params)
            print "margin jac wrt com pos", marginJacWrtComPos
            jac_com_pos[count] = marginJacWrtComPos[1]
            count += 1

        return margin, jac_com_pos
