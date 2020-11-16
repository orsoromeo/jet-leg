# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 15:57:17 2018

@author: Romeo Orsolino
"""

import pypoman
import numpy as np
from numpy import array, dot, eye, hstack, vstack, zeros
from scipy.spatial import ConvexHull
from jet_leg.constraints.constraints import Constraints
from jet_leg.kinematics.kinematics_interface import KinematicsInterface
from jet_leg.robots.robot_model_interface import RobotModelInterface
from jet_leg.computational_geometry.math_tools import Math
from jet_leg.computational_geometry.geometry import Geometry
from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
from jet_leg.dynamics.instantaneous_capture_point import InstantaneousCapturePoint
from jet_leg.dynamics.zero_moment_point import ZeroMomentPoint
from jet_leg.dynamics.rigid_body_dynamics import RigidBodyDynamics
from cvxopt import matrix, solvers
import time


class ComputationalDynamics:
    def __init__(self, robot_name):
        self.robotName = robot_name
        self.geom = Geometry()
        self.math = Math()
        self.kin = KinematicsInterface(self.robotName)
        self.robotModel = RobotModelInterface(self.robotName)
        self.constr = Constraints(self.kin, self.robotModel)
        self.ineq = ([],[])
        self.eq = ([],[])
        self.rbd = RigidBodyDynamics()
        self.compGeom = ComputationalGeometry()
        self.icp = InstantaneousCapturePoint()
        self.zmp = ZeroMomentPoint()

    ''' 
    This function is used to prepare all the variables that will be later used 
    for the computation of the Iterative Projection algorithm 
    
    The arguments are: 
    constraint_mode = either 'ONLY_ACTUATION' or 'ONLY_FRICTION'
    comWF = position of the CoM
    contacts = location of the nc contacts
    normals = [3x nc] matrix containing the normals of each contact point on its rows
    trunk_mass = mass of the robots trunk (excluding the legs mass) in Kg 
    nc = number of point contacts
    ng = number of edges to be used to linearize the friction cones (in case that the friction constraint is considered)
    mu = friction coefficient (we assume here the same coeff for all the contact points)
    saturate_normal_force = if True this sets a max constant value on the normal force of the friction cones
    '''
    def setup_iterative_projection(self, iterative_projection_params, saturate_normal_force):
        #print "CoM position in IP setup ", iterative_projection_params.getCoMPosWF()
        stanceLegs = iterative_projection_params.getStanceFeet()
        contactsWF = iterative_projection_params.getContactsPosWF()
        contactsNumber = np.sum(stanceLegs)

        if contactsNumber > 2:
            iterative_projection_params.useContactTorque = False
        else:
            iterative_projection_params.useContactTorque = True

        # Unprojected state is:
        #
        #     x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
        Ex = np.zeros((0)) 
        Ey = np.zeros((0))        
        G = np.zeros((6,0))

        totalCentroidalWrench = self.rbd.computeCentroidalWrench(iterative_projection_params.robotMass,
                                                                 self.robotModel.robotModel.trunkInertia,
                                                                 iterative_projection_params.getCoMPosWF(),
                                                                 iterative_projection_params.externalCentroidalWrench,
                                                                 iterative_projection_params.getCoMLinAcc(),
                                                                 iterative_projection_params.getCoMAngAcc())

        stanceIndex = iterative_projection_params.getStanceIndex(stanceLegs)

        for j in range(0,contactsNumber):
            r = contactsWF[int(stanceIndex[j]),:]
            #print "foot ", r
            if not iterative_projection_params.useContactTorque:
                graspMatrix = self.math.getGraspMatrix(r)[:,0:3]
            else:
                graspMatrix = self.math.getGraspMatrix(r)[:,0:5]
            Ex = hstack([Ex, -graspMatrix[4]])
            Ey = hstack([Ey, graspMatrix[3]])
            G = hstack([G, graspMatrix])

        #print "totalCentroidalWrench", totalCentroidalWrench
        E = vstack((Ex, Ey)) / (totalCentroidalWrench[2] )
        #f = zeros(2)
        f = hstack([ totalCentroidalWrench[4], - totalCentroidalWrench[3]])  / (totalCentroidalWrench[2])
        proj = (E, f)  # y = E * x + f
        
        # see Equation (52) in "ZMP Support Areas for Multicontact..."
        A_f_and_tauz = array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1]])
        A = dot(A_f_and_tauz, G)
        t = hstack([totalCentroidalWrench[0:3], totalCentroidalWrench[5]])
        #print "total wrench ", totalCentroidalWrench
#        print 'mass ', robotMass
        eq = (A, t)  # A * x == t

        C, d, isIKoutOfWorkSpace, actuation_polygons = self.constr.getInequalities(iterative_projection_params)
        ineq = (C, d)    
        return proj, eq, ineq, actuation_polygons, isIKoutOfWorkSpace
        
    def reorganizeActuationPolytopes(self, actPolytope):
        outputPolytopeX = np.zeros((1,8))
        outputPolytopeY = np.zeros((1,8))
        outputPolytopeZ = np.zeros((1,8))
        outputPolytope = np.array([[outputPolytopeX], [outputPolytopeY], [outputPolytopeZ]])
#        print 'out poly',outputPolytope[1]
#        print 'out poly X',outputPolytopeX
        for i in range(0,8):
#            print i
            if(actPolytope[0][i]>0) and (actPolytope[1][i]<0) and (actPolytope[2][i]<0):
                outputPolytopeX[0][0] = actPolytope[0][i]
                outputPolytopeY[0][0] = actPolytope[1][i]
                outputPolytopeZ[0][0] = actPolytope[2][i]
            if(actPolytope[0][i]>0) and (actPolytope[1][i]>-10.) and (actPolytope[2][i]<0):
                outputPolytopeX[0][1] = actPolytope[0][i]
                outputPolytopeY[0][1] = actPolytope[1][i]
                outputPolytopeZ[0][1] = actPolytope[2][i]
            if(actPolytope[0][i]<0) and (actPolytope[1][i]>0) and (actPolytope[2][i]<0):
                outputPolytopeX[0][2] = actPolytope[0][i]
                outputPolytopeY[0][2] = actPolytope[1][i]
                outputPolytopeZ[0][2] = actPolytope[2][i]
            if(actPolytope[0][i]<0) and (actPolytope[1][i]<0) and (actPolytope[2][i]<0):
                outputPolytopeX[0][3] = actPolytope[0][i]
                outputPolytopeY[0][3] = actPolytope[1][i]
                outputPolytopeZ[0][3] = actPolytope[2][i]
            if(actPolytope[0][i]>0) and (actPolytope[1][i]<0) and (actPolytope[2][i]>0):
                outputPolytopeX[0][4] = actPolytope[0][i]
                outputPolytopeY[0][4] = actPolytope[1][i]
                outputPolytopeZ[0][4] = actPolytope[2][i]
            if(actPolytope[0][i]>0) and (actPolytope[1][i]>0) and (actPolytope[2][i]>0):
                outputPolytopeX[0][5] = actPolytope[0][i]
                outputPolytopeY[0][5] = actPolytope[1][i]
                outputPolytopeZ[0][5] = actPolytope[2][i]
            if(actPolytope[0][i]<0) and (actPolytope[1][i]>0) and (actPolytope[2][i]>0):
                outputPolytopeX[0][6] = actPolytope[0][i]
                outputPolytopeY[0][6] = actPolytope[1][i]
                outputPolytopeZ[0][6] = actPolytope[2][i]
            if(actPolytope[0][i]<0) and (actPolytope[1][i]<10.) and (actPolytope[2][i]>0):
                outputPolytopeX[0][7] = actPolytope[0][i]
                outputPolytopeY[0][7] = actPolytope[1][i]
                outputPolytopeZ[0][7] = actPolytope[2][i]
#        outputPolytope = actPolytope
        
        outputPolytope = np.array([[outputPolytopeX], [outputPolytopeY], [outputPolytopeZ]])
#        print 'out poly',outputPolytope
#        print 'input ', actPolytope
#        print 'out poly X',outputPolytopeX
#        print 'out poly Y',outputPolytopeY
#        print 'out poly Z',outputPolytopeZ
        return outputPolytope

    def try_iterative_projection_bretl(self, iterative_projection_params, saturate_normal_force = False):
        try:
            compressed_hull, actuation_polygons, computation_time = self.iterative_projection_bretl(iterative_projection_params, saturate_normal_force)
            return compressed_hull, actuation_polygons, computation_time
        except ValueError as err:
            print 'Could not compute the feasible region'
            print(err.args)
            return False, False, False

    def iterative_projection_bretl(self, iterative_projection_params, saturate_normal_force = False):

        start_t_IP = time.time()
#        print stanceLegs, contacts, normals, comWF, ng, mu, saturate_normal_force
        proj, self.eq, self.ineq, actuation_polygons, isIKoutOfWorkSpace = self.setup_iterative_projection(iterative_projection_params, saturate_normal_force)
        if isIKoutOfWorkSpace:
            return False, False, False
        else:
            vertices_WF = pypoman.project_polytope(proj, self.ineq, self.eq, method='bretl', max_iter=500, init_angle=0.0)
            if vertices_WF is False:
                print 'Project polytope function is False'
                return False, False, False

            else:
                compressed_vertices = np.compress([True, True], vertices_WF, axis=1)
                try:
                    hull = ConvexHull(compressed_vertices)
                except Exception as err:
                    print("QHull type error: " + str(err))
                    print("matrix to compute qhull:",compressed_vertices)
                    return False, False, False

            compressed_hull = compressed_vertices[hull.vertices]
            compressed_hull = self.geom.clockwise_sort(compressed_hull)
            compressed_hull = compressed_hull
            computation_time = (time.time() - start_t_IP)
            return compressed_hull, actuation_polygons, computation_time
        
        
    def instantaneous_actuation_region_bretl(self, stanceLegs, contacts, normals, total_mass, comWF = np.array([0.0,0.0,0.0])):
        constraint_mode = 'ONLY_ACTUATION'
        number_of_generators = 4
        mu = 1.0
        IP_points, actuation_polygons, computation_time = self.iterative_projection_bretl(constraint_mode, stanceLegs, contacts, normals, total_mass, number_of_generators, mu, comWF)
        IP_points = self.geom.clockwise_sort(np.array(IP_points))
        
        return IP_points, actuation_polygons, computation_time

    def support_region_bretl(self, stanceLegs, contacts, normals, total_mass, number_of_generators = 4, mu = 1.0, comWF = np.array([0.0,0.0,0.0])):
        constraint_mode = 'ONLY_FRICTION'
        IP_points, actuation_polygons, computation_time = self.iterative_projection_bretl(constraint_mode, stanceLegs, contacts, normals, total_mass, number_of_generators, mu, comWF)
        IP_points = self.geom.clockwise_sort(np.array(IP_points))
        
        return IP_points, actuation_polygons, computation_time

    def check_equilibrium(self, LPparams, useVariableJacobian = False, verbose = False):

        p, G, h, A, b, isIKoutOfWorkSpace, LP_actuation_polygons = self.setup_lp(LPparams)

        if isIKoutOfWorkSpace:
            #unfeasible_points = np.vstack([unfeasible_points, com_WF])
            print 'something is wrong in the inequalities or the point is out of workspace'
            x = -1
            return False, x, LP_actuation_polygons
        else:
            #print 'Solving LP'
            sol = solvers.lp(p, G, h, A, b)
            x = sol['x']
            status = sol['status']
            if x == None:
                isConfigurationStable = False
            else:
                isConfigurationStable = True
            return isConfigurationStable, x, LP_actuation_polygons
                
    ''' 
    This function is used to check every single CoM position and see if there is a feasible set of contact forces for that configuration.
    This function can consider either friction constraints or actuation constraints separately (depending on the value of the constraint_mode variable).
    
    The arguments are: 
    constraint_mode = either 'ONLY_ACTUATION' or 'ONLY_FRICTION'
    contacts = location of the nc contacts
    normals = [3x nc] matrix containing the normals of each contact point on its rows
    mass = mass of the robots trunk (excluding the legs mass) in Kg 
    friction_coeff = friction coefficient (we assume here the same coeff for all the contact points)
    nc = number of point contacts
    ng = number of edges to be used to linearize the friction cones (in case that the friction constraint is considered)
    '''
    def LP_projection(self, LPparams, useVariableJacobian = False, stepX = 0.05, stepY = 0.05, stepZ = 0.05):
        start_t_LP = time.time()
        stanceLegs = LPparams.getStanceFeet()
        nc = np.sum(stanceLegs)
        feasible_points = np.zeros((0,3))
        unfeasible_points = np.zeros((0,3))
        contact_forces = np.zeros((0,nc*3))  
        verbose = False
        com_WF = np.array([0.0, 0.0, 0.0])
#        default_com_WF = com_WF
#        p, G, h, A, b, isConstraintOk, LP_actuation_polygons = self.setup_lp(LPparams)
        
        """ Defining the equality constraints """
        for com_x in np.arange(-0.25,0.25,stepX):
            for com_y in np.arange(-0.2,0.2,stepY):
                for com_z in np.arange(-0.2,0.25,stepZ):
                    com_WF = np.array([com_x, com_y, com_z])
                    #LPparams.setCoMPosWF(com_WF)
                    status, x, force_polytopes = self.check_equilibrium(LPparams)
        #            if useVariableJacobian:
        #                p, G, h, A, b, isConstraintOk, LP_actuation_polygons = self.setup_lp(LPparams)
        #            else:
        #                p, G, h, A, b, isConstraintOk, LP_actuation_polygons = self.setup_lp(LPparams)
        #
        #            if not isConstraintOk:
        #                unfeasible_points = np.vstack([unfeasible_points, com_WF])
        #                if verbose:
        #                    print 'something is wrong in the inequalities or the point is out of workspace'
        #            else:
        #                sol=solvers.lp(p, G, h, A, b)
        #                x = sol['x']
        #                status = sol['status']
                        #print x
                    if status == 'optimal':
                        feasible_points = np.vstack([feasible_points, com_WF])
                        contact_forces = np.vstack([contact_forces, np.transpose(x)])
                    else:
                        unfeasible_points = np.vstack([unfeasible_points, com_WF])

                    
        print("LP test: --- %s seconds ---" % (time.time() - start_t_LP))
        
        return feasible_points, unfeasible_points, contact_forces

    ''' 
    This function is used to prepare all the variables that will be later used 
    for the computation of the LP ground truth. 
    
    The arguments are: 
    mass = mass of the robots trunk (excluding the legs mass) in Kg 
    nc = number of point contacts
    contacts = location of the nc contacts
    numberOfGenerators = number of edges to be used to linearize the friction cones (in case that the friction constraint is considered)
    normals = [3x nc] matrix containing the normals of each contact point on its rows
    comWorldFrame = position of the CoM
    constraint_mode = either 'ONLY_ACTUATION' or 'ONLY_FRICTION'
    friction_coeff = friction coefficient (we assume here the same coeff for all the contact points)
    '''
    def setup_lp(self, LPparams):

        g = 9.81

        #        trunk_mass = LPparams.getTrunkMass()
        stanceLegs = LPparams.getStanceFeet()
        totMass = LPparams.robotMass
        nc = np.sum(stanceLegs)
        grav = np.array([[0.], [0.], [-g*totMass]])
        if not LPparams.useContactTorque:
            p = matrix(np.zeros((3*nc,1)))
        else:
            p = matrix(np.zeros((5*nc,1)))

        contactsPosWF = LPparams.getContactsPosWF()
        totalCentroidalWrench = self.rbd.computeCentroidalWrench(LPparams.robotMass,
                                                                 self.robotModel.robotModel.trunkInertia,
                                                                 LPparams.getCoMPosWF(),
                                                                 LPparams.externalCentroidalWrench,
                                                                 LPparams.getCoMLinAcc(),
                                                                 LPparams.getCoMAngAcc())
        if np.sum(stanceLegs) == 1:
            A = np.zeros((5,0))
        else:
            A = np.zeros((6,0))
        stanceIndex = LPparams.getStanceIndex(stanceLegs)
        for j in stanceIndex:
            j = int(j)
            #print 'index in lp ',j
            r = contactsPosWF[j,:]
            GraspMat = self.math.getGraspMatrix(r)
            if not LPparams.useContactTorque:
                A = np.hstack((A, GraspMat[:,0:3]))
                A = matrix(A)
                b = matrix(totalCentroidalWrench.reshape((6)))
            else:
                ''' non-zero foot size case'''
                if np.sum(stanceLegs) == 1:
                    ''' if there is only one stance foot the problem is overconstrained and we can remove the constraint on tau_z'''
                    A = np.hstack((A, GraspMat[0:5,0:5]))
                    A = matrix(A)
                    totW = totalCentroidalWrench[0:5]
                    b = matrix(totW.reshape((5)))
                else:
                    A = np.hstack((A, GraspMat[:,0:5]))
                    A = matrix(A)
                    b = matrix(totalCentroidalWrench.reshape((6)))



        G, h, isIKoutOfWorkSpace, LP_actuation_polygons = self.constr.getInequalities(LPparams)
        G = matrix(G)
        h = matrix(h)

        lp = p, G, h, A, b
        return p, G, h, A, b, isIKoutOfWorkSpace, LP_actuation_polygons

    def compute_IP_margin(self, iterative_projection_params, reference_type, saturate_normal_force = False):
        ''' compute iterative projection
        Outputs of "iterative_projection_bretl" are:
        IP_points = resulting 2D vertices
        actuation_polygons = these are the vertices of the 3D force polytopes (one per leg)
        computation_time = how long it took to compute the iterative projection
        '''
        IP_points, force_polytopes, IP_computation_time = self.try_iterative_projection_bretl(iterative_projection_params)
        if IP_points is not False:
            facets = self.compGeom.compute_halfspaces_convex_hull(IP_points)
            reference_point = self.getReferencePoint(iterative_projection_params, reference_type)
            isPointFeasible, margin = self.compGeom.isPointRedundant(facets, reference_point)
            return isPointFeasible, margin
        else:
            print "Warning! IP failed."
            return False, -1000.0

    def getReferencePoint(self, iterative_projection_params, type):
        comWF = iterative_projection_params.getCoMPosWF()
        if(type=="ICP"):
            print "compute ICP"
            ICP = self.icp.compute(iterative_projection_params)
            iterative_projection_params.instantaneousCapturePoint = ICP
            referencePoint = np.array([ICP[0], ICP[1]])
        elif(type=="ZMP"):
            ZMP = self.zmp.compute(iterative_projection_params)
            referencePoint = np.array([ZMP[0], ZMP[1]])
        elif(type=="COM"):
            referencePoint = np.array([comWF[0], comWF[1]])
        else:
            print "Reference point type unspecified"

        return referencePoint

    def getReferencePointWrtBaseFrame(self, iterative_projection_params):
        pendulum_height = 100.0
        iterative_projection_params.setCoMPosWF([0.0, 0.0, pendulum_height])
        comWF = iterative_projection_params.getCoMPosWF()
        if(iterative_projection_params.useInstantaneousCapturePoint):
            ICP = self.icp.compute(iterative_projection_params)
            iterative_projection_params.instantaneousCapturePoint = ICP
            referencePoint = np.array([ICP[0], ICP[1]])
        else:
            referencePoint = np.array([comWF[0], comWF[1]])

        return referencePoint
