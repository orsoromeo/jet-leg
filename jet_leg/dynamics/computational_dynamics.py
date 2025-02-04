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
from jet_leg.maths.math_tools import Math
from jet_leg.maths.geometry import Geometry
from cvxopt import matrix, solvers
import time


class ComputationalDynamics:
    def __init__(self, robot_name):
        self.robotName = robot_name
        self.geom = Geometry()
        self.math = Math()
        self.kin = KinematicsInterface(self.robotName)
        self.constr = Constraints(self.kin)
        self.ineq = ([],[])
        self.eq = ([],[])

    def getGraspMatrix(self, r):

        G = np.vstack([np.hstack([eye(3), zeros((3, 3))]),np.hstack([self.math.skew(r), eye(3)])])
        return G

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
        
        stanceLegs = iterative_projection_params.getStanceFeet()
   
        contactsWF = iterative_projection_params.getContactsPosWF()
        robotMass = iterative_projection_params.robotMass
        
        ''' parameters to be tuned'''
        g = 9.81
        contactsNumber = np.sum(stanceLegs)
        
        # Unprojected state is:
        #
        #     x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
        Ex = np.zeros((0)) 
        Ey = np.zeros((0))        
        G = np.zeros((6,0))

        extForce = iterative_projection_params.externalForceWF
        stanceIndex = iterative_projection_params.getStanceIndex(stanceLegs)

        for j in range(0,contactsNumber):
           
            r = contactsWF[int(stanceIndex[j]),:]
            #print 'r is ', r
           
            graspMatrix = self.getGraspMatrix(r)[:,0:3]
            Ex = hstack([Ex, -graspMatrix[4]])
            Ey = hstack([Ey, graspMatrix[3]])
            G = hstack([G, graspMatrix])            
            
#        print 'grasp matrix',G
        E = vstack((Ex, Ey)) / (g*robotMass - extForce[2] )
        f = zeros(2)
        proj = (E, f)  # y = E * x + f
        
        # see Equation (52) in "ZMP Support Areas for Multicontact..."
        A_f_and_tauz = array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1]])
        A = dot(A_f_and_tauz, G)
#        print A
        t = hstack([- extForce[0], - extForce[1], g*robotMass - extForce[2], 0])
#        print extForceWF, t
#        print 'mass ', robotMass
#        print A,t
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
            print('Could not compute the feasible region')
            print(err.args)
            return False, False, False


    def iterative_projection_bretl(self, iterative_projection_params, saturate_normal_force = False):

        start_t_IP = time.time()
        proj, self.eq, self.ineq, actuation_polygons, isIKoutOfWorkSpace = self.setup_iterative_projection(iterative_projection_params, saturate_normal_force)

        if isIKoutOfWorkSpace:
            return False, False, False
        else:
            vertices_WF = pypoman.project_polytope(proj, self.ineq, self.eq, method='bretl', max_iter=500, init_angle=0.0)
            if vertices_WF is False:
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
        #vertices_WF = vertices_BF + np.transpose(comWF[0:2])
            computation_time = (time.time() - start_t_IP)

        if np.size(actuation_polygons,0) is 4:
            if np.size(actuation_polygons,1) is 3:
                p = self.reorganizeActuationPolytopes(actuation_polygons[1])

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
            print('something is wrong in the inequalities or the point is out of workspace')
            x = -1
            return False, x, LP_actuation_polygons
        else:
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

        p = matrix(np.zeros((3*nc,1)))

        contactsPosWF = LPparams.getContactsPosWF()
        contactsBF = np.zeros((4,3)) # this is just for initialization
        comWorldFrame = LPparams.getCoMPosWF()
        extForce = LPparams.externalForceWF
        totForce = grav
        totForce[0] += extForce[0]
        totForce[1] += extForce[1]
        totForce[2] += extForce[2]

        torque = -np.cross(comWorldFrame, np.transpose(totForce))
        A = np.zeros((6,0))
        stanceIndex = LPparams.getStanceIndex(stanceLegs)
        for j in stanceIndex:
            j = int(j)
            r = contactsPosWF[j,:]
            GraspMat = self.getGraspMatrix(r)
            A = np.hstack((A, GraspMat[:,0:3]))
            A = matrix(A)
            b = matrix(np.vstack([-totForce, np.transpose(torque)]).reshape((6)))

        G, h, isIKoutOfWorkSpace, LP_actuation_polygons = self.constr.getInequalities(LPparams)
        G = matrix(G)
        h = matrix(h)

        lp = p, G, h, A, b
        return p, G, h, A, b, isIKoutOfWorkSpace, LP_actuation_polygons