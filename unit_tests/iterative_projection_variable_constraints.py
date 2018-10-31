# -*- coding: utf-8 -*-
"""
Created on Sun Jul 22 21:00:59 2018

@author: Romeo Orsolino
"""

import numpy as np

from context import jet_leg 

import time
from pypoman.lp import solve_lp, GLPK_IF_AVAILABLE
#from pypoman.bretl import Vertex
#from pypoman.bretl import Polygon

import cvxopt
from cvxopt import matrix, solvers

from numpy import array, cos, cross, pi, sin
from numpy.random import random
from scipy.linalg import norm
from scipy.linalg import block_diag

from numpy import array, cross, dot, eye, hstack, vstack, zeros, matrix

import matplotlib.pyplot as plt
from jet_leg.plotting_tools import Plotter
from jet_leg.constraints import Constraints
from jet_leg.hyq_kinematics import HyQKinematics
from jet_leg.math_tools import Math
from jet_leg.computational_dynamics import ComputationalDynamics
from jet_leg.vertex_based_projection import VertexBasedProjection

class VertexVariableConstraints():
    def __init__(self, p):
        self.x = p[0]
        self.y = p[1]
        self.next = None
        self.expanded = False

    def length(self):
        return norm([self.x-self.next.x, self.y-self.next.y])
        
    def expand(self):
        print "EXPAND VERTICES VARIABLE CONSTRAINTS"
        v1 = self
        v2 = self.next
        direction = array([v2.y - v1.y, v1.x - v2.x])  # orthogonal direction to edge
        direction = 1 / norm(direction) * direction
        #print "Current vertices V1: ", v1.x, v1.y        
        #print "Current vertices V2: ", v2.x, v2.y        
        #print "New direction ", direction
        try:
            comWorldFrame = np.array([v1.x, v1.y, 0.0])
            #print comWorldFrame
            #comWorldFrame = np.array([0.0, 0.0, 0.0])
            z, error = optimize_direction_variable_constraint(comWorldFrame, direction)
            print 'error: ', error
        except ValueError:
            self.expanded = True
            return False, None
        xopt, yopt = z
        delta_area = cross([xopt-v1.x, yopt-v1.y], [v1.x-v2.x, v1.y-v2.y])
        delta_area_abs = abs(cross([xopt-v1.x, yopt-v1.y], [v1.x-v2.x, v1.y-v2.y]))
        print "Area: ",delta_area
        if delta_area_abs < 1e-3:
            self.expanded = True
            print "the area is small enough so the vertex expansion is over"
            return False, None
        else:
            vnew = VertexVariableConstraints([xopt, yopt])
            vnew.next = self.next
            self.next = vnew
            self.expanded = False
            return True, vnew
            
class PolygonVariableConstraint():
    
    def from_vertices(self, v1, v2, v3):
        v1.next = v2
        v2.next = v3
        v3.next = v1
        self.vertices = [v1, v2, v3]

    def all_expanded(self):
        for v in self.vertices:
            if not v.expanded:
                return False
        return True
        
    def iter_expand(self, max_iter):
        """
        Returns true if there's a edge that can be expanded, and expands that
        edge, otherwise returns False.
        """
        print "START expansion"
        nb_iter = 0
        v = self.vertices[0]
        while not self.all_expanded() and nb_iter < max_iter:
            print "iter expand"
            if v.expanded:
                v = v.next
                continue
            res, vnew = v.expand()
            if not res:
                continue
            self.vertices.append(vnew)
            nb_iter += 1
        print "STOP expansion"
        
    def sort_vertices(self):
        """
        Export vertices starting from the left-most and going clockwise.
        Assumes all vertices are on the positive halfplane.
        """
        minsd = 1e10
        ibottom = 0
        for i in range(len(self.vertices)):
            v = self.vertices[i]
            if (v.y + v.next.y) < minsd:
                ibottom = i
                minsd = v.y + v.next.y
        for v in self.vertices:
            v.checked = False
        vcur = self.vertices[ibottom]
        newvertices = []
        while not vcur.checked:
            vcur.checked = True
            newvertices.append(vcur)
            vcur = vcur.next
        newvertices.reverse()
        vfirst = newvertices.pop(-1)
        newvertices.insert(0, vfirst)
        self.vertices = newvertices

    def export_vertices(self, threshold=1e-2):
        export_list = [self.vertices[0]]
        for i in range(1, len(self.vertices)-1):
            vcur = self.vertices[i]
            vlast = export_list[-1]
            if norm([vcur.x-vlast.x, vcur.y-vlast.y]) > threshold:
                export_list.append(vcur)
        export_list.append(self.vertices[-1])  # always add last vertex
        return export_list
        
class IterativeProjection:
        
    def setup_iterative_projection(self, contacts, comWF, trunk_mass, mu, normals):
        ''' parameters to be tuned'''
        g = 9.81
        isOutOfWorkspace = False;

        grav = array([0., 0., -g])
        contactsNumber = np.size(contacts,0)
        # Unprojected state is:
        #
        #     x = [f1_x, f1_y, f1_z, ... , f3_x, f3_y, f3_z]
        Ex = np.zeros((0)) 
        Ey = np.zeros((0))        
        G = np.zeros((6,0))   
        for j in range(0,contactsNumber):
            r = contacts[j,:]
            graspMatrix = compDyn.getGraspMatrix(r)[:,0:3]
            Ex = hstack([Ex, -graspMatrix[4]])
            Ey = hstack([Ey, graspMatrix[3]])
            G = hstack([G, graspMatrix])            
            
        E = vstack((Ex, Ey)) / (g)
        f = zeros(2)
        proj = (E, f)  # y = E * x + f
        
        # number of the equality constraints
        m_eq = 6
        
        # see Equation (52) in "ZMP Support Areas for Multicontact..."
        A_f_and_tauz = array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1]])
        A = dot(A_f_and_tauz, G)
        t = hstack([0, 0, g, 0])
        #print A,t
        eq = (A, t)  # A * x == t
        
        act_LF = np.zeros((0,1))
        act_RF = np.zeros((0,1))
        act_LH = np.zeros((0,1))
        act_RH = np.zeros((0,1))
        actuation_polygons = np.zeros((0,1))
        # Inequality matrix for a contact force in local contact frame:
        constr = Constraints()
        #C_force = constr.linearized_cone_halfspaces(ng, mu)
        # Inequality matrix for stacked contact forces in world frame:
        if constraint_mode == 'ONLY_FRICTION':
            C, d = constr.linearized_cone_halfspaces_world(contactsNumber, ng, mu, normals)
            
        elif constraint_mode == 'ONLY_ACTUATION':
            #kin = Kinematics()
            kin = HyQKinematics()
            foot_vel = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]])
            contactsFourLegs = np.vstack([contacts, np.zeros((4-contactsNumber,3))])
            q, q_dot, J_LF, J_RF, J_LH, J_RH, isOutOfWorkspace = kin.inverse_kin(np.transpose(contactsFourLegs[:,0] - comWF[0]),
                                                  np.transpose(foot_vel[:,0]),
                                                    np.transpose(contactsFourLegs[:,1] - comWF[1]),
                                                    np.transpose(foot_vel[:,1]),
                                                    np.transpose(contactsFourLegs[:,2] - comWF[2]),
                                                    np.transpose(foot_vel[:,2]))
            J_LF, J_RF, J_LH, J_RH = kin.update_jacobians(q)

            if isOutOfWorkspace:
                C = np.zeros((0,0))
                d = np.zeros((1,0))
            else:
                act_LF = constr.computeLegActuationPolygon(J_LF)
                act_RF = constr.computeLegActuationPolygon(J_RF)
                act_LH = constr.computeLegActuationPolygon(J_LH)
                act_RH = constr.computeLegActuationPolygon(J_RH)            
                ''' in the case of the IP alg. the contact force limits must be divided by the mass
                because the gravito inertial wrench is normalized'''
                
                C = np.zeros((0,0))
                d = np.zeros((1,0))
                actuation_polygons = np.array([act_LF,act_RF,act_LH,act_RH])
                for j in range (0,contactsNumber):
                    hexahedronHalfSpaceConstraints, knownTerm = constr.hexahedron(actuation_polygons[j]/trunk_mass)
                    C = block_diag(C, hexahedronHalfSpaceConstraints)
                    d = hstack([d, knownTerm.T])
                    
                d = d.reshape(6*contactsNumber)    
                #C = block_diag(c1, c2, c3, c4)
                #d = np.vstack([e1, e2, e3, e4]).reshape(6*4)
                #print C, d
        
        ineq = (C, d)  # C * x <= d
        
        if isOutOfWorkspace:
            lp = 0
        else:
            max_radius=1e5
            (E, f), (A, b), (C, d) = proj, ineq, eq
            assert E.shape[0] == f.shape[0] == 2
            # Inequality constraints: A_ext * [ x  u  v ] <= b_ext iff
            # (1) A * x <= b and (2) |u|, |v| <= max_radius
            A_ext = zeros((A.shape[0] + 4, A.shape[1] + 2))
            A_ext[:-4, :-2] = A
            A_ext[-4, -2] = 1
            A_ext[-3, -2] = -1
            A_ext[-2, -1] = 1
            A_ext[-1, -1] = -1
            A_ext = cvxopt.matrix(A_ext)
            
            b_ext = zeros(b.shape[0] + 4)
            b_ext[:-4] = b
            b_ext[-4:] = array([max_radius] * 4)
            b_ext = cvxopt.matrix(b_ext)
            
            # Equality constraints: C_ext * [ x  u  v ] == d_ext iff
            # (1) C * x == d and (2) [ u  v ] == E * x + f
            C_ext = zeros((C.shape[0] + 2, C.shape[1] + 2))
            C_ext[:-2, :-2] = C
            C_ext[-2:, :-2] = E[:2]
            C_ext[-2:, -2:] = array([[-1, 0], [0, -1]])
            C_ext = cvxopt.matrix(C_ext)
            
            d_ext = zeros(d.shape[0] + 2)
            d_ext[:-2] = d
            d_ext[-2:] = -f[:2]
            d_ext = cvxopt.matrix(d_ext)
            
            lp_obj = cvxopt.matrix(zeros(A.shape[1] + 2))
            lp = lp_obj, A_ext, b_ext, C_ext, d_ext
        
        return lp, actuation_polygons/trunk_mass, isOutOfWorkspace


def optimize_direction_variable_constraint(comWorld, vdir, solver=GLPK_IF_AVAILABLE):
    print 'I am hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee'
    """
    Optimize in one direction.

    Parameters
    ----------
    vdir : (3,) array
        Direction in which the optimization is performed.
    lp : array tuple
        Tuple `(q, G, h, A, b)` defining the LP. See
        :func:`pypoman.lp..solve_lp` for details.
    solver : string, optional
        Backend LP solver to call.

    Returns
    -------
    succ : bool
        Success boolean.
    z : (3,) array, or 0
        Maximum vertex of the polygon in the direction `vdir`, or 0 in case of
        solver failure.
    """
    """ contact points """
    LF_foot = np.array([0.3, 0.3, -0.5])
    RF_foot = np.array([0.3, -0.3, -0.5])
    LH_foot = np.array([-0.3, 0.3, -0.5])
    RH_foot = np.array([-0.3, -0.2, -0.5])
    nc = 3
    numberOfGenerators = 4
    trunk_mass = 100
    mu = 0.8
    verbose = True
    

    contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
    contactsWorldFrame = contactsToStack[0:nc, :]
    axisZ= array([[0.0], [0.0], [1.0]])
    
    n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
    n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
    n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
    n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
    # %% Cell 2
    normals = np.vstack([n1, n2, n3, n4])
        
    iterProj = IterativeProjection()
    #comWorldFrame = np.array([0.0, 0.0, 0.0])
    #comZero = np.array([0.0, 0.0, 0.0])
    #comWorldFrame = np.array([0.28975694, 0.04463657, 0.0]) 

    optimalSolutionFound = True
    comp_dyn = ComputationalDynamics()
    iter = 0
    tol = 0.02
    errorNorm = np.zeros((0,1))
    
    while (optimalSolutionFound)&(iter<50):
        print "New tested CoM: ", comWorld
        iter += 1
        p, G, h, A, b, isConstraintOk, LP_actuation_polygons = comp_dyn.setup_lp(trunk_mass, contactsWorldFrame, nc, numberOfGenerators, normals, comWorld, constraint_mode, mu)

        if not isConstraintOk:
            #unfeasible_points = np.vstack([unfeasible_points, com_WF])
            if verbose:
                print 'something is wrong in the inequalities or the point is out of workspace'
        else:
            sol=solvers.lp(p, G, h, A, b)
            x = sol['x']
            status = sol['status']
            #print x
            if status == 'optimal':
                print "state is Feasible!"
                #feasible_points = np.vstack([feasible_points, com_WF])
                #contact_forces = np.vstack([contact_forces, np.transpose(x)])
            else:
                print "state is UNfeasible!"
                #unfeasible_points = np.vstack([unfeasible_points, com_WF])
                
        new_lp, actuation_polygons, isOutOfWorkspace = iterProj.setup_iterative_projection(contactsWorldFrame, comWorld, trunk_mass, mu, normals)
        if isOutOfWorkspace:
            optimalSolutionFound = False
            print "Point is out of workspace!"
        else:
            lp_q, lp_Gextended, lp_hextended, lp_A, lp_b = new_lp
            lp_q[-2] = -vdir[0]
            lp_q[-1] = -vdir[1]
            x = solve_lp(
                lp_q, lp_Gextended, lp_hextended, lp_A, lp_b, solver=solver)
            tempSolution = x[-2:]
            forces = x[0:10]
            print "New solution point: ",tempSolution
            #print forces
            #print actuation_polygons[0:3]
            #print "LP actuation polygons: ", LP_actuation_polygons[0:3]/trunk_mass
            delta_x = tempSolution[0] - comWorld[0]
            delta_y = tempSolution[1] - comWorld[1]
            tempSolutionNorm = norm(tempSolution)
            comWorldFrameNorm = norm(comWorld[0:2])
            errorNorm = np.vstack([errorNorm, tempSolutionNorm - comWorldFrameNorm])
            print 'eer', tempSolutionNorm - comWorldFrameNorm
            #if (np.abs(tempSolution[0])<np.abs(comWorldFrame[0]))|(np.abs(tempSolution[1])<np.abs(comWorldFrame[1])):
            if (np.abs(tempSolutionNorm - comWorldFrameNorm) < tol):            
                optimalSolutionFound = False
            else:
                stepIncrement = 0.1
                comWorld[0] += delta_x*stepIncrement;
                comWorld[1] += delta_y*stepIncrement;
                
        
    #return tempSolution, errorNorm
    return comWorld[0:2], errorNorm


def optimize_angle_variable_constraint(comWorldFrame, theta, solver=GLPK_IF_AVAILABLE):

    """
    Optimize in one direction.

    Parameters
    ----------
    theta : scalar
        Angle of the direction in which the optimization is performed.
    lp : array tuple
        Tuple `(q, G, h, A, b)` defining the LP. See
        :func:`pypoman.lp..solve_lp` for details.
    solver : string, optional
        Backend LP solver to call.

    Returns
    -------
    succ : bool
        Success boolean.
    z : (3,) array, or 0
        Maximum vertex of the polygon in the direction `vdir`, or 0 in case of
        solver failure.
    """
    print "Optimize angle!!!!!!!!!!!!!!!!!!!!!!"
    d = array([cos(theta), sin(theta)])
    z, error = optimize_direction_variable_constraint(comWorldFrame, d, solver=solver)
    return z


def compute_polygon_variable_constraint(max_iter=1000, solver=GLPK_IF_AVAILABLE):
    """
    Expand a polygon iteratively.

    Parameters
    ----------
    lp : array tuple
        Tuple `(q, G, h, A, b)` defining the linear program. See
        :func:`pypoman.lp.solve_lp` for details.
    max_iter : integer, optional
        Maximum number of calls to the LP solver.
    solver : string, optional
        Name of backend LP solver.

    Returns
    -------
    poly : Polygon
        Output polygon.
    """
    two_pi = 2 * pi
    theta = pi * random()
    comWorldFrame = np.array([0.0, 0.0, 0.0])
    init_vertices = [optimize_angle_variable_constraint(comWorldFrame, theta, solver)]
    step = two_pi / 3
    while len(init_vertices) < 3 and max_iter >= 0:
        theta += step
        if theta >= two_pi:
            #step *= 0.25 + 0.5 * random()
            theta += step - two_pi
        comWorldFrame = np.array([0.0, 0.0, 0.0])
        z = optimize_angle_variable_constraint(comWorldFrame, theta, solver)
        if all([norm(z - z0) > 1e-5 for z0 in init_vertices]):
            init_vertices.append(z)
        max_iter -= 1
    if len(init_vertices) < 3:
        raise Exception("problem is not linearly feasible")
    v0 = VertexVariableConstraints(init_vertices[0])
    v1 = VertexVariableConstraints(init_vertices[1])
    v2 = VertexVariableConstraints(init_vertices[2])
    polygon = PolygonVariableConstraint()
    polygon.from_vertices(v0, v1, v2)
    polygon.iter_expand(max_iter)
    return polygon
    
''' MAIN '''
start_t_IPVC = time.time()
math = Math()
compDyn = ComputationalDynamics()
# number of contacts
nc = 3
# number of generators, i.e. rays used to linearize the friction cone
ng = 4

# ONLY_ACTUATION or ONLY_FRICTION
constraint_mode = 'ONLY_ACTUATION'
useVariableJacobian = True
trunk_mass = 100
mu = 0.8
# number of decision variables of the problem
n = nc*6

""" contact points """
LF_foot = np.array([0.3, 0.3, -0.5])
RF_foot = np.array([0.3, -0.3, -0.5])
LH_foot = np.array([-0.3, 0.3, -0.5])
RH_foot = np.array([-0.3, -0.2, -0.5])

contactsToStack = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
contacts = contactsToStack[0:nc, :]

iterProj = IterativeProjection()
comWF = np.array([0.0, 0.0, 0.0])
#lp, actuation_polygons, isOutOfWorkspace = iterProj.setup_iterative_projection(contacts, comWF, trunk_mass, mu)
#print "is out of workspace? ", isOutOfWorkspace

polygon = compute_polygon_variable_constraint()
polygon.sort_vertices()
vertices_list = polygon.export_vertices()
vertices = [array([v.x, v.y]) for v in vertices_list]
print "Final vertices: ",vertices
print("Iterative Projection (Bretl): --- %s seconds ---" % (time.time() - start_t_IPVC))


''' plotting Iterative Projection points '''

axisZ= array([[0.0], [0.0], [1.0]])
n1 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n2 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n3 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
n4 = np.transpose(np.transpose(math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2
        
normals = np.vstack([n1, n2, n3, n4])
IP_points, actuation_polygons = compDyn.iterative_projection_bretl(constraint_mode, contacts, normals, trunk_mass, ng, mu)

feasible, unfeasible, contact_forces = compDyn.LP_projection(constraint_mode, contacts, normals, trunk_mass, mu, ng, nc, mu, useVariableJacobian, 0.025, 0.025)

#IP_points_saturated_friction, actuation_polygons = compDyn.iterative_projection_bretl('ONLY_FRICTION', contacts, normals, trunk_mass, ng, mu, saturate_normal_force = True)

'''Plotting'''
#plt.close('all')
plotter = Plotter()

plt.figure()
plt.grid()
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
h1 = plt.plot(contacts[0:nc,0],contacts[0:nc,1],'ko',markersize=15, label='feet')
vx = np.asanyarray(vertices)
plotter.plot_polygon(vx, color = 'y')

plotter.plot_polygon(np.transpose(IP_points))

#plotter.plot_polygon(np.transpose(IP_points_saturated_friction), color = '--r')

feasiblePointsSize = np.size(feasible,0)
for j in range(0, feasiblePointsSize):
    if (feasible[j,2]<0.01)&(feasible[j,2]>-0.01):
        plt.scatter(feasible[j,0], feasible[j,1],c='g',s=50)
        lastFeasibleIndex = j
unfeasiblePointsSize = np.size(unfeasible,0)

for j in range(0, unfeasiblePointsSize):
    if (unfeasible[j,2]<0.01)&(unfeasible[j,2]>-0.01):
        plt.scatter(unfeasible[j,0], unfeasible[j,1],c='r',s=50)
        lastUnfeasibleIndex = j
h2 = plt.scatter(feasible[lastFeasibleIndex,0], feasible[lastFeasibleIndex,1],c='g',s=50, label='LP feasible')
h3 = plt.scatter(unfeasible[lastUnfeasibleIndex,0], unfeasible[lastUnfeasibleIndex,1],c='r',s=50, label='LP unfeasible')


print "final vertices: ", vx
print "number of vertices: ", np.size(vx, 0)
plt.legend()
plt.show()

