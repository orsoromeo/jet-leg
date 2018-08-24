# -*- coding: utf-8 -*-
"""
Created on Sun Aug  5 19:50:44 2018

@author: Romeo Orsolino

cyipot: Python wrapper for the Ipopt optimization package, written in Cython.

Copyright (C) 2012 Amit Aides
Author: Amit Aides <amitibo@tx.technion.ac.il>
URL: <http://http://code.google.com/p/cyipopt/>
License: EPL 1.0
"""
#
# Test the "ipopt" Python interface on the Hock & Schittkowski test problem
# #71. See: Willi Hock and Klaus Schittkowski. (1981) Test Examples for
# Nonlinear Programming Codes. Lecture Notes in Economics and Mathematical
# Systems Vol. 187, Springer-Verlag.
#
# Based on matlab code by Peter Carbonetto.
#

import context
import time
from termcolor import colored
import numpy as np
import scipy.sparse as sps
import ipopt
from jet_leg.math_tools import Math

class buildBilinearFormulation(object):
    def __init__(self, x0_init):
        self.initial_guess = x0_init
        pass
    
    def objective(self, x):
        #
        # The callback for calculating the objective
        #
        #return np.power(x[0]-2.5, 2.0)
        return 0.0        
        
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        #
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #return np.array([2.0*x[0]-5.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
    def constraints(self, x):
        #
        # The callback for calculating the constraints
        #
        math = Math()
        pos = x[0:3]
        force = x[3:6]
        tau_constraint = np.dot(math.skew(pos), force)
       
        return np.array((tau_constraint))
    
    def jacobian(self, x):
        #
        # The callback for calculating the Jacobian
        #
        #return np.array([x[1], x[0]])
        return False
    
    def hessianstructure(self):
        #
        # The structure of the Hessian
        # Note:
        # The default hessian structure is of a lower triangular matrix. Therefore
        # this function is redundant. I include it as an example for structure
        # callback.
        # 
        global hs
        
        hs = sps.coo_matrix(np.tril(np.ones((4, 4))))
        return (hs.col, hs.row)
    
    def hessian(self, x, lagrange, obj_factor):
        #
        # The callback for calculating the Hessian
        #
        H = obj_factor*np.array((
                (2*x[3], 0, 0, 0),
                (x[3],   0, 0, 0),
                (x[3],   0, 0, 0),
                (2*x[0]+x[1]+x[2], x[0], x[0], 0)))
                
        H += lagrange[0]*np.array((
                (0, 0, 0, 0),
                (x[2]*x[3], 0, 0, 0),
                (x[1]*x[3], x[0]*x[3], 0, 0),
                (x[1]*x[2], x[0]*x[2], x[0]*x[1], 0)))
                
        H += lagrange[1]*2*np.eye(4)
    
        #
        # Note:
        # 
        #
        return H[hs.row, hs.col]

    def intermediate(
            self, 
            alg_mod,
            iter_count,
            obj_value,
            inf_pr,
            inf_du,
            mu,
            d_norm,
            regularization_size,
            alpha_du,
            alpha_pr,
            ls_trials
            ):

        #
        # Example for the use of the intermediate callback.
        #
        print "Objective value at iteration #%d is - %g" % (iter_count, obj_value)
        
class buildConvexFormulationHardConstraint(object):
    def __init__(self, x0_init):
        self.initial_guess = x0_init
        pass
    
    def objective(self, x):
        #
        # The callback for calculating the objective
        #
        #return np.power(x[0]-2.5, 2.0)
        return 0.0        
        
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        #
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #return np.array([2.0*x[0]-5.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    def buildConvexConstraints(self, pos, force, p_hat, q_hat, p_value, q_value, sigma):
        
        p = np.add(pos, force)
        quadratic_inequality1 = p_hat - np.dot(p, p)
        
        q = np.subtract(pos, force)
        quadratic_inequality2 = q_hat - np.dot(q, q)
        
        kx = 0.25*(p_hat - q_hat)
        #print quadratic_inequality1, quadratic_inequality2, kx

        #p_value = np.add(self.initial_guess[0], self.initial_guess[3])
        diff = np.subtract(p, p_value)
        linear_ineq1 = np.dot(p_value, p_value) + 2.0*np.dot(p_value, diff) + sigma - p_hat

        #q_value = np.subtract(self.initial_guess[0], self.initial_guess[3])
        diff = np.subtract(q, q_value)
        linear_ineq2 = np.dot(q_value, q_value) + 2.0*np.dot(q_value, diff) + sigma - q_hat
        #print linear_ineq1, linear_ineq2
        return kx, quadratic_inequality1, quadratic_inequality2, linear_ineq1, linear_ineq2
        
    def constraints(self, x):
        #
        # The callback for calculating the constraints
        #
        
        pos_1 = np.array([-x[2], x[1]])
        force_1 = np.array([x[4], x[5]])
        p_hat_1 = x[6]
        q_hat_1 = x[9]
        p_value_1 = np.array([self.initial_guess[4] - self.initial_guess[2], self.initial_guess[1] + self.initial_guess[5]])
        q_value_1 = np.array([self.initial_guess[4] + self.initial_guess[2], self.initial_guess[1] - self.initial_guess[5]])  
        sigma_1 = 1000    
        tau_x_contraints = self.buildConvexConstraints(pos_1, force_1, p_hat_1, q_hat_1, p_value_1, q_value_1, sigma_1)
        
        pos_2 = np.array([x[2], -x[0]])
        force_2 = np.array([x[3], x[5]])
        p_hat_2 = x[7]
        q_hat_2 = x[10]
        p_value_2 = np.array([self.initial_guess[2] + self.initial_guess[3], - self.initial_guess[0] + self.initial_guess[5]])
        q_value_2 = np.array([self.initial_guess[2] - self.initial_guess[3], - self.initial_guess[0] - self.initial_guess[5]])  
        sigma_2 = 100
        tau_y_contraints = self.buildConvexConstraints(pos_2, force_2, p_hat_2, q_hat_2, p_value_2, q_value_2, sigma_2)
       
        pos_3 = np.array([-x[1], x[0]])
        force_3 = np.array([x[3], x[4]])
        p_hat_3 = x[8]
        q_hat_3 = x[11]
        p_value_3 = np.array([ - self.initial_guess[1] + self.initial_guess[3], self.initial_guess[0] + self.initial_guess[4]])
        q_value_3 = np.array([ - self.initial_guess[1] - self.initial_guess[3], self.initial_guess[0] - self.initial_guess[4]])  
        sigma_3 = 50
        tau_z_contraints = self.buildConvexConstraints(pos_3, force_3, p_hat_3, q_hat_3, p_value_3, q_value_3, sigma_3)
       
        return np.array((tau_x_contraints, tau_y_contraints, tau_z_contraints))
    
    def jacobian(self, x):
        #
        # The callback for calculating the Jacobian
        #
        #return np.array([x[1], x[0]])
        return False
    
    def hessianstructure(self):
        #
        # The structure of the Hessian
        # Note:
        # The default hessian structure is of a lower triangular matrix. Therefore
        # this function is redundant. I include it as an example for structure
        # callback.
        # 
        global hs
        
        hs = sps.coo_matrix(np.tril(np.ones((4, 4))))
        return (hs.col, hs.row)
    
    def hessian(self, x, lagrange, obj_factor):
        #
        # The callback for calculating the Hessian
        #
        H = obj_factor*np.array((
                (2*x[3], 0, 0, 0),
                (x[3],   0, 0, 0),
                (x[3],   0, 0, 0),
                (2*x[0]+x[1]+x[2], x[0], x[0], 0)))
                
        H += lagrange[0]*np.array((
                (0, 0, 0, 0),
                (x[2]*x[3], 0, 0, 0),
                (x[1]*x[3], x[0]*x[3], 0, 0),
                (x[1]*x[2], x[0]*x[2], x[0]*x[1], 0)))
                
        H += lagrange[1]*2*np.eye(4)
    
        #
        # Note:
        # 
        #
        return H[hs.row, hs.col]

    def intermediate(
            self, 
            alg_mod,
            iter_count,
            obj_value,
            inf_pr,
            inf_du,
            mu,
            d_norm,
            regularization_size,
            alpha_du,
            alpha_pr,
            ls_trials
            ):

        #
        # Example for the use of the intermediate callback.
        #
        print "Objective value at iteration #%d is - %g" % (iter_count, obj_value)
        
    
def convex_formulation_hard_constraints():
    ''' Definition the bilinear problem '''
    
    math = Math()
    
    pos0 = np.array([2.5, 2.5, 2.5])
    force0 = np.array([0.0, 0.0, 10.0])

    #pos0 = np.array([2.5, 2.5, 2.5])
    #force0 = np.array([-10.0, 10.0, 10.0])
    
    posDesired = np.array([2.0, 2.0, 2.0])
    forceDesired = np.array([0.0, 0.0, 10.0])    
    tauDesired = np.dot(math.skew(posDesired), forceDesired)  
    print "initial contact position: " ,pos0
    print "initial force: " ,force0
    print "Desired torque: " ,tauDesired
    p0 = np.add(pos0 ,force0)
    q0 = np.subtract(pos0, force0)
    
    p_hat0 = np.array([p0[0]*p0[0], p0[1]*p0[1], p0[2]*p0[2]])
    q_hat0 = np.array([q0[0]*q0[0], q0[1]*q0[1], q0[2]*q0[2]])
    
    x0 = np.hstack([pos0, force0, p_hat0, q_hat0])
    #x0 = [2, 0, 0, 4, 0, 0, 40.0, 40.0, 40.0, 10.0, 10.0, 10.0]
    
    lb = [1.0, 1.0, 1.0, -20.0, -20.0, 0.0,-1000.0, -1000.0, -1000.0, -1000.0, -1000.0, -1000.0]
    ub = [3.0, 3.0, 3.0, 20.0, 20.0, 20.0, +1000.0, +1000.0, +1000.0, +1000.0, +1000.0, +1000.0]
   
    tau_x_constraints_lb = np.array([20.0, 0.0, 0.0, 0.0, 0.0])
    tau_x_constraints_ub = np.array([20.0, +2e19, +2e19, +2e19, +2e19]) 
    
    tau_y_constraints_lb = np.array([-20.0, 0.0, 0.0, 0.0, 0.0])
    tau_y_constraints_ub = np.array([-20.0, +2e19, +2e19, +2e19, +2e19])

    tau_z_constraints_lb = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    tau_z_constraints_ub = np.array([0.0, +2e19, +2e19, +2e19, +2e19])
    
    cl = np.hstack([tau_x_constraints_lb, tau_y_constraints_lb, tau_z_constraints_lb])
    cu = np.hstack([tau_x_constraints_ub, tau_y_constraints_ub, tau_z_constraints_ub])
    
    nlpConvex = ipopt.problem(
                n=len(x0),
                m=len(cl),
                problem_obj=buildConvexFormulationHardConstraint(x0),
                lb=lb,
                ub=ub,
                cl=cl,
                cu=cu
                )

    #
    # Set solver options
    #
    #nlp.addOption('derivative_test', 'second-order')
    nlpConvex.addOption('mu_strategy', 'adaptive')
    nlpConvex.addOption('jacobian_approximation', 'finite-difference-values')
    nlpConvex.addOption('hessian_approximation', 'limited-memory')    
    nlpConvex.addOption('tol', 1e-2)

    #
    # Scale the problem (Just for demonstration purposes)
    #
    nlpConvex.setProblemScaling(
        obj_scaling=1,
        x_scaling=[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
        )
    nlpConvex.addOption('nlp_scaling_method', 'user-scaling')
    
    #
    # Solve the problem
    #
    sol, info = nlpConvex.solve(x0)
    
    print "Solution of the primal variables: x=%s\n" % repr(sol)
    
    print "Solution of the dual variables: lambda=%s\n" % repr(info['mult_g'])
    
    print "Objective=%s\n" % repr(info['obj_val'])
    l = sol[0:3]
    f = sol[3:6]
    tau = np.dot(math.skew(l), f)
    print colored('-----------> Results check: ', 'red')
    print colored('desired torque: ', 'blue'),  tauDesired[0], tauDesired[1], tauDesired[2]     
    print colored('actual torque: ', 'green'),  tau
    print colored('torque error: ', 'red'),  np.subtract(tau, tauDesired)
    print colored('foot pos: ', 'green'), l
    print colored('force: ', 'green'), f

def bilinear_formulation():
    ''' Definition the bilinear problem '''
    
    math = Math()
    
    pos0 = np.array([2.5, 2.5, 2.5])
    force0 = np.array([-10.0, 10.0, 10.0])

    posDesired = np.array([2.0, 2.0, 2.0])
    forceDesired = np.array([0.0, 0.0, 10.0])    
    tauDesired = np.dot(math.skew(posDesired), forceDesired)  
    print "initial contact position: " ,pos0
    print "initial force: " ,force0
    print "Desired torque: " ,tauDesired
    
    x0 = np.hstack([pos0, force0])
    
    lb = [1.0, 1.0, 1.0, -20.0, -20.0, 0.0]
    ub = [3.0, 3.0, 3.0, 20.0, 20.0, 20.0]
   
    tau_x_constraints_lb = np.array([20.0])
    tau_x_constraints_ub = np.array([20.0]) 
    
    tau_y_constraints_lb = np.array([-20.0])
    tau_y_constraints_ub = np.array([-20.0])

    tau_z_constraints_lb = np.array([0.0])
    tau_z_constraints_ub = np.array([0.0])
    
    cl = np.hstack([tau_x_constraints_lb, tau_y_constraints_lb, tau_z_constraints_lb])
    cu = np.hstack([tau_x_constraints_ub, tau_y_constraints_ub, tau_z_constraints_ub])
    
    nlpConvex = ipopt.problem(
                n=len(x0),
                m=len(cl),
                problem_obj=buildBilinearFormulation(x0),
                lb=lb,
                ub=ub,
                cl=cl,
                cu=cu
                )

    #
    # Set solver options
    #
    #nlp.addOption('derivative_test', 'second-order')
    nlpConvex.addOption('mu_strategy', 'adaptive')
    nlpConvex.addOption('jacobian_approximation', 'finite-difference-values')
    nlpConvex.addOption('hessian_approximation', 'limited-memory')    
    nlpConvex.addOption('tol', 1e-2)

    #
    # Scale the problem (Just for demonstration purposes)
    #
    nlpConvex.setProblemScaling(
        obj_scaling=1,
        x_scaling=[1.0,1.0,1.0,1.0,1.0,1.0]
        )
    nlpConvex.addOption('nlp_scaling_method', 'user-scaling')
    
    #
    # Solve the problem
    #
    sol, info = nlpConvex.solve(x0)
    
    print "Solution of the primal variables: x=%s\n" % repr(sol)
    
    print "Solution of the dual variables: lambda=%s\n" % repr(info['mult_g'])
    
    print "Objective=%s\n" % repr(info['obj_val'])
    l = sol[0:3]
    f = sol[3:6]
    tau = np.dot(math.skew(l), f)
    print colored('-----------> Results check: ', 'red')
    print colored('desired torque: ', 'blue'),  tauDesired[0], tauDesired[1], tauDesired[2]     
    print colored('actual torque: ', 'green'),  tau
    print colored('torque error: ', 'red'),  np.subtract(tau, tauDesired)
    print colored('foot pos: ', 'green'), l
    print colored('force: ', 'green'), f


def main():

    start_cv_hard_time = time.time()
    print colored('Bilinear Formulation', 'blue')
    bilinear_formulation()
    print("Total time: --- %s seconds ---" % (time.time() - start_cv_hard_time)) 
    
    start_cv_hard_time = time.time()
    print colored('Convex Formulation HARD constraints', 'blue')
    convex_formulation_hard_constraints()
    print("Total time: --- %s seconds ---" % (time.time() - start_cv_hard_time)) 


if __name__ == '__main__':
    main()
