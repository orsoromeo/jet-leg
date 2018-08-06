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

import time
from termcolor import colored
import numpy as np
import scipy.sparse as sps
import ipopt
        
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
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #return np.array([2.0*x[0]-5.0 , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    def constraints(self, x):
        #
        # The callback for calculating the constraints
        #
        pos = x[0:3]
        force = x[3:6]
        p_x = pos[0] + force[0]
        p_hat_x = x[6]
        quadratic_inequality1 = p_hat_x - np.dot(p_x, p_x)
        
        q_x = pos[0] - force[0]
        q_hat_x = x[7]
        quadratic_inequality2 = q_hat_x - np.dot(q_x, q_x)
        
        kx = 0.25*(p_hat_x - q_hat_x)

        sigma = 1.0

        p_value_x = np.add(self.initial_guess[0], self.initial_guess[3])
        diff_x = np.subtract(p_x, p_value_x)
        linear_ineq1 = np.dot(p_value_x, p_value_x) + 2.0*np.dot(p_value_x, diff_x) + sigma - p_hat_x

        q_value_x = np.subtract(self.initial_guess[0], self.initial_guess[3])
        diff_x = np.subtract(q_x, q_value_x)
        linear_ineq2 = np.dot(q_value_x, q_value_x) + 2.0*np.dot(q_value_x, diff_x) + sigma - q_hat_x
       
        return np.array((kx, quadratic_inequality1, quadratic_inequality2, linear_ineq1, linear_ineq2))
    
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
    x0 = [2, 0, 0, 4, 0, 0, 40.0, 10.0]
    
    lb = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0,-1000.0, -1000.0]
    ub = [3.0, 3.0, 3.0, 20.0, 20.0, 20.0, 1000.0, 1000.0]
    
    cl = [6.0, 0.0, 0.0, 0.0, 0.0]
    cu = [6.0, +2e19, +2e19, +2e19, +2e19]
    
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
    nlpConvex.addOption('tol', 1e-6)
    #nlpConvex.addOption('acceptable_tol', 1e-9)
    nlpConvex.addOption('dual_inf_tol', 1e-6)
    nlpConvex.addOption('constr_viol_tol', 1e-6)
    nlpConvex.addOption('compl_inf_tol', 1e-6)
    #
    # Scale the problem (Just for demonstration purposes)
    #
    nlpConvex.setProblemScaling(
        obj_scaling=1,
        x_scaling=[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
        )
    nlpConvex.addOption('nlp_scaling_method', 'user-scaling')
    
    #
    # Solve the problem
    #
    x, info = nlpConvex.solve(x0)
    
    print "Solution of the primal variables: x=%s\n" % repr(x)
    
    print "Solution of the dual variables: lambda=%s\n" % repr(info['mult_g'])
    
    print "Objective=%s\n" % repr(info['obj_val'])
    
    print colored('torque check: ', 'red'),  - x[2]*x[4] + x[1]*x[5], + x[2]*x[3] - x[0]*x[5], - x[1]*x[3] + x[0]*x[4] 
    
    
def main():
    
    start_cv_hard_time = time.time()
    print colored('Convex Formulation HARD constraints', 'blue')
    convex_formulation_hard_constraints()
    print("Directed Iterative Projection: --- %s seconds ---" % (time.time() - start_cv_hard_time))
    


if __name__ == '__main__':
    main()
