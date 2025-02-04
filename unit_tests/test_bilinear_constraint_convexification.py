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

class buildConvexFormulationSoftConstraint(object):
    def __init__(self, x0_init):
        self.initial_guess = x0_init
        pass
    
    def objective(self, x):
        #
        # The callback for calculating the objective
        #
        p = x[0] + x[1]
        p_hat = x[2]
        
        q = x[0] - x[1]
        q_hat = x[3]

        p_value = self.initial_guess[0] + self.initial_guess[1]     
        q_value = self.initial_guess[0] - self.initial_guess[1]
        
        sigma_p = p_hat - (p_value*p_value +2.0*p_value*(p - p_value))
        sigma_q = q_hat - (q_value*q_value +2.0*q_value*(q - q_value))
        
        #np.power(x[0]-2.5, 2.0)
        return np.power(sigma_p, 2.0) + np.power(sigma_q, 2.0)
        #return 0.0        
        
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        #
        p_value = self.initial_guess[0] + self.initial_guess[1]     
        q_value = self.initial_guess[0] - self.initial_guess[1]
        
        f_grad1 = 2.0*x[0]-5.0
        
        dsp2_dx0 = -4.0*x[2]*p_value + 8.0*p_value*p_value*(x[0] + x[1] - p_value) + 4.0*p_value*p_value*p_value
        dsp2_dx1 = -4.0*x[2]*p_value + 8.0*p_value*p_value*(x[0] + x[1] - p_value) + 4.0*p_value*p_value*p_value
        dsp2_dx2 = 2.0*x[2] - 2.0*p_value*p_value - 4.0*p_value*(x[0] + x[1] - p_value)
        dsp2_dx3 = 0.0        
        
        dsq2_dx0 = -4.0*x[3]*q_value + 8.0*q_value*q_value*(x[0] - x[1] - q_value) + 4.0*q_value*q_value*q_value
        dsq2_dx1 = -4.0*x[3]*q_value + 8.0*q_value*q_value*(x[0] - x[1] - q_value) + 4.0*q_value*q_value*q_value
        dsq2_dx2 = 0.0
        dsq2_dx3 = 2.0*x[3] - 2.0*q_value*q_value - 4.0*q_value*(x[0] - x[1] - q_value)
        #return np.array([0.0,0.0, 0.0, 0.0])
        df_dx0 = dsp2_dx0 + dsq2_dx0 
        df_dx1 = dsp2_dx1 + dsq2_dx1
        df_dx2 = dsp2_dx2 + dsq2_dx2
        df_dx3 = dsp2_dx3 + dsq2_dx3
        #return np.array([0.0,0.0, 0.0, 0.0])
        return np.array([df_dx0, df_dx1, df_dx2, df_dx3])
    
    def constraints(self, x):
        #
        # The callback for calculating the constraints
        #
        p = x[0] + x[1]
        p_hat = x[2]
        quadratic_inequality1 = p_hat - np.power(p,2.0)
        
        q = x[0] - x[1]
        q_hat = x[3]
        quadratic_inequality2 = q_hat - np.power(q, 2.0)
        
        kx = 0.25*(p_hat - q_hat)
            
        return np.array((kx, quadratic_inequality1, quadratic_inequality2))
    
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
        return np.power(x[0]-2.5, 2.0)
        #return 0.0        
        
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        #
        #return np.array([0.0,0.0, 0.0, 0.0])
        return np.array([2.0*x[0]-5.0 , 0.0, 0.0, 0.0])
    
    def constraints(self, x):
        #
        # The callback for calculating the constraints
        #
        p = x[0] + x[1]
        p_hat = x[2]
        quadratic_inequality1 = p_hat - np.power(p,2.0)
        
        q = x[0] - x[1]
        q_hat = x[3]
        quadratic_inequality2 = q_hat - np.power(q, 2.0)
        
        kx = 0.25*(p_hat - q_hat)
    
        sigma = 1.0
        p_value = self.initial_guess[0] + self.initial_guess[1] #quadratic_inequality1
        linear_ineq1 = p_value*p_value +2.0*p_value*(p - p_value) + sigma - p_hat
        
        q_value = self.initial_guess[0] - self.initial_guess[1]#quadratic_inequality2
        linear_ineq2 = q_value*q_value +2.0*q_value*(q - q_value) + sigma - q_hat
        
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
        
class buildBilinearFormulation(object):
    def __init__(self):
        pass
    
    def objective(self, x):
        #
        # The callback for calculating the objective
        #
        return np.power(x[0]-2.5, 2.0)
        #return 0.0        
        
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        #
        #return np.array([0.0,0.0])
        return np.array([2.0*x[0] - 5.0, 0.0])
    
    def constraints(self, x):
        #
        # The callback for calculating the constraints
        #
        return np.array((x[0]*x[1]))
    
    def jacobian(self, x):
        #
        # The callback for calculating the Jacobian
        #
        return np.array([x[1], x[0]])
        #return False
    
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
        return False#H[hs.row, hs.col]

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
def convex_formulation_soft_constraints():
    ''' Definition the bilinear problem '''
    x0 = [3, 2, 40.0, 10.0]
    
    lb = [1.0, 0.0, -1000.0, -1000.0]
    ub = [3.0, 20.0, 1000.0, 1000.0]
    
    cl = [6.0, 0.0, 0.0]
    cu = [6.0, +2e19, +2e19]
    
    nlpConvex = ipopt.problem(
                n=len(x0),
                m=len(cl),
                problem_obj=buildConvexFormulationSoftConstraint(x0),
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
        x_scaling=[1.0,1.0,1.0,1.0]
        )
    nlpConvex.addOption('nlp_scaling_method', 'user-scaling')
    
    #
    # Solve the problem
    #
    x, info = nlpConvex.solve(x0)
    
    print "Solution of the primal variables: x=%s\n" % repr(x)
    
    print "Solution of the dual variables: lambda=%s\n" % repr(info['mult_g'])
    
    print "Objective=%s\n" % repr(info['obj_val'])
    
    print colored('torque check: ', 'red'), x[0]*x[1]
    
def convex_formulation_hard_constraints():
    ''' Definition the bilinear problem '''
    x0 = [2, 4, 40.0, 10.0]
    
    lb = [1.0, 0.0, -1000.0, -1000.0]
    ub = [3.0, 20.0, 1000.0, 1000.0]
    
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
        x_scaling=[1.0,1.0,1.0,1.0]
        )
    nlpConvex.addOption('nlp_scaling_method', 'user-scaling')
    
    #
    # Solve the problem
    #
    x, info = nlpConvex.solve(x0)
    
    print "Solution of the primal variables: x=%s\n" % repr(x)
    
    print "Solution of the dual variables: lambda=%s\n" % repr(info['mult_g'])
    
    print "Objective=%s\n" % repr(info['obj_val'])
    
    print colored('torque check: ', 'red'), x[0]*x[1]
    
def bilinear_formulation():
    ''' Definition the bilinear problem '''
    x0 = [0.0, 4.0]
    
    lb = [1.0, 0.0]
    ub = [3.0, 20.0]
    
    cl = [6.0]
    cu = [6.0]

    nlpBilinear = ipopt.problem(
                n=len(x0),
                m=len(cl),
                problem_obj=buildBilinearFormulation(),
                lb=lb,
                ub=ub,
                cl=cl,
                cu=cu
                )

    #
    # Set solver options
    #
    #nlp.addOption('derivative_test', 'second-order')
    nlpBilinear.addOption('mu_strategy', 'adaptive')
    #nlpBilinear.addOption('jacobian_approximation', 'finite-difference-values')
    nlpBilinear.addOption('hessian_approximation', 'limited-memory')    
    nlpBilinear.addOption('tol', 1e-6)
    #nlpBilinear.addOption('acceptable_tol', 1e-9)
    nlpBilinear.addOption('dual_inf_tol', 1e-6)
    nlpBilinear.addOption('constr_viol_tol', 1e-6)
    nlpBilinear.addOption('compl_inf_tol', 1e-6)
    #
    # Scale the problem (Just for demonstration purposes)
    #
    nlpBilinear.setProblemScaling(
        obj_scaling=1,
        x_scaling=[1.0,1.0]
        )
    nlpBilinear.addOption('nlp_scaling_method', 'user-scaling')
    
    #
    # Solve the problem
    #
    x, info = nlpBilinear.solve(x0)
    
    print "Solution of the primal variables: x=%s\n" % repr(x)
    
    print "Solution of the dual variables: lambda=%s\n" % repr(info['mult_g'])
    
    print "Objective=%s\n" % repr(info['obj_val'])
    
def main():
    #start_bilinear_time = time.time()
    #print colored('Bilinear Formulation', 'blue')
    #for i in range(0,100):
    #    bilinear_formulation()
    #print("Directed Iterative Projection: --- %s seconds ---" % (time.time() - start_bilinear_time))
    
    start_cv_hard_time = time.time()
    print colored('Convex Formulation HARD constraints', 'blue')
    for i in range(0,100):
        convex_formulation_hard_constraints()
    print("Directed Iterative Projection: --- %s seconds ---" % (time.time() - start_cv_hard_time))
    
    #start_cv_soft_time = time.time()
    #print colored('Convex Formulation SOFT constraints', 'blue')
    #for i in range(0,100):
    #    convex_formulation_soft_constraints()
    #print("Directed Iterative Projection: --- %s seconds ---" % (time.time() - start_cv_soft_time))
    


if __name__ == '__main__':
    main()
