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

import numpy as np
import scipy.sparse as sps
import ipopt

class ConvexFormulationHardConstraint(object):
    def __init__(self):
        pass
    
    def objective(self, x):
        #
        # The callback for calculating the objective
        #
        return np.power(x[0]-1.5, 2.0)
        #return 0.0        
        
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        #
        #return np.array([0.0,0.0])
        return np.array([2.0*x[0]-2.0 , 0.0])
    
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
        
class bilinearFormulation(object):
    def __init__(self):
        pass
    
    def objective(self, x):
        #
        # The callback for calculating the objective
        #
        return np.power(x[0]-2.0, 2.0)
        #return 0.0        
        
    def gradient(self, x):
        #
        # The callback for calculating the gradient
        #
        #return np.array([0.0,0.0])
        return np.array([2.0*x[0] - 4.0, 0.0])
    
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
    
    
def main():
    #
    # Define the problem
    #
    x0 = [0.0, 4.0]
    
    lb = [1.0, 0.0]
    ub = [3.0, 20.0]
    
    cl = [6.0]
    cu = [6.0]

    nlpBilinear = ipopt.problem(
                n=len(x0),
                m=len(cl),
                problem_obj=bilinearFormulation(),
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
        x_scaling=[1, 1]
        )
    nlpBilinear.addOption('nlp_scaling_method', 'user-scaling')
    
    #
    # Solve the problem
    #
    x, info = nlpBilinear.solve(x0)
    
    print "Solution of the primal variables: x=%s\n" % repr(x)
    
    print "Solution of the dual variables: lambda=%s\n" % repr(info['mult_g'])
    
    print "Objective=%s\n" % repr(info['obj_val'])


if __name__ == '__main__':
    main()
