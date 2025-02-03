# -*- coding: utf-8 -*-
"""
Created on Sun Aug  5 19:11:34 2018

@author: Romeo Orsolino
"""

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import random as rnd


class BilinearConstraints:
    def make_convex(self, value):
        resolution = 0.1
        # Make data.
        x = np.arange(-5.0, 5.0, resolution)
        f = np.arange(-5.0, 5.0, resolution)
        X, F = np.meshgrid(x, f)
        self.Tau = X*F
        
        p = X+F
        q = X-F
        p_hat = p*p
        q_hat = q*q
        Conv_plus = 0.25*np.power(p,2.0)
        Conv_minus = -0.25*np.power(q,2.0)
        
        Tau_approx = Conv_plus + Conv_minus
        
        sigma = 0.0
        p_hat_relaxation = value*value +2.0*value*(p - value) + sigma
        
        return X, F, p_hat, p_hat_relaxation, q_hat
        
    def get_torque(self):
        return self.Tau