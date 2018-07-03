# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 06:10:18 2018

@author: Romeo Orsolino
"""

from hyq_kinematics import HyQKinematics

hyqKin = HyQKinematics()

hyqKin.init_jacobians()
hyqKin.init_homogeneous()


q = np.zeros((12))
hyqKin.update_homogeneous(q)
hyqKin.update_jacobians(q)