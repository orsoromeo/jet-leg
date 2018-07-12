# -*- coding: utf-8 -*-
"""
Created on Mon May 28 09:39:54 2018

@author: Romeo Orsolino

This code computes the inverse kinematics for the HyQ quadruped robot.
Besides the joint positions and velocities it also returns the 2D jacobians referring to the HFE and KFE joints
"""
import numpy as np

class Kinematics:
    
    def compute_FK(self, q):
        x = [None]
        return x        
        
    def compute_xy_IK(self, x, x_dot, z, z_dot):
        ''' 
        This funciton computes the joint positions given the feet positions and velocities.
        Only the X Y feet coordinates are considered inthis version.
        Besides the joint positions, this function also returns the 2D jacobians referred to the
        HFE and KFE joints of the HyQ robot.
        '''
        isOutOfWorkSpace = False
        footPosDes = np.vstack([x,z])
        BASE2HAA_offsets = np.array([[0.3735,0.3735,-0.3735,-0.3735],
                                 [-.08, -.08, -.08, -.08]]);
        upperLegLength = 0.35;
        lowerLegLength = 0.346;
        footPosHAA = np.subtract(footPosDes, BASE2HAA_offsets)

        haa2hfeLength = 0.045;
        M_PI = 3.1415;
    
        sz = np.size(x,0)
        # 1 -> LF
        # 2 -> RF
        # 3 -> LH
        # 4 -> RH
        q = np.zeros((12,1))
        q_dot = np.zeros((12,1))
    
        # remove the haa2hfe offset and rotate in the sagittal plane of the leg
        hfe2foot = np.sqrt(np.square(footPosHAA[0]) + np.square(footPosHAA[1])) - haa2hfeLength;
        # add the x component
        # hfe2foot = sqrt(hfe2foot * hfe2foot);
        # HAA joints
        q[0] = -np.arctan2(footPosHAA[0,0],-footPosHAA[1,0]); # LF HAA
        q[6] = -np.arctan2(footPosHAA[0,1],-footPosHAA[1,1]); # LH HAA
        q[3] = -np.arctan2(-footPosHAA[0,2],-footPosHAA[1,2]);# RF HAA
        q[9] = -np.arctan2(-footPosHAA[0,3],-footPosHAA[1,3]);# RH HAA
    
        # HFE and KFE joints (use cosine law)
        cos_arg = (upperLegLength * upperLegLength + lowerLegLength * lowerLegLength - hfe2foot * hfe2foot) / (2 * upperLegLength * lowerLegLength);
        q[2] = - M_PI + np.arccos(cos_arg[0]); # LF KFE
        q[5] = - M_PI + np.arccos(cos_arg[0]); # RF KFE
        cos_arg = (np.square(upperLegLength) + np.square(hfe2foot) - np.square(lowerLegLength)) / (2 * upperLegLength * hfe2foot);
        sin_arg = footPosHAA[0] / hfe2foot; #it should be footPosHFE(rbd::X)/hfe2foot but footPosHFE(rbd::X) = footPosHAA(rbd::X)	
        q[1] = -np.arcsin(sin_arg[0]) + np.arccos(cos_arg[0]);# LF HFE
        if (np.isnan(q[1])):
            isOutOfWorkSpace = True
            print 'Warning! point is out of workspace!'
        q[4] = -np.arcsin(sin_arg[0]) + np.arccos(cos_arg[0]);# RF HFE
        if (np.isnan(q[4])):
            isOutOfWorkSpace = True
            print 'Warning! point is out of workspace!'
        cos_arg = (upperLegLength * upperLegLength + lowerLegLength * lowerLegLength - hfe2foot * hfe2foot)/ (2 * upperLegLength * lowerLegLength);
        q[8]= + M_PI- np.arccos(cos_arg[0]); # LH KFE
        q[11] = + M_PI - np.arccos(cos_arg[0]); # RH KFE
        cos_arg = (upperLegLength * upperLegLength + hfe2foot * hfe2foot- lowerLegLength * lowerLegLength) / (2 * upperLegLength * hfe2foot);
        sin_arg = footPosHAA[0,2] / hfe2foot; # it should be footPosHFE(rbd::X)/hfe2foot but footPosHFE(rbd::X) = footPosHAA(rbd::X)
        q[7] = -np.arcsin(sin_arg[0])- np.arccos(cos_arg[0]);# LH HFE
        if (np.isnan(q[7])):
            isOutOfWorkSpace = True
            print 'Warning! point is out of workspace!'
        q[10] = -np.arcsin(sin_arg[0])- np.arccos(cos_arg[0]);# RH HFE
        if (np.isnan(q[10])):
            isOutOfWorkSpace = True
            print 'Warning! point is out of workspace!'    
        """ compute joint velocities updating the 2D jacobians with the computed position """
        l1 = upperLegLength;
        l2 = lowerLegLength;
        Jac_LF = np.array([[np.asscalar(-l1*np.cos([q[1]]) - l2 * np.cos([q[1] + q[2]])),np.asscalar( - l2 * np.cos([q[1] + q[2]]))],
                            [np.asscalar(l1*np.sin([q[1]]) + l2 * np.sin([q[1] + q[2]])),np.asscalar(   l2 * np.sin([q[1] + q[2]]))]])
        #Jac_LF = np.array([[np.asscalar(-l1*np.cos([q[1]])+ l2 * np.cos(q[4]+q[5])),0],[0,0]])
        Jac_RF = np.array([[np.asscalar(l1*np.cos(q[4]) + l2 * np.cos(q[4]+q[5])), np.asscalar( l2 * np.cos(q[4] + q[4]))],
                            [np.asscalar(l1*np.sin(q[4]) + l2 * np.sin(q[4]+q[5])), np.asscalar( l2 * np.sin(q[4] + q[4]))]])
                            
        Jac_LH = np.array([[np.asscalar(-l1*np.cos(q[7]) - l2 * np.cos(q[7]+q[8])), np.asscalar( -l2 * np.cos(q[7] + q[8]))],
                            [np.asscalar(l1*np.sin(q[7]) + l2 * np.sin(q[7]+q[8])), np.asscalar(  l2 * np.sin(q[7] + q[8]))]])
                            
        Jac_RH = np.array([[np.asscalar(l1*np.cos(q[10]) + l2 * np.cos(q[10]+q[11])), np.asscalar( l2 * np.cos(q[10] + q[11]))],
                           [np.asscalar(l1*np.sin(q[10]) + l2 * np.sin(q[10]+q[11])), np.asscalar( l2 * np.sin(q[10] + q[11]))]])     

        footVelDes = np.vstack([x_dot, z_dot]); 
        #print footVelDes
        #q_dot[1:2] = np.linalg.inv(Jac_LF)*footVelDes;
        #q_dot[4:5] = np.linalg.inv(Jac_RF)*footVelDes;
        #q_dot[7:8] = np.linalg.inv(Jac_LH)*footVelDes;
        #q_dot[10:11] = np.linalg.inv(Jac_RH)*footVelDes;   
        return q, q_dot, Jac_LF, Jac_RF, Jac_LH, Jac_RH, isOutOfWorkSpace
    
    def update_jacobians(self, q):
        '''
        Here in the following I copy pasted the "update_jacobians.m" file by Michele Focchi.
        Once translated into python, this will update the 3D jacobians of HyQ's legs given 
        the current joint positions.
        '''
       # s__q_LF_HAA = sin( q(1));
       # s__q_LF_HFE = sin( q(2));
       # s__q_LF_KFE = sin( q(3));
       # c__q_LF_HAA = cos( q(1));
       # c__q_LF_HFE = cos( q(2));
       # c__q_LF_KFE = cos( q(3));
       # 
       # fr_trunk_J_LF_foot(2,2) =  c__q_LF_HAA;
       # fr_trunk_J_LF_foot(2,3) =  c__q_LF_HAA;
       # fr_trunk_J_LF_foot(3,2) = - s__q_LF_HAA;
       # fr_trunk_J_LF_foot(3,3) = - s__q_LF_HAA;
       # fr_trunk_J_LF_foot(4,2) = ( 0.341 *  s__q_LF_HFE *  s__q_LF_KFE) - ( 0.341 *  c__q_LF_HFE *  c__q_LF_KFE) - ( 0.35 *  c__q_LF_HFE);
       # fr_trunk_J_LF_foot(4,3) = ( 0.341 *  s__q_LF_HFE *  s__q_LF_KFE) - ( 0.341 *  c__q_LF_HFE *  c__q_LF_KFE);
       # fr_trunk_J_LF_foot(5,1) = ( 0.341 *  c__q_LF_HAA *  s__q_LF_HFE *  s__q_LF_KFE) - ( 0.341 *  c__q_LF_HAA *  c__q_LF_HFE *  c__q_LF_KFE) - ( 0.35 *  c__q_LF_HAA *  c__q_LF_HFE) - ( 0.08 *  c__q_LF_HAA);
       # fr_trunk_J_LF_foot(5,2) = ( 0.341 *  s__q_LF_HAA *  c__q_LF_HFE *  s__q_LF_KFE) + ( 0.341 *  s__q_LF_HAA *  s__q_LF_HFE *  c__q_LF_KFE) + ( 0.35 *  s__q_LF_HAA *  s__q_LF_HFE);
       # fr_trunk_J_LF_foot(5,3) = ( 0.341 *  s__q_LF_HAA *  c__q_LF_HFE *  s__q_LF_KFE) + ( 0.341 *  s__q_LF_HAA *  s__q_LF_HFE *  c__q_LF_KFE);
       # fr_trunk_J_LF_foot(6,1) = (- 0.341 *  s__q_LF_HAA *  s__q_LF_HFE *  s__q_LF_KFE) + ( 0.341 *  s__q_LF_HAA *  c__q_LF_HFE *  c__q_LF_KFE) + ( 0.35 *  s__q_LF_HAA *  c__q_LF_HFE) + ( 0.08 *  s__q_LF_HAA);
       # fr_trunk_J_LF_foot(6,2) = ( 0.341 *  c__q_LF_HAA *  c__q_LF_HFE *  s__q_LF_KFE) + ( 0.341 *  c__q_LF_HAA *  s__q_LF_HFE *  c__q_LF_KFE) + ( 0.35 *  c__q_LF_HAA *  s__q_LF_HFE);
       # fr_trunk_J_LF_foot(6,3) = ( 0.341 *  c__q_LF_HAA *  c__q_LF_HFE *  s__q_LF_KFE) + ( 0.341 *  c__q_LF_HAA *  s__q_LF_HFE *  c__q_LF_KFE);
       # 
       # 
       # s__q_RF_HAA = sin( q(4));
       # s__q_RF_HFE = sin( q(5));
       # s__q_RF_KFE = sin( q(6));
       # c__q_RF_HAA = cos( q(4));
       # c__q_RF_HFE = cos( q(5));
       # c__q_RF_KFE = cos( q(6));
       # 
       # fr_trunk_J_RF_foot(2,2) =  c__q_RF_HAA;
       # fr_trunk_J_RF_foot(2,3) =  c__q_RF_HAA;
       # fr_trunk_J_RF_foot(3,2) =  s__q_RF_HAA;
       # fr_trunk_J_RF_foot(3,3) =  s__q_RF_HAA;
       # fr_trunk_J_RF_foot(4,2) = ( 0.341 *  s__q_RF_HFE *  s__q_RF_KFE) - ( 0.341 *  c__q_RF_HFE *  c__q_RF_KFE) - ( 0.35 *  c__q_RF_HFE);
       # fr_trunk_J_RF_foot(4,3) = ( 0.341 *  s__q_RF_HFE *  s__q_RF_KFE) - ( 0.341 *  c__q_RF_HFE *  c__q_RF_KFE);
       # fr_trunk_J_RF_foot(5,1) = (- 0.341 *  c__q_RF_HAA *  s__q_RF_HFE *  s__q_RF_KFE) + ( 0.341 *  c__q_RF_HAA *  c__q_RF_HFE *  c__q_RF_KFE) + ( 0.35 *  c__q_RF_HAA *  c__q_RF_HFE) + ( 0.08 *  c__q_RF_HAA);
       # fr_trunk_J_RF_foot(5,2) = (- 0.341 *  s__q_RF_HAA *  c__q_RF_HFE *  s__q_RF_KFE) - ( 0.341 *  s__q_RF_HAA *  s__q_RF_HFE *  c__q_RF_KFE) - ( 0.35 *  s__q_RF_HAA *  s__q_RF_HFE);
       # fr_trunk_J_RF_foot(5,3) = (- 0.341 *  s__q_RF_HAA *  c__q_RF_HFE *  s__q_RF_KFE) - ( 0.341 *  s__q_RF_HAA *  s__q_RF_HFE *  c__q_RF_KFE);
       # fr_trunk_J_RF_foot(6,1) = (- 0.341 *  s__q_RF_HAA *  s__q_RF_HFE *  s__q_RF_KFE) + ( 0.341 *  s__q_RF_HAA *  c__q_RF_HFE *  c__q_RF_KFE) + ( 0.35 *  s__q_RF_HAA *  c__q_RF_HFE) + ( 0.08 *  s__q_RF_HAA);
       # fr_trunk_J_RF_foot(6,2) = ( 0.341 *  c__q_RF_HAA *  c__q_RF_HFE *  s__q_RF_KFE) + ( 0.341 *  c__q_RF_HAA *  s__q_RF_HFE *  c__q_RF_KFE) + ( 0.35 *  c__q_RF_HAA *  s__q_RF_HFE);
       # fr_trunk_J_RF_foot(6,3) = ( 0.341 *  c__q_RF_HAA *  c__q_RF_HFE *  s__q_RF_KFE) + ( 0.341 *  c__q_RF_HAA *  s__q_RF_HFE *  c__q_RF_KFE);
       # 
       # 
       # s__q_LH_HAA = sin( q(7));
       # s__q_LH_HFE = sin( q(8));
       # s__q_LH_KFE = sin( q(9));
       # c__q_LH_HAA = cos( q(7));
       # c__q_LH_HFE = cos( q(8));
       # c__q_LH_KFE = cos( q(9));
       # 
       # fr_trunk_J_LH_foot(2,2) =  c__q_LH_HAA;
       # fr_trunk_J_LH_foot(2,3) =  c__q_LH_HAA;
       # fr_trunk_J_LH_foot(3,2) = - s__q_LH_HAA;
       # fr_trunk_J_LH_foot(3,3) = - s__q_LH_HAA;
       # fr_trunk_J_LH_foot(4,2) = ( 0.341 *  s__q_LH_HFE *  s__q_LH_KFE) - ( 0.341 *  c__q_LH_HFE *  c__q_LH_KFE) - ( 0.35 *  c__q_LH_HFE);
       # fr_trunk_J_LH_foot(4,3) = ( 0.341 *  s__q_LH_HFE *  s__q_LH_KFE) - ( 0.341 *  c__q_LH_HFE *  c__q_LH_KFE);
       # fr_trunk_J_LH_foot(5,1) = ( 0.341 *  c__q_LH_HAA *  s__q_LH_HFE *  s__q_LH_KFE) - ( 0.341 *  c__q_LH_HAA *  c__q_LH_HFE *  c__q_LH_KFE) - ( 0.35 *  c__q_LH_HAA *  c__q_LH_HFE) - ( 0.08 *  c__q_LH_HAA);
       # fr_trunk_J_LH_foot(5,2) = ( 0.341 *  s__q_LH_HAA *  c__q_LH_HFE *  s__q_LH_KFE) + ( 0.341 *  s__q_LH_HAA *  s__q_LH_HFE *  c__q_LH_KFE) + ( 0.35 *  s__q_LH_HAA *  s__q_LH_HFE);
       # fr_trunk_J_LH_foot(5,3) = ( 0.341 *  s__q_LH_HAA *  c__q_LH_HFE *  s__q_LH_KFE) + ( 0.341 *  s__q_LH_HAA *  s__q_LH_HFE *  c__q_LH_KFE);
       # fr_trunk_J_LH_foot(6,1) = (- 0.341 *  s__q_LH_HAA *  s__q_LH_HFE *  s__q_LH_KFE) + ( 0.341 *  s__q_LH_HAA *  c__q_LH_HFE *  c__q_LH_KFE) + ( 0.35 *  s__q_LH_HAA *  c__q_LH_HFE) + ( 0.08 *  s__q_LH_HAA);
       # fr_trunk_J_LH_foot(6,2) = ( 0.341 *  c__q_LH_HAA *  c__q_LH_HFE *  s__q_LH_KFE) + ( 0.341 *  c__q_LH_HAA *  s__q_LH_HFE *  c__q_LH_KFE) + ( 0.35 *  c__q_LH_HAA *  s__q_LH_HFE);
       # fr_trunk_J_LH_foot(6,3) = ( 0.341 *  c__q_LH_HAA *  c__q_LH_HFE *  s__q_LH_KFE) + ( 0.341 *  c__q_LH_HAA *  s__q_LH_HFE *  c__q_LH_KFE);
       # 
       # 
       # s__q_RH_HAA = sin( q(10));
       # s__q_RH_HFE = sin( q(11));
       # s__q_RH_KFE = sin( q(12));
       # c__q_RH_HAA = cos( q(10));
       # c__q_RH_HFE = cos( q(11));
       # c__q_RH_KFE = cos( q(12));
       # 
       # fr_trunk_J_RH_foot(2,2) =  c__q_RH_HAA;
       # fr_trunk_J_RH_foot(2,3) =  c__q_RH_HAA;
       # fr_trunk_J_RH_foot(3,2) =  s__q_RH_HAA;
       # fr_trunk_J_RH_foot(3,3) =  s__q_RH_HAA;
       # fr_trunk_J_RH_foot(4,2) = ( 0.341 *  s__q_RH_HFE *  s__q_RH_KFE) - ( 0.341 *  c__q_RH_HFE *  c__q_RH_KFE) - ( 0.35 *  c__q_RH_HFE);
       # fr_trunk_J_RH_foot(4,3) = ( 0.341 *  s__q_RH_HFE *  s__q_RH_KFE) - ( 0.341 *  c__q_RH_HFE *  c__q_RH_KFE);
       # fr_trunk_J_RH_foot(5,1) = (- 0.341 *  c__q_RH_HAA *  s__q_RH_HFE *  s__q_RH_KFE) + ( 0.341 *  c__q_RH_HAA *  c__q_RH_HFE *  c__q_RH_KFE) + ( 0.35 *  c__q_RH_HAA *  c__q_RH_HFE) + ( 0.08 *  c__q_RH_HAA);
       # fr_trunk_J_RH_foot(5,2) = (- 0.341 *  s__q_RH_HAA *  c__q_RH_HFE *  s__q_RH_KFE) - ( 0.341 *  s__q_RH_HAA *  s__q_RH_HFE *  c__q_RH_KFE) - ( 0.35 *  s__q_RH_HAA *  s__q_RH_HFE);
       # fr_trunk_J_RH_foot(5,3) = (- 0.341 *  s__q_RH_HAA *  c__q_RH_HFE *  s__q_RH_KFE) - ( 0.341 *  s__q_RH_HAA *  s__q_RH_HFE *  c__q_RH_KFE);
       # fr_trunk_J_RH_foot(6,1) = (- 0.341 *  s__q_RH_HAA *  s__q_RH_HFE *  s__q_RH_KFE) + ( 0.341 *  s__q_RH_HAA *  c__q_RH_HFE *  c__q_RH_KFE) + ( 0.35 *  s__q_RH_HAA *  c__q_RH_HFE) + ( 0.08 *  s__q_RH_HAA);
       # fr_trunk_J_RH_foot(6,2) = ( 0.341 *  c__q_RH_HAA *  c__q_RH_HFE *  s__q_RH_KFE) + ( 0.341 *  c__q_RH_HAA *  s__q_RH_HFE *  c__q_RH_KFE) + ( 0.35 *  c__q_RH_HAA *  s__q_RH_HFE);
       # fr_trunk_J_RH_foot(6,3) = ( 0.341 *  c__q_RH_HAA *  c__q_RH_HFE *  s__q_RH_KFE) + ( 0.341 *  c__q_RH_HAA *  s__q_RH_HFE *  c__q_RH_KFE);
        aaa = 0
        return aaa