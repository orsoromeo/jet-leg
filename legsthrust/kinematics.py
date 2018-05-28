# -*- coding: utf-8 -*-
"""
Created on Mon May 28 09:39:54 2018

@author: rorsolino
"""
import numpy as np

class Kinematics:
        
    def computeIK(self, x, x_dot, z, z_dot):
        footPosDes = np.vstack([x,z])
        BASE2HAA_offsets = np.array([[0.3735,0.3735,-0.3735,-0.3735],
                                 [-.08, -.08, -.08, -.08]]);
        upperLegLength = 0.35;
        lowerLegLength = 0.341;
        footPosHAA = np.subtract(footPosDes, BASE2HAA_offsets)
        haa2hfeLength = 0.045;
        M_PI = 3.1415;
    
        sz = np.size(x,0);
        # 1 -> LF
        # 2 -> RF
        # 3 -> LH
        # 4 -> RH
        q = np.zeros(12,sz);
        q_dot = np.zeros(12,sz);
    
        # remove the haa2hfe offset and rotate in the sagittal plane of the leg
        hfe2foot = np.sqrt(footPosHAA(1)^2 + footPosHAA(2)^2) - haa2hfeLength;
        # add the x component
        # hfe2foot = sqrt(hfe2foot * hfe2foot);
        # HAA joints
        q[0] = -np.arctan2(footPosHAA[0,0],-footPosHAA[1,0]); # LF HAA
        q[6] = -np.arctan2(footPosHAA(0,1),-footPosHAA(1,1)); # LH HAA
        q[3] = -np.arctan2(-footPosHAA(0,2),-footPosHAA(1,2));# RF HAA
        q[9] = -np.arctan2(-footPosHAA(0,3),-footPosHAA(1,3));# RH HAA
    
        # HFE and KFE joints (use cosine law)
        cos_arg = (upperLegLength * upperLegLength + lowerLegLength * lowerLegLength - hfe2foot * hfe2foot) / (2 * upperLegLength * lowerLegLength);
        q[3] = - M_PI + acos(cos_arg); # LF KFE
        q[6] = - M_PI + acos(cos_arg); # RF KFE
        cos_arg = (upperLegLength^2 + hfe2foot^2 - lowerLegLength^2) / (2 * upperLegLength * hfe2foot) ;
        sin_arg = footPosHAA(1) / hfe2foot; #it should be footPosHFE(rbd::X)/hfe2foot but footPosHFE(rbd::X) = footPosHAA(rbd::X)	
        q[2] = -asin(sin_arg) + acos(cos_arg);# LF HFE
        q[5] = -asin(sin_arg) + acos(cos_arg);# RF HFE
    
        cos_arg = (upperLegLength * upperLegLength + lowerLegLength * lowerLegLength - hfe2foot * hfe2foot)/ (2 * upperLegLength * lowerLegLength);
        q[9]= + M_PI- acos(cos_arg); # LH KFE
        q[12] = + M_PI- acos(cos_arg); # RH KFE
        cos_arg = (upperLegLength * upperLegLength + hfe2foot * hfe2foot- lowerLegLength * lowerLegLength) / (2 * upperLegLength * hfe2foot);
        sin_arg = footPosHAA(1,3) / hfe2foot; # it should be footPosHFE(rbd::X)/hfe2foot but footPosHFE(rbd::X) = footPosHAA(rbd::X)
        q[8] = -asin(sin_arg)- acos(cos_arg);# LH HFE
        q[11] = -asin(sin_arg)- acos(cos_arg);# RH HFE
    
        # compute joint velocities updating the jac with the computed position
        l1 = upperLegLength;
        l2 = lowerLegLength;
        
        Jac_LF = np.array([[-l1*cos(q(2)) - l2 * cos(q(2) + q(3)), - l2 * cos(q(2) + q(3))],
                            [l1*sin(q(2)) + l2 * sin(q(2)+q(3)),  l2 * sin(q(2)+q(3))]]);
        Jac_RF = np.array([[l1*cos(q(5)) + l2 * cos(q(5)+q(6)),  l2 * cos(q(5) + q(6))],
                 [l1*sin(q(5)) + l2 * sin(q(5)+q(6)),  l2 * sin(q(5) + q(6))]]);
         
        Jac_LH = np.array([[-l1*cos(q(8)) - l2 * cos(q(8)+q(9)),  -l2 * cos(q(8)+q(9))],
                  [l1*sin(q(8)) + l2 * sin(q(8)+q(9)),  l2 * sin(q(8)+q(9))]]);
        Jac_RH = np.array([[l1*cos(q(11)) + l2 * cos(q(11)+q(12)),  l2 * cos(q(11)+q(12))],
                  [l1*sin(q(11)) + l2 * sin(q(11)+q(12)),  l2 * sin(q(11)+q(12))]]);
        
        footVelDes = np.vstack([x_dot, z_dot]); 
        return q, Jac_LF, Jac_RF, Jac_LH, Jac_RH