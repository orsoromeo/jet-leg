# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 05:34:42 2018

@author: romeo orsolino
"""
import numpy as np

class HyQKinematics:
    def __init__(self):

        self.upperLegLength = 0.35;
        self.lowerLegLength = 0.346;

        self.BASE2HAA_offset_x = 0.3735;
        self.BASE2HAA_offset_z = 0.08;
        self.BASE2HAA_offset_y = 0.208;

        self.isOutOfWorkSpace = False
        
        self.fr_LF_lowerleg_Xh_LF_foot = np.zeros((4,4));	
        self.fr_RF_lowerleg_Xh_RF_foot = np.zeros((4,4));
        self.fr_LH_lowerleg_Xh_LH_foot = np.zeros((4,4));
        self.fr_RH_lowerleg_Xh_RH_foot = np.zeros((4,4));
        self.LF_foot_Xh_fr_trunk = np.zeros((4,4));
        self.RF_foot_Xh_fr_trunk = np.zeros((4,4));
        self.LH_foot_Xh_fr_trunk = np.zeros((4,4));
        self.RH_foot_Xh_fr_trunk = np.zeros((4,4));
        self.fr_trunk_Xh_LF_foot = np.zeros((4,4));    
        self.fr_trunk_Xh_fr_LF_HAA = np.zeros((4,4));
        self.fr_trunk_Xh_fr_LF_HFE = np.zeros((4,4));
        self.fr_trunk_Xh_fr_LF_KFE = np.zeros((4,4));
        self.fr_trunk_Xh_fr_RF_HAA = np.zeros((4,4));
        self.fr_trunk_Xh_RF_foot = np.zeros((4,4));
        self.fr_trunk_Xh_fr_RF_HFE = np.zeros((4,4));
        self.fr_trunk_Xh_fr_RF_KFE = np.zeros((4,4));
        self.fr_trunk_Xh_LH_foot = np.zeros((4,4));
        self.fr_trunk_Xh_fr_LH_HAA = np.zeros((4,4));
        self.fr_trunk_Xh_fr_LH_HFE = np.zeros((4,4));
        self.fr_trunk_Xh_fr_LH_KFE = np.zeros((4,4));
        self.fr_trunk_Xh_RH_foot =np.zeros((4,4));
        self.fr_trunk_Xh_fr_RH_HAA =np.zeros((4,4));
        self.fr_trunk_Xh_fr_RH_HFE =np.zeros((4,4));
        self.fr_trunk_Xh_fr_RH_KFE =np.zeros((4,4));
        self.fr_LF_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        self.fr_trunk_Xh_fr_LF_hipassembly =np.zeros((4,4));
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly =np.zeros((4,4));
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg =np.zeros((4,4));
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg =np.zeros((4,4));
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg =np.zeros((4,4));
        self.fr_RF_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        self.fr_trunk_Xh_fr_RF_hipassembly =np.zeros((4,4));
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly =np.zeros((4,4));
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg =np.zeros((4,4));
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg =np.zeros((4,4));
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg =np.zeros((4,4));
        self.fr_LH_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        self.fr_trunk_Xh_fr_LH_hipassembly =np.zeros((4,4));
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly =np.zeros((4,4));
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg =np.zeros((4,4));
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg =np.zeros((4,4));
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg =np.zeros((4,4));
        self.fr_RH_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        self.fr_trunk_Xh_fr_RH_hipassembly =np.zeros((4,4));
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly =np.zeros((4,4));
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg =np.zeros((4,4));
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg =np.zeros((4,4));
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg =np.zeros((4,4));

        self.s__q_LF_HFE = 0.0
        self.s__q_LF_KFE = 0.0
        self.s__q_LF_HAA = 0.0
        self.s__q_RF_HFE = 0.0
        self.s__q_RF_KFE = 0.0
        self.s__q_RF_HAA = 0.0
        self.s__q_LH_HFE = 0.0
        self.s__q_LH_KFE = 0.0
        self.s__q_LH_HAA = 0.0
        self.s__q_RH_HFE = 0.0
        self.s__q_RH_KFE = 0.0
        self.s__q_RH_HAA = 0.0
        self.c__q_LF_HFE = 0.0
        self.c__q_LF_KFE = 0.0
        self.c__q_LF_HAA = 0.0
        self.c__q_RF_HFE = 0.0
        self.c__q_RF_KFE = 0.0
        self.c__q_RF_HAA = 0.0
        self.c__q_LH_HFE = 0.0
        self.c__q_LH_KFE = 0.0
        self.c__q_LH_HAA = 0.0
        self.c__q_RH_HFE = 0.0
        self.c__q_RH_KFE = 0.0
        self.c__q_RH_HAA = 0.0
        
        '''jacobians'''
        self.fr_trunk_J_LF_foot = np.zeros((6,3));
        self.fr_trunk_J_RF_foot = np.zeros((6,3));
        self.fr_trunk_J_LH_foot = np.zeros((6,3));
        self.fr_trunk_J_RH_foot = np.zeros((6,3));
        
        '''initialize quantities'''
        self.init_jacobians()
        self.init_homogeneous()
        

    def init_jacobians(self):

        self.fr_trunk_J_LF_foot[0,0] = - 1.0;
        self.fr_trunk_J_RF_foot[0,0] = 1.0;
        self.fr_trunk_J_LH_foot[0,0] = - 1.0;
        self.fr_trunk_J_RH_foot[0,0] = 1.0;

    def init_homogeneous(self):
        
        self.fr_LF_lowerleg_Xh_LF_foot[0,2] = - 1.0;
        self.fr_LF_lowerleg_Xh_LF_foot[0,3] = self.lowerLegLength;
        self.fr_LF_lowerleg_Xh_LF_foot[1,0] = - 1;
        self.fr_LF_lowerleg_Xh_LF_foot[2,1] = 1;
        self.fr_LF_lowerleg_Xh_LF_foot[3,3] = 1;	
               
        self.fr_RF_lowerleg_Xh_RF_foot[0,2] = - 1.0;
        self.fr_RF_lowerleg_Xh_RF_foot[0,3] = self.lowerLegLength;
        self.fr_RF_lowerleg_Xh_RF_foot[1,0] = - 1;
        self.fr_RF_lowerleg_Xh_RF_foot[2,1] = 1;
        self.fr_RF_lowerleg_Xh_RF_foot[3,3] = 1;	
        
        self.fr_LH_lowerleg_Xh_LH_foot[0,2] = - 1.0;
        self.fr_LH_lowerleg_Xh_LH_foot[0,3] = self.lowerLegLength;
        self.fr_LH_lowerleg_Xh_LH_foot[1,0] = - 1;
        self.fr_LH_lowerleg_Xh_LH_foot[2,1] = 1;
        self.fr_LH_lowerleg_Xh_LH_foot[3,3] = 1;	
        
        self.fr_RH_lowerleg_Xh_RH_foot[0,2] = - 1.0;
        self.fr_RH_lowerleg_Xh_RH_foot[0,3] = self.lowerLegLength;
        self.fr_RH_lowerleg_Xh_RH_foot[1,0] = - 1;
        self.fr_RH_lowerleg_Xh_RH_foot[2,1] = 1;
        self.fr_RH_lowerleg_Xh_RH_foot[3,3] = 1;	
        
        self.LF_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        self.RF_foot_Xh_fr_trunk[3,3]= 1.0;	
        
        self.LH_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        self.RH_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_LF_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LF_HAA[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HAA[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LF_HAA[1,1] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HAA[1,3] = self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LF_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LF_HFE[0,1] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HFE[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LF_HFE[3,3] = 1.0;	
         
        self.fr_trunk_Xh_fr_LF_KFE[3,3] = 1.0;	  
         
        self.fr_trunk_Xh_RF_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_HAA[0,2] = 1.0;
        self.fr_trunk_Xh_fr_RF_HAA[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RF_HAA[1,1] = 1.0;
        self.fr_trunk_Xh_fr_RF_HAA[1,3] = - self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RF_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_RF_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_HFE[0,1] = - 1.0;
        self.fr_trunk_Xh_fr_RF_HFE[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RF_HFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_KFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_LH_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_HAA[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HAA[0,3] = - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LH_HAA[1,1] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HAA[1,3] = self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LH_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_HFE[0,0] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HFE[0,3] = - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LH_HFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_KFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_RH_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_HAA[0,2] = 1.0;
        self.fr_trunk_Xh_fr_RH_HAA[0,3] = - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RH_HAA[1,1] = 1.0;
        self.fr_trunk_Xh_fr_RH_HAA[1,3] = - self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RH_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_RH_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_HFE[0,1] = - 1.0;
        self.fr_trunk_Xh_fr_RH_HFE[0,3] = - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RH_HFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_KFE[3,3] = 1.0;	
        
        self.fr_LF_hipassembly_Xh_fr_trunk[2,0] = - 1.0;
        self.fr_LF_hipassembly_Xh_fr_trunk[2,3] = self.BASE2HAA_offset_x;
        self.fr_LF_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LF_hipassembly[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LF_hipassembly[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LF_hipassembly[1,3] = self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LF_hipassembly[3,3] = 1.0;	
            
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[2,1] = - 1;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[3,3] = 1.0;	
     
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[0,3] = self.BASE2HAA_offset_z;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[1,2] = - 1;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[3,3] = 1;	
        
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[2,2] = 1;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[3,3] = 1.0;	
        
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[0,3] = 0.35;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[2,2] = 1;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[3,3] = 1;	
        
        self.fr_RF_hipassembly_Xh_fr_trunk[2,0] = 1.0;
        self.fr_RF_hipassembly_Xh_fr_trunk[2,3] = - self.BASE2HAA_offset_x;
        self.fr_RF_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_hipassembly[0,2] = 1.0;
        self.fr_trunk_Xh_fr_RF_hipassembly[0,3] = self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RF_hipassembly[1,3] = - self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RF_hipassembly[3,3] = 1.0;	
        
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[2,1] = 1;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[3,3] = 1.0;	
        
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[0,3] = self.BASE2HAA_offset_z;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[1,2] = 1;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[3,3] = 1;	
        
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[2,2] = 1;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[3,3] = 1.0;	
        
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[0,3] = self.upperLegLength;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[2,2] = 1;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[3,3] = 1;	
        
        self.fr_LH_hipassembly_Xh_fr_trunk[2,0] = - 1.0;
        self.fr_LH_hipassembly_Xh_fr_trunk[2,3] = - self.BASE2HAA_offset_x;
        self.fr_LH_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_hipassembly[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LH_hipassembly[0,3] = - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LH_hipassembly[1,3] = self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LH_hipassembly[3,3] = 1.0;	
        
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[2,1] = - 1;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[3,3] = 1.0;	
        
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[0,3] = self.BASE2HAA_offset_z;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[1,2] = - 1;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[3,3] = 1;	
        
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[2,2]= 1;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[3,3]= 1.0;	
        
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[0,3] = self.upperLegLength;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[2,2] = 1;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[3,3] = 1;	
        

        self.fr_RH_hipassembly_Xh_fr_trunk[2,0] = 1.0;
        self.fr_RH_hipassembly_Xh_fr_trunk[2,3] = self.BASE2HAA_offset_x;
        self.fr_RH_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_hipassembly[0,2]= 1.0;
        self.fr_trunk_Xh_fr_RH_hipassembly[0,3]= - self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RH_hipassembly[1,3]= - self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RH_hipassembly[3,3]= 1.0;	
        
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[2,1]= 1;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[3,3]= 1.0;	
        
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[0,3] = self.BASE2HAA_offset_z;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[1,2] = 1;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[3,3] = 1;	
        
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[2,2] = 1;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[3,3] = 1.0;
        
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[0,3] = self.upperLegLength;
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[2,2] = 1;
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[3,3] = 1;
        
        return True
                
    def update_homogeneous(self, q):
        
        self.s__q_LF_HFE = np.sin( q[1]);
        self.s__q_LF_KFE = np.sin( q[2]);
        self.s__q_LF_HAA = np.sin( q[0]);
        self.s__q_RF_HFE = np.sin( q[4]);
        self.s__q_RF_KFE = np.sin( q[5]);
        self.s__q_RF_HAA = np.sin( q[3]);
        self.s__q_LH_HFE = np.sin( q[7]);
        self.s__q_LH_KFE = np.sin( q[8]);
        self.s__q_LH_HAA = np.sin( q[6]);
        self.s__q_RH_HFE = np.sin( q[10]);
        self.s__q_RH_KFE = np.sin( q[11]);
        self.s__q_RH_HAA = np.sin( q[9]);
        self.c__q_LF_HFE = np.cos( q[1]);
        self.c__q_LF_KFE = np.cos( q[2]);
        self.c__q_LF_HAA = np.cos( q[0]);
        self.c__q_RF_HFE = np.cos( q[4]);
        self.c__q_RF_KFE = np.cos( q[5]);
        self.c__q_RF_HAA = np.cos( q[3]);
        self.c__q_LH_HFE = np.cos( q[7]);
        self.c__q_LH_KFE = np.cos( q[8]);
        self.c__q_LH_HAA = np.cos( q[6]);
        self.c__q_RH_HFE = np.cos( q[10]);
        self.c__q_RH_KFE = np.cos( q[11]);
        self.c__q_RH_HAA = np.cos( q[9]);
        
        
        self.LF_foot_Xh_fr_trunk[0,0] = ( self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[0,1] = (- self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[0,2] = (- self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[0,3] = ((( self.BASE2HAA_offset_x *  self.s__q_LF_HFE) + ((( self.BASE2HAA_offset_y *  self.s__q_LF_HAA) -  self.BASE2HAA_offset_z) *  self.c__q_LF_HFE) -  self.upperLegLength) *  self.s__q_LF_KFE) + ((((( self.BASE2HAA_offset_y *  self.s__q_LF_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_LF_HFE) - ( self.BASE2HAA_offset_x *  self.c__q_LF_HFE)) *  self.c__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[1,1] =  self.c__q_LF_HAA;
        self.LF_foot_Xh_fr_trunk[1,2] = - self.s__q_LF_HAA;
        self.LF_foot_Xh_fr_trunk[1,3] = - self.BASE2HAA_offset_y *  self.c__q_LF_HAA;
        self.LF_foot_Xh_fr_trunk[2,0] = ( self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[2,1] = ( self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[2,2] = ( self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[2,3] = ((((( self.BASE2HAA_offset_y *  self.s__q_LF_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_LF_HFE) - ( self.BASE2HAA_offset_x *  self.c__q_LF_HFE)) *  self.s__q_LF_KFE) + (((- self.BASE2HAA_offset_x *  self.s__q_LF_HFE) + (( self.BASE2HAA_offset_z - ( self.BASE2HAA_offset_y *  self.s__q_LF_HAA)) *  self.c__q_LF_HFE) +  self.upperLegLength) *  self.c__q_LF_KFE) +  self.lowerLegLength;
        
        
        
        self.RF_foot_Xh_fr_trunk[0,0] = ( self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[0,1] = ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[0,2] = (- self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[0,3] = ((( self.BASE2HAA_offset_x *  self.s__q_RF_HFE) + ((( self.BASE2HAA_offset_y *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_z) *  self.c__q_RF_HFE) -  self.upperLegLength) *  self.s__q_RF_KFE) + ((((( self.BASE2HAA_offset_y *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_RF_HFE) - ( self.BASE2HAA_offset_x *  self.c__q_RF_HFE)) *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[1,1] =  self.c__q_RF_HAA;
        self.RF_foot_Xh_fr_trunk[1,2] =  self.s__q_RF_HAA;
        self.RF_foot_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_RF_HAA;
        self.RF_foot_Xh_fr_trunk[2,0] = ( self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[2,1] = ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[2,2] = ( self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[2,3] = ((((( self.BASE2HAA_offset_y *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_RF_HFE) - ( self.BASE2HAA_offset_x *  self.c__q_RF_HFE)) *  self.s__q_RF_KFE) + (((- self.BASE2HAA_offset_x *  self.s__q_RF_HFE) + (( self.BASE2HAA_offset_z - ( self.BASE2HAA_offset_y *  self.s__q_RF_HAA)) *  self.c__q_RF_HFE) +  self.upperLegLength) *  self.c__q_RF_KFE) +  self.lowerLegLength;
        
        
        
        self.LH_foot_Xh_fr_trunk[0,0] = ( self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[0,1] = (- self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[0,2] = (- self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[0,3] = (((- self.BASE2HAA_offset_x *  self.s__q_LH_HFE) + ((( self.BASE2HAA_offset_y *  self.s__q_LH_HAA) -  self.BASE2HAA_offset_z) *  self.c__q_LH_HFE) -  self.upperLegLength) *  self.s__q_LH_KFE) + ((((( self.BASE2HAA_offset_y *  self.s__q_LH_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_LH_HFE) + ( self.BASE2HAA_offset_x *  self.c__q_LH_HFE)) *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[1,1] =  self.c__q_LH_HAA;
        self.LH_foot_Xh_fr_trunk[1,2] = - self.s__q_LH_HAA;
        self.LH_foot_Xh_fr_trunk[1,3] = - self.BASE2HAA_offset_y *  self.c__q_LH_HAA;
        self.LH_foot_Xh_fr_trunk[2,0] = ( self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[2,1] = ( self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[2,2] = ( self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[2,3] = ((((( self.BASE2HAA_offset_y *  self.s__q_LH_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_LH_HFE) + ( self.BASE2HAA_offset_x *  self.c__q_LH_HFE)) *  self.s__q_LH_KFE) + ((( self.BASE2HAA_offset_x *  self.s__q_LH_HFE) + (( self.BASE2HAA_offset_z - ( self.BASE2HAA_offset_y *  self.s__q_LH_HAA)) *  self.c__q_LH_HFE) +  self.upperLegLength) *  self.c__q_LH_KFE) +  self.lowerLegLength;
        
        
        
        self.RH_foot_Xh_fr_trunk[0,0] = ( self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[0,1] = ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[0,2] = (- self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[0,3] = (((- self.BASE2HAA_offset_x *  self.s__q_RH_HFE) + ((( self.BASE2HAA_offset_y *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_z) *  self.c__q_RH_HFE) -  self.upperLegLength) *  self.s__q_RH_KFE) + ((((( self.BASE2HAA_offset_y *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_RH_HFE) + ( self.BASE2HAA_offset_x *  self.c__q_RH_HFE)) *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[1,1] =  self.c__q_RH_HAA;
        self.RH_foot_Xh_fr_trunk[1,2] =  self.s__q_RH_HAA;
        self.RH_foot_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_RH_HAA;
        self.RH_foot_Xh_fr_trunk[2,0] = ( self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[2,1] = ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[2,2] = ( self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[2,3] = ((((( self.BASE2HAA_offset_y *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_z) *  self.s__q_RH_HFE) + ( self.BASE2HAA_offset_x *  self.c__q_RH_HFE)) *  self.s__q_RH_KFE) + ((( self.BASE2HAA_offset_x *  self.s__q_RH_HFE) + (( self.BASE2HAA_offset_z - ( self.BASE2HAA_offset_y *  self.s__q_RH_HAA)) *  self.c__q_RH_HFE) +  self.upperLegLength) *  self.c__q_RH_KFE) +  self.lowerLegLength;
        
        
        
        self.fr_trunk_Xh_LF_foot[0,0] = ( self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[0,2] = ( self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[0,3] = (- self.lowerLegLength *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.s__q_LF_HFE *  self.c__q_LF_KFE) - ( self.upperLegLength *  self.s__q_LF_HFE) +  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_LF_foot[1,0] = (- self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[1,1] =  self.c__q_LF_HAA;
        self.fr_trunk_Xh_LF_foot[1,2] = ( self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[1,3] = ( self.lowerLegLength *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.upperLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE) - ( self.BASE2HAA_offset_z *  self.s__q_LF_HAA) +  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_LF_foot[2,0] = (- self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[2,1] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_LF_foot[2,2] = ( self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[2,3] = ( self.lowerLegLength *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.upperLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LF_HAA);
        
        
        
        self.fr_trunk_Xh_fr_LF_HFE[1,0] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[1,2] =  self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[1,3] =  self.BASE2HAA_offset_y - ( self.BASE2HAA_offset_z *  self.s__q_LF_HAA);
        self.fr_trunk_Xh_fr_LF_HFE[2,0] = - self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[2,2] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[2,3] = - self.BASE2HAA_offset_z *  self.c__q_LF_HAA;
        
        
        
        self.fr_trunk_Xh_fr_LF_KFE[0,0] = - self.s__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[0,1] = - self.c__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[0,3] =  self.BASE2HAA_offset_x - ( self.upperLegLength *  self.s__q_LF_HFE);
        self.fr_trunk_Xh_fr_LF_KFE[1,0] = - self.s__q_LF_HAA *  self.c__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[1,1] =  self.s__q_LF_HAA *  self.s__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[1,2] =  self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_KFE[1,3] = (- self.upperLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE) - ( self.BASE2HAA_offset_z *  self.s__q_LF_HAA) +  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LF_KFE[2,0] = - self.c__q_LF_HAA *  self.c__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[2,1] =  self.c__q_LF_HAA *  self.s__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[2,2] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_KFE[2,3] = (- self.upperLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LF_HAA);
        
        
        
        self.fr_trunk_Xh_RF_foot[0,0] = ( self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[0,2] = ( self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[0,3] = (- self.lowerLegLength *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.s__q_RF_HFE *  self.c__q_RF_KFE) - ( self.upperLegLength *  self.s__q_RF_HFE) +  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_RF_foot[1,0] = ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[1,1] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_RF_foot[1,2] = ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[1,3] = (- self.lowerLegLength *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) + ( self.lowerLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) + ( self.upperLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_RF_foot[2,0] = (- self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[2,1] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_RF_foot[2,2] = ( self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[2,3] = ( self.lowerLegLength *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.upperLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_RF_HAA);
        
        
        
        self.fr_trunk_Xh_fr_RF_HFE[1,0] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[1,2] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[1,3] = ( self.BASE2HAA_offset_z *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RF_HFE[2,0] = - self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[2,2] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[2,3] = - self.BASE2HAA_offset_z *  self.c__q_RF_HAA;
        
        
        
        self.fr_trunk_Xh_fr_RF_KFE[0,0] = - self.s__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[0,1] = - self.c__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[0,3] =  self.BASE2HAA_offset_x - ( self.upperLegLength *  self.s__q_RF_HFE);
        self.fr_trunk_Xh_fr_RF_KFE[1,0] =  self.s__q_RF_HAA *  self.c__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[1,1] = - self.s__q_RF_HAA *  self.s__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[1,2] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_KFE[1,3] = ( self.upperLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RF_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RF_KFE[2,0] = - self.c__q_RF_HAA *  self.c__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[2,1] =  self.c__q_RF_HAA *  self.s__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[2,2] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_KFE[2,3] = (- self.upperLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_RF_HAA);
        
        
        
        self.fr_trunk_Xh_LH_foot[0,0] = ( self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[0,2] = ( self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[0,3] = (- self.lowerLegLength *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.s__q_LH_HFE *  self.c__q_LH_KFE) - ( self.upperLegLength *  self.s__q_LH_HFE) -  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_LH_foot[1,0] = (- self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[1,1] =  self.c__q_LH_HAA;
        self.fr_trunk_Xh_LH_foot[1,2] = ( self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[1,3] = ( self.lowerLegLength *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.upperLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE) - ( self.BASE2HAA_offset_z *  self.s__q_LH_HAA) +  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_LH_foot[2,0] = (- self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[2,1] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_LH_foot[2,2] = ( self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[2,3] = ( self.lowerLegLength *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.upperLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LH_HAA);
        
        
        
        self.fr_trunk_Xh_fr_LH_HFE[1,0] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[1,2] =  self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[1,3] =  self.BASE2HAA_offset_y - ( self.BASE2HAA_offset_z *  self.s__q_LH_HAA);
        self.fr_trunk_Xh_fr_LH_HFE[2,0] = - self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[2,2] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[2,3] = - self.BASE2HAA_offset_z *  self.c__q_LH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_LH_KFE[0,0] = - self.s__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[0,1] = - self.c__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[0,3] = (- self.upperLegLength *  self.s__q_LH_HFE) -  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_LH_KFE[1,0] = - self.s__q_LH_HAA *  self.c__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[1,1] =  self.s__q_LH_HAA *  self.s__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[1,2] =  self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_KFE[1,3] = (- self.upperLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE) - ( self.BASE2HAA_offset_z *  self.s__q_LH_HAA) +  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_LH_KFE[2,0] = - self.c__q_LH_HAA *  self.c__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[2,1] =  self.c__q_LH_HAA *  self.s__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[2,2] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_KFE[2,3] = (- self.upperLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LH_HAA);
        
        
        
        self.fr_trunk_Xh_RH_foot[0,0] = ( self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[0,2] = ( self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[0,3] = (- self.lowerLegLength *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.s__q_RH_HFE *  self.c__q_RH_KFE) - ( self.upperLegLength *  self.s__q_RH_HFE) -  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_RH_foot[1,0] = ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[1,1] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_RH_foot[1,2] = ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[1,3] = (- self.lowerLegLength *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) + ( self.lowerLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) + ( self.upperLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_RH_foot[2,0] = (- self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[2,1] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_RH_foot[2,2] = ( self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[2,3] = ( self.lowerLegLength *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.upperLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_RH_HAA);
        
        
        
        self.fr_trunk_Xh_fr_RH_HFE[1,0] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[1,2] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[1,3] = ( self.BASE2HAA_offset_z *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RH_HFE[2,0] = - self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[2,2] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[2,3] = - self.BASE2HAA_offset_z *  self.c__q_RH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_RH_KFE[0,0] = - self.s__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[0,1] = - self.c__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[0,3] = (- self.upperLegLength *  self.s__q_RH_HFE) -  self.BASE2HAA_offset_x;
        self.fr_trunk_Xh_fr_RH_KFE[1,0] =  self.s__q_RH_HAA *  self.c__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[1,1] = - self.s__q_RH_HAA *  self.s__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[1,2] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_KFE[1,3] = ( self.upperLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RH_HAA) -  self.BASE2HAA_offset_y;
        self.fr_trunk_Xh_fr_RH_KFE[2,0] = - self.c__q_RH_HAA *  self.c__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[2,1] =  self.c__q_RH_HAA *  self.s__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[2,2] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_KFE[2,3] = (- self.upperLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_RH_HAA);
        
        self.fr_LF_hipassembly_Xh_fr_trunk[0,1] = - self.s__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[0,3] =  self.BASE2HAA_offset_y *  self.s__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[1,1] = - self.c__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_LF_HAA;
        
        self.fr_trunk_Xh_fr_LF_hipassembly[1,0] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_hipassembly[1,1] = - self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_hipassembly[2,0] = - self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_hipassembly[2,1] =  self.s__q_LF_HAA;
        
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[0,0] =  self.c__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[0,2] =  self.s__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[0,3] = - self.BASE2HAA_offset_z *  self.c__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[1,0] = - self.s__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[1,2] =  self.c__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[1,3] =  self.BASE2HAA_offset_z *  self.s__q_LF_HFE;
        
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[0,0] =  self.c__q_LF_HFE;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[0,1] = - self.s__q_LF_HFE;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[2,0] =  self.s__q_LF_HFE;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[2,1] =  self.c__q_LF_HFE;
        
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[0,0] =  self.c__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[0,1] =  self.s__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[0,3] = - self.upperLegLength *  self.c__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[1,0] = - self.s__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[1,1] =  self.c__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[1,3] =  self.upperLegLength *  self.s__q_LF_KFE;
        
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[0,0] =  self.c__q_LF_KFE;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[0,1] = - self.s__q_LF_KFE;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[1,0] =  self.s__q_LF_KFE;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[1,1] =  self.c__q_LF_KFE;
        
        self.fr_RF_hipassembly_Xh_fr_trunk[0,1] =  self.s__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[0,3] =  self.BASE2HAA_offset_y *  self.s__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[1,1] =  self.c__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_RF_HAA;
        
        self.fr_trunk_Xh_fr_RF_hipassembly[1,0] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_hipassembly[1,1] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_hipassembly[2,0] = - self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_hipassembly[2,1] =  self.s__q_RF_HAA;
        
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[0,0] =  self.c__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[0,2] = - self.s__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[0,3] = - self.BASE2HAA_offset_z *  self.c__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[1,0] = - self.s__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[1,2] = - self.c__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[1,3] =  self.BASE2HAA_offset_z *  self.s__q_RF_HFE;
        
        
        
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[0,0] =  self.c__q_RF_HFE;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[0,1] = - self.s__q_RF_HFE;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[2,0] = - self.s__q_RF_HFE;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[2,1] = - self.c__q_RF_HFE;
        
        
        
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[0,0] =  self.c__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[0,1] =  self.s__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[0,3] = - self.upperLegLength *  self.c__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[1,0] = - self.s__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[1,1] =  self.c__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[1,3] =  self.upperLegLength *  self.s__q_RF_KFE;
        
        
        
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[0,0] =  self.c__q_RF_KFE;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[0,1] = - self.s__q_RF_KFE;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[1,0] =  self.s__q_RF_KFE;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[1,1] =  self.c__q_RF_KFE;
        
        
        
        self.fr_LH_hipassembly_Xh_fr_trunk[0,1] = - self.s__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[0,3] =  self.BASE2HAA_offset_y *  self.s__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[1,1] = - self.c__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_LH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_LH_hipassembly[1,0] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_hipassembly[1,1] = - self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_hipassembly[2,0] = - self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_hipassembly[2,1] =  self.s__q_LH_HAA;
        
        
        
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[0,0] =  self.c__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[0,2] =  self.s__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[0,3] = - self.BASE2HAA_offset_z *  self.c__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[1,0] = - self.s__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[1,2] =  self.c__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[1,3] =  self.BASE2HAA_offset_z *  self.s__q_LH_HFE;
        
        
        
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[0,0] =  self.c__q_LH_HFE;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[0,1] = - self.s__q_LH_HFE;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[2,0] =  self.s__q_LH_HFE;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[2,1] =  self.c__q_LH_HFE;
        
        
        
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[0,0] =  self.c__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[0,1] =  self.s__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[0,3] = - self.upperLegLength *  self.c__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[1,0] = - self.s__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[1,1] =  self.c__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[1,3] =  self.upperLegLength *  self.s__q_LH_KFE;
        
        
        
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[0,0] =  self.c__q_LH_KFE;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[0,1] = - self.s__q_LH_KFE;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[1,0] =  self.s__q_LH_KFE;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[1,1] =  self.c__q_LH_KFE;
        
        
        
        self.fr_RH_hipassembly_Xh_fr_trunk[0,1] =  self.s__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[0,3] =  self.BASE2HAA_offset_y *  self.s__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[1,1] =  self.c__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[1,3] =  self.BASE2HAA_offset_y *  self.c__q_RH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_RH_hipassembly[1,0] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_hipassembly[1,1] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_hipassembly[2,0] = - self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_hipassembly[2,1] =  self.s__q_RH_HAA;
        
        
        
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[0,0] =  self.c__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[0,2] = - self.s__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[0,3] = - self.BASE2HAA_offset_z *  self.c__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[1,0] = - self.s__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[1,2] = - self.c__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[1,3] =  self.BASE2HAA_offset_z *  self.s__q_RH_HFE;
        
        
        
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[0,0] =  self.c__q_RH_HFE;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[0,1] = - self.s__q_RH_HFE;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[2,0] = - self.s__q_RH_HFE;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[2,1] = - self.c__q_RH_HFE;
        
        
        
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[0,0] =  self.c__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[0,1] =  self.s__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[0,3] = - self.upperLegLength *  self.c__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[1,0] = - self.s__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[1,1] =  self.c__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[1,3] =  self.upperLegLength *  self.s__q_RH_KFE;
        
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[0,0] =  self.c__q_RH_KFE;
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[0,1] = - self.s__q_RH_KFE;
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[1,0] =  self.s__q_RH_KFE;
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[1,1] =  self.c__q_RH_KFE;

    def update_jacobians(self, q):
        self.s__q_LF_HAA = np.sin(q[1-1]);
        self.s__q_LF_HFE = np.sin(q[2-1]);
        self.s__q_LF_KFE = np.sin(q[3-1]);
        self.c__q_LF_HAA = np.cos(q[1-1]);
        self.c__q_LF_HFE = np.cos(q[2-1]);
        self.c__q_LF_KFE = np.cos(q[3-1]);

        self.fr_trunk_J_LF_foot[2-1,2-1] =  self.c__q_LF_HAA;
        self.fr_trunk_J_LF_foot[2-1,3-1] =  self.c__q_LF_HAA;
        self.fr_trunk_J_LF_foot[3-1,2-1] = - self.s__q_LF_HAA;
        self.fr_trunk_J_LF_foot[3-1,3-1] = - self.s__q_LF_HAA;
        self.fr_trunk_J_LF_foot[4-1,2-1] = ( self.lowerLegLength *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.upperLegLength *  self.c__q_LF_HFE);
        self.fr_trunk_J_LF_foot[4-1,3-1] = ( self.lowerLegLength *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.c__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_J_LF_foot[5-1,1-1] = ( self.lowerLegLength *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( self.lowerLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.upperLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LF_HAA);
        self.fr_trunk_J_LF_foot[5-1,2-1] = ( self.lowerLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.lowerLegLength *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE) + ( self.upperLegLength *  self.s__q_LF_HAA *  self.s__q_LF_HFE);
        self.fr_trunk_J_LF_foot[5-1,3-1] = ( self.lowerLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.lowerLegLength *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_J_LF_foot[6-1,1-1] = (- self.lowerLegLength *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) + ( self.lowerLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) + ( self.upperLegLength *  self.s__q_LF_HAA *  self.c__q_LF_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_LF_HAA);
        self.fr_trunk_J_LF_foot[6-1,2-1] = ( self.lowerLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.lowerLegLength *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE) + ( self.upperLegLength *  self.c__q_LF_HAA *  self.s__q_LF_HFE);
        self.fr_trunk_J_LF_foot[6-1,3-1] = ( self.lowerLegLength *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.lowerLegLength *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);

        self.s__q_RF_HAA = np.sin(q[4-1]);
        self.s__q_RF_HFE = np.sin(q[5-1]);
        self.s__q_RF_KFE = np.sin(q[6-1]);
        self.c__q_RF_HAA = np.cos(q[4-1]);
        self.c__q_RF_HFE = np.cos(q[5-1]);
        self.c__q_RF_KFE = np.cos(q[6-1]);

        self.fr_trunk_J_RF_foot[2-1,2-1] =  self.c__q_RF_HAA;
        self.fr_trunk_J_RF_foot[2-1,3-1] =  self.c__q_RF_HAA;
        self.fr_trunk_J_RF_foot[3-1,2-1] =  self.s__q_RF_HAA;
        self.fr_trunk_J_RF_foot[3-1,3-1] =  self.s__q_RF_HAA;
        self.fr_trunk_J_RF_foot[4-1,2-1] = ( self.lowerLegLength *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.upperLegLength *  self.c__q_RF_HFE);
        self.fr_trunk_J_RF_foot[4-1,3-1] = ( self.lowerLegLength *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.c__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_J_RF_foot[5-1,1-1] = (- self.lowerLegLength *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) + ( self.lowerLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) + ( self.upperLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE) + ( self.BASE2HAA_offset_z *  self.c__q_RF_HAA);
        self.fr_trunk_J_RF_foot[5-1,2-1] = (- self.lowerLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE) - ( self.upperLegLength *  self.s__q_RF_HAA *  self.s__q_RF_HFE);
        self.fr_trunk_J_RF_foot[5-1,3-1] = (- self.lowerLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.lowerLegLength *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_J_RF_foot[6-1,1-1] = (- self.lowerLegLength *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) + ( self.lowerLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) + ( self.upperLegLength *  self.s__q_RF_HAA *  self.c__q_RF_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RF_HAA);
        self.fr_trunk_J_RF_foot[6-1,2-1] = ( self.lowerLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.lowerLegLength *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE) + ( self.upperLegLength *  self.c__q_RF_HAA *  self.s__q_RF_HFE);
        self.fr_trunk_J_RF_foot[6-1,3-1] = ( self.lowerLegLength *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.lowerLegLength *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);

        self.s__q_LH_HAA = np.sin(q[7-1]);
        self.s__q_LH_HFE = np.sin(q[8-1]);
        self.s__q_LH_KFE = np.sin(q[9-1]);
        self.c__q_LH_HAA = np.cos(q[7-1]);
        self.c__q_LH_HFE = np.cos(q[8-1]);
        self.c__q_LH_KFE = np.cos(q[9-1]);

        self.fr_trunk_J_LH_foot[2-1,2-1] =  self.c__q_LH_HAA;
        self.fr_trunk_J_LH_foot[2-1,3-1] =  self.c__q_LH_HAA;
        self.fr_trunk_J_LH_foot[3-1,2-1] = - self.s__q_LH_HAA;
        self.fr_trunk_J_LH_foot[3-1,3-1] = - self.s__q_LH_HAA;
        self.fr_trunk_J_LH_foot[4-1,2-1] = ( self.lowerLegLength *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.upperLegLength *  self.c__q_LH_HFE);
        self.fr_trunk_J_LH_foot[4-1,3-1] = ( self.lowerLegLength *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.c__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_J_LH_foot[5-1,1-1] = ( self.lowerLegLength *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( self.lowerLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.upperLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE) - ( self.BASE2HAA_offset_z *  self.c__q_LH_HAA);
        self.fr_trunk_J_LH_foot[5-1,2-1] = ( self.lowerLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.lowerLegLength *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE) + ( self.upperLegLength *  self.s__q_LH_HAA *  self.s__q_LH_HFE);
        self.fr_trunk_J_LH_foot[5-1,3-1] = ( self.lowerLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.lowerLegLength *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_J_LH_foot[6-1,1-1] = (- self.lowerLegLength *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) + ( self.lowerLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) + ( self.upperLegLength *  self.s__q_LH_HAA *  self.c__q_LH_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_LH_HAA);
        self.fr_trunk_J_LH_foot[6-1,2-1] = ( self.lowerLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.lowerLegLength *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE) + ( self.upperLegLength *  self.c__q_LH_HAA *  self.s__q_LH_HFE);
        self.fr_trunk_J_LH_foot[6-1,3-1] = ( self.lowerLegLength *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.lowerLegLength *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);

        self.s__q_RH_HAA = np.sin(q[10-1]);
        self.s__q_RH_HFE = np.sin(q[11-1]);
        self.s__q_RH_KFE = np.sin(q[12-1]);
        self.c__q_RH_HAA = np.cos(q[10-1]);
        self.c__q_RH_HFE = np.cos(q[11-1]);
        self.c__q_RH_KFE = np.cos(q[12-1]);

        self.fr_trunk_J_RH_foot[2-1,2-1] =  self.c__q_RH_HAA;
        self.fr_trunk_J_RH_foot[2-1,3-1] =  self.c__q_RH_HAA;
        self.fr_trunk_J_RH_foot[3-1,2-1] =  self.s__q_RH_HAA;
        self.fr_trunk_J_RH_foot[3-1,3-1] =  self.s__q_RH_HAA;
        self.fr_trunk_J_RH_foot[4-1,2-1] = ( self.lowerLegLength *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.upperLegLength *  self.c__q_RH_HFE);
        self.fr_trunk_J_RH_foot[4-1,3-1] = ( self.lowerLegLength *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.c__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_J_RH_foot[5-1,1-1] = (- self.lowerLegLength *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) + ( self.lowerLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) + ( self.upperLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE) + ( self.BASE2HAA_offset_z *  self.c__q_RH_HAA);
        self.fr_trunk_J_RH_foot[5-1,2-1] = (- self.lowerLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE) - ( self.upperLegLength *  self.s__q_RH_HAA *  self.s__q_RH_HFE);
        self.fr_trunk_J_RH_foot[5-1,3-1] = (- self.lowerLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.lowerLegLength *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_J_RH_foot[6-1,1-1] = (- self.lowerLegLength *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) + ( self.lowerLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) + ( self.upperLegLength *  self.s__q_RH_HAA *  self.c__q_RH_HFE) + ( self.BASE2HAA_offset_z *  self.s__q_RH_HAA);
        self.fr_trunk_J_RH_foot[6-1,2-1] = ( self.lowerLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.lowerLegLength *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE) + ( self.upperLegLength *  self.c__q_RH_HAA *  self.s__q_RH_HFE);
        self.fr_trunk_J_RH_foot[6-1,3-1] = ( self.lowerLegLength *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.lowerLegLength *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        #print self.fr_trunk_J_LF_foot[3:6,:]
        return self.fr_trunk_J_LF_foot[3:6,:] , self.fr_trunk_J_RF_foot[3:6,:], self.fr_trunk_J_LH_foot[3:6,:], self.fr_trunk_J_RH_foot[3:6,:]


    def forward_kin(self, q):
        LF_foot = self.fr_trunk_Xh_LF_foot[0:3,3]
        RF_foot = self.fr_trunk_Xh_RF_foot[0:3,3]
        LH_foot = self.fr_trunk_Xh_LH_foot[0:3,3]
        RH_foot = self.fr_trunk_Xh_RH_foot[0:3,3]
        
        contacts = np.vstack((LF_foot,RF_foot,LH_foot,RH_foot))
        
        return contacts
        
    def inverse_kin(self, x, x_dot, y, y_dot, z, z_dot, verbose = False):
        ''' 
        This function computes the joint positions given the feet positions and velocities.
        Only the X Y feet coordinates are considered inthis version.
        Besides the joint positions, this function also returns the 2D jacobians referred to the
        HFE and KFE joints of the HyQ robot.
        '''
        self.isOutOfWorkSpace = False
        footPosDes_xz = np.vstack([x,z])
        BASE2HAA_offsets_xz = np.array([[self.BASE2HAA_offset_x,self.BASE2HAA_offset_x,-self.BASE2HAA_offset_x,-self.BASE2HAA_offset_x],
                                 [-self.BASE2HAA_offset_z, -self.BASE2HAA_offset_z, -self.BASE2HAA_offset_z, -self.BASE2HAA_offset_z]]);

        footPosHAA = np.subtract(footPosDes_xz, BASE2HAA_offsets_xz)
        #print footPosHAA

        y[0] = y[0]-self.BASE2HAA_offset_y
        y[1] = y[1]+self.BASE2HAA_offset_y
        y[2] = y[2]-self.BASE2HAA_offset_y
        y[3] = y[3]+self.BASE2HAA_offset_y
        
#        haa2hfeLength = 0.045;
        haa2hfeLength = 0.0;
        M_PI = np.pi;
    
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

        q[0] = -np.arctan2(y[0],-footPosHAA[1,0]); # LF HAA
        q[6] = -np.arctan2(y[1],-footPosHAA[1,1]); # LH HAA
        q[3] = -np.arctan2(-y[2],-footPosHAA[1,2]);# RF HAA
        q[9] = -np.arctan2(-y[3],-footPosHAA[1,3]);# RH HAA

        #q[0] = -np.arctan2(footPosHAA[0,0],-footPosHAA[1,0]); # LF HAA
        #q[6] = -np.arctan2(footPosHAA[0,1],-footPosHAA[1,1]); # LH HAA
        #q[3] = -np.arctan2(-footPosHAA[0,2],-footPosHAA[1,2]);# RF HAA
        #q[9] = -np.arctan2(-footPosHAA[0,3],-footPosHAA[1,3]);# RH HAA
    
        # HFE and KFE joints (use cosine law)
        cos_arg = (self.upperLegLength * self.upperLegLength + self.lowerLegLength * self.lowerLegLength - hfe2foot * hfe2foot) / (2 * self.upperLegLength * self.lowerLegLength);
        q[2] = - M_PI + np.arccos(cos_arg[0]); # LF KFE
        q[5] = - M_PI + np.arccos(cos_arg[0]); # RF KFE
        cos_arg = (np.square(self.upperLegLength) + np.square(hfe2foot) - np.square(self.lowerLegLength)) / (2 * self.upperLegLength * hfe2foot);
        sin_arg = footPosHAA[0] / hfe2foot; #it should be footPosHFE(rbd::X)/hfe2foot but footPosHFE(rbd::X) = footPosHAA(rbd::X)	
        q[1] = -np.arcsin(sin_arg[0]) + np.arccos(cos_arg[0]);# LF HFE
        if (np.isnan(q[1])):
            self.isOutOfWorkSpace = True
            if verbose:
                print 'Warning! point is out of workspace!'
        q[4] = -np.arcsin(sin_arg[0]) + np.arccos(cos_arg[0]);# RF HFE
        if (np.isnan(q[4])):
            self.isOutOfWorkSpace = True
            if verbose:
                print 'Warning! point is out of workspace!'    
        cos_arg = (self.upperLegLength * self.upperLegLength + self.lowerLegLength * self.lowerLegLength - hfe2foot * hfe2foot)/ (2 * self.upperLegLength * self.lowerLegLength);
        q[8]= + M_PI- np.arccos(cos_arg[0]); # LH KFE
        q[11] = + M_PI - np.arccos(cos_arg[0]); # RH KFE
        cos_arg = (self.upperLegLength * self.upperLegLength + hfe2foot * hfe2foot- self.lowerLegLength * self.lowerLegLength) / (2 * self.upperLegLength * hfe2foot);
        sin_arg = footPosHAA[0,2] / hfe2foot; # it should be footPosHFE(rbd::X)/hfe2foot but footPosHFE(rbd::X) = footPosHAA(rbd::X)
        q[7] = -np.arcsin(sin_arg[0])- np.arccos(cos_arg[0]);# LH HFE
        if (np.isnan(q[7])):
            self.isOutOfWorkSpace = True
            if verbose:
                print 'Warning! point is out of workspace!'
        q[10] = -np.arcsin(sin_arg[0])- np.arccos(cos_arg[0]);# RH HFE
        if (np.isnan(q[10])):
            self.isOutOfWorkSpace = True
            if verbose:
                print 'Warning! point is out of workspace!'    

        """ compute joint velocities updating the 2D jacobians with the computed position """
        l1 = self.upperLegLength;
        l2 = self.lowerLegLength;
        Jac_LF_2D = np.array([[np.asscalar(-l1*np.cos([q[1]]) - l2 * np.cos([q[1] + q[2]])),np.asscalar( - l2 * np.cos([q[1] + q[2]]))],
                            [np.asscalar(l1*np.sin([q[1]]) + l2 * np.sin([q[1] + q[2]])),np.asscalar(   l2 * np.sin([q[1] + q[2]]))]])
        
        Jac_RF_2D = np.array([[np.asscalar(l1*np.cos(q[4]) + l2 * np.cos(q[4]+q[5])), np.asscalar( l2 * np.cos(q[4] + q[4]))],
                            [np.asscalar(l1*np.sin(q[4]) + l2 * np.sin(q[4]+q[5])), np.asscalar( l2 * np.sin(q[4] + q[4]))]])
                            
        Jac_LH_2D = np.array([[np.asscalar(-l1*np.cos(q[7]) - l2 * np.cos(q[7]+q[8])), np.asscalar( -l2 * np.cos(q[7] + q[8]))],
                            [np.asscalar(l1*np.sin(q[7]) + l2 * np.sin(q[7]+q[8])), np.asscalar(  l2 * np.sin(q[7] + q[8]))]])
                            
        Jac_RH_2D = np.array([[np.asscalar(l1*np.cos(q[10]) + l2 * np.cos(q[10]+q[11])), np.asscalar( l2 * np.cos(q[10] + q[11]))],
                           [np.asscalar(l1*np.sin(q[10]) + l2 * np.sin(q[10]+q[11])), np.asscalar( l2 * np.sin(q[10] + q[11]))]])     

        #footVelDes = np.vstack([x_dot, z_dot]); 
        
        #print footVelDes
        #q_dot[1:2] = np.linalg.inv(Jac_LF)*footVelDes;
        #q_dot[4:5] = np.linalg.inv(Jac_RF)*footVelDes;
        #q_dot[7:8] = np.linalg.inv(Jac_LH)*footVelDes;
        #q_dot[10:11] = np.linalg.inv(Jac_RH)*footVelDes;   
        return q, q_dot, Jac_LF_2D, Jac_RF_2D, Jac_LH_2D, Jac_RH_2D, self.isOutOfWorkSpace




