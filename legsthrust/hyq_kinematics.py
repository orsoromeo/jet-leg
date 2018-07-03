# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 05:34:42 2018

@author: romeo orsolino
"""
import numpy as np

class HyQKinematics:
    def __init__(self):
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
        self.fr_LF_lowerleg_Xh_LF_foot[0,3] = 0.341;
        self.fr_LF_lowerleg_Xh_LF_foot[1,0] = - 1;
        self.fr_LF_lowerleg_Xh_LF_foot[2,1] = 1;
        self.fr_LF_lowerleg_Xh_LF_foot[3,3] = 1;	
               
        self.fr_RF_lowerleg_Xh_RF_foot[0,2] = - 1.0;
        self.fr_RF_lowerleg_Xh_RF_foot[0,3] = 0.341;
        self.fr_RF_lowerleg_Xh_RF_foot[1,0] = - 1;
        self.fr_RF_lowerleg_Xh_RF_foot[2,1] = 1;
        self.fr_RF_lowerleg_Xh_RF_foot[3,3] = 1;	
        
        self.fr_LH_lowerleg_Xh_LH_foot[0,2] = - 1.0;
        self.fr_LH_lowerleg_Xh_LH_foot[0,3] = 0.341;
        self.fr_LH_lowerleg_Xh_LH_foot[1,0] = - 1;
        self.fr_LH_lowerleg_Xh_LH_foot[2,1] = 1;
        self.fr_LH_lowerleg_Xh_LH_foot[3,3] = 1;	
        
        self.fr_RH_lowerleg_Xh_RH_foot[0,2] = - 1.0;
        self.fr_RH_lowerleg_Xh_RH_foot[0,3] = 0.341;
        self.fr_RH_lowerleg_Xh_RH_foot[1,0] = - 1;
        self.fr_RH_lowerleg_Xh_RH_foot[2,1] = 1;
        self.fr_RH_lowerleg_Xh_RH_foot[3,3] = 1;	
        
        self.LF_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        self.RF_foot_Xh_fr_trunk[3,3]= 1.0;	
        
        self.LH_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        self.RH_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_LF_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LF_HAA[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HAA[0,3] = 0.3735;
        self.fr_trunk_Xh_fr_LF_HAA[1,1] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HAA[1,3] = 0.207;
        self.fr_trunk_Xh_fr_LF_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LF_HFE[0,1] = - 1.0;
        self.fr_trunk_Xh_fr_LF_HFE[0,3] = 0.3735;
        self.fr_trunk_Xh_fr_LF_HFE[3,3] = 1.0;	
         
        self.fr_trunk_Xh_fr_LF_KFE[3,3] = 1.0;	  
         
        self.fr_trunk_Xh_RF_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_HAA[0,2] = 1.0;
        self.fr_trunk_Xh_fr_RF_HAA[0,3] = 0.3735;
        self.fr_trunk_Xh_fr_RF_HAA[1,1] = 1.0;
        self.fr_trunk_Xh_fr_RF_HAA[1,3] = - 0.207;
        self.fr_trunk_Xh_fr_RF_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_RF_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_HFE[0,1] = - 1.0;
        self.fr_trunk_Xh_fr_RF_HFE[0,3] = 0.3735;
        self.fr_trunk_Xh_fr_RF_HFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_KFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_LH_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_HAA[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HAA[0,3] = - 0.3735;
        self.fr_trunk_Xh_fr_LH_HAA[1,1] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HAA[1,3] = 0.207;
        self.fr_trunk_Xh_fr_LH_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_HFE[0,0] = - 1.0;
        self.fr_trunk_Xh_fr_LH_HFE[0,3] = - 0.3735;
        self.fr_trunk_Xh_fr_LH_HFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_KFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_RH_foot[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_HAA[0,2] = 1.0;
        self.fr_trunk_Xh_fr_RH_HAA[0,3] = - 0.3735;
        self.fr_trunk_Xh_fr_RH_HAA[1,1] = 1.0;
        self.fr_trunk_Xh_fr_RH_HAA[1,3] = - 0.207;
        self.fr_trunk_Xh_fr_RH_HAA[2,0] = - 1.0;
        self.fr_trunk_Xh_fr_RH_HAA[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_HFE[0,1] = - 1.0;
        self.fr_trunk_Xh_fr_RH_HFE[0,3] = - 0.3735;
        self.fr_trunk_Xh_fr_RH_HFE[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_KFE[3,3] = 1.0;	
        
        self.fr_LF_hipassembly_Xh_fr_trunk[2,0] = - 1.0;
        self.fr_LF_hipassembly_Xh_fr_trunk[2,3] = 0.3735;
        self.fr_LF_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LF_hipassembly[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LF_hipassembly[0,3] = 0.3735;
        self.fr_trunk_Xh_fr_LF_hipassembly[1,3] = 0.207;
        self.fr_trunk_Xh_fr_LF_hipassembly[3,3] = 1.0;	
            
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[2,1] = - 1;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[3,3] = 1.0;	
     
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[0,3] = 0.08;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[1,2] = - 1;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[3,3] = 1;	
        
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[2,2] = 1;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[3,3] = 1.0;	
        
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[0,3] = 0.35;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[2,2] = 1;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[3,3] = 1;	
        
        self.fr_RF_hipassembly_Xh_fr_trunk[2,0] = 1.0;
        self.fr_RF_hipassembly_Xh_fr_trunk[2,3] = - 0.3735;
        self.fr_RF_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RF_hipassembly[0,2] = 1.0;
        self.fr_trunk_Xh_fr_RF_hipassembly[0,3] = 0.3735;
        self.fr_trunk_Xh_fr_RF_hipassembly[1,3] = - 0.207;
        self.fr_trunk_Xh_fr_RF_hipassembly[3,3] = 1.0;	
        
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[2,1] = 1;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[3,3] = 1.0;	
        
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[0,3] = 0.08;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[1,2] = 1;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[3,3] = 1;	
        
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[2,2] = 1;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[3,3] = 1.0;	
        
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[0,3] = 0.35;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[2,2] = 1;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[3,3] = 1;	
        
        self.fr_LH_hipassembly_Xh_fr_trunk[2,0] = - 1.0;
        self.fr_LH_hipassembly_Xh_fr_trunk[2,3] = - 0.3735;
        self.fr_LH_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_LH_hipassembly[0,2] = - 1.0;
        self.fr_trunk_Xh_fr_LH_hipassembly[0,3] = - 0.3735;
        self.fr_trunk_Xh_fr_LH_hipassembly[1,3] = 0.207;
        self.fr_trunk_Xh_fr_LH_hipassembly[3,3] = 1.0;	
        
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[2,1] = - 1;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[3,3] = 1.0;	
        
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[0,3] = 0.08;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[1,2] = - 1;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[3,3] = 1;	
        
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[2,2]= 1;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[3,3]= 1.0;	
        
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[0,3] = 0.35;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[2,2] = 1;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[3,3] = 1;	
        

        self.fr_RH_hipassembly_Xh_fr_trunk[2,0] = 1.0;
        self.fr_RH_hipassembly_Xh_fr_trunk[2,3] = 0.3735;
        self.fr_RH_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        self.fr_trunk_Xh_fr_RH_hipassembly[0,2]= 1.0;
        self.fr_trunk_Xh_fr_RH_hipassembly[0,3]= - 0.3735;
        self.fr_trunk_Xh_fr_RH_hipassembly[1,3]= - 0.207;
        self.fr_trunk_Xh_fr_RH_hipassembly[3,3]= 1.0;	
        
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[2,1]= 1;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[3,3]= 1.0;	
        
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[0,3] = 0.08;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[1,2] = 1;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[3,3] = 1;	
        
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[2,2] = 1;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[3,3] = 1.0;
        
        self.fr_RH_upperleg_Xh_fr_RH_lowerleg[0,3] = 0.35;
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
        self.LF_foot_Xh_fr_trunk[0,3] = ((( 0.3735 *  self.s__q_LF_HFE) + ((( 0.207 *  self.s__q_LF_HAA) -  0.08) *  self.c__q_LF_HFE) -  0.35) *  self.s__q_LF_KFE) + ((((( 0.207 *  self.s__q_LF_HAA) -  0.08) *  self.s__q_LF_HFE) - ( 0.3735 *  self.c__q_LF_HFE)) *  self.c__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[1,1] =  self.c__q_LF_HAA;
        self.LF_foot_Xh_fr_trunk[1,2] = - self.s__q_LF_HAA;
        self.LF_foot_Xh_fr_trunk[1,3] = - 0.207 *  self.c__q_LF_HAA;
        self.LF_foot_Xh_fr_trunk[2,0] = ( self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[2,1] = ( self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[2,2] = ( self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.LF_foot_Xh_fr_trunk[2,3] = ((((( 0.207 *  self.s__q_LF_HAA) -  0.08) *  self.s__q_LF_HFE) - ( 0.3735 *  self.c__q_LF_HFE)) *  self.s__q_LF_KFE) + (((- 0.3735 *  self.s__q_LF_HFE) + (( 0.08 - ( 0.207 *  self.s__q_LF_HAA)) *  self.c__q_LF_HFE) +  0.35) *  self.c__q_LF_KFE) +  0.341;
        
        
        
        self.RF_foot_Xh_fr_trunk[0,0] = ( self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[0,1] = ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[0,2] = (- self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[0,3] = ((( 0.3735 *  self.s__q_RF_HFE) + ((( 0.207 *  self.s__q_RF_HAA) -  0.08) *  self.c__q_RF_HFE) -  0.35) *  self.s__q_RF_KFE) + ((((( 0.207 *  self.s__q_RF_HAA) -  0.08) *  self.s__q_RF_HFE) - ( 0.3735 *  self.c__q_RF_HFE)) *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[1,1] =  self.c__q_RF_HAA;
        self.RF_foot_Xh_fr_trunk[1,2] =  self.s__q_RF_HAA;
        self.RF_foot_Xh_fr_trunk[1,3] =  0.207 *  self.c__q_RF_HAA;
        self.RF_foot_Xh_fr_trunk[2,0] = ( self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[2,1] = ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[2,2] = ( self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.RF_foot_Xh_fr_trunk[2,3] = ((((( 0.207 *  self.s__q_RF_HAA) -  0.08) *  self.s__q_RF_HFE) - ( 0.3735 *  self.c__q_RF_HFE)) *  self.s__q_RF_KFE) + (((- 0.3735 *  self.s__q_RF_HFE) + (( 0.08 - ( 0.207 *  self.s__q_RF_HAA)) *  self.c__q_RF_HFE) +  0.35) *  self.c__q_RF_KFE) +  0.341;
        
        
        
        self.LH_foot_Xh_fr_trunk[0,0] = ( self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[0,1] = (- self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[0,2] = (- self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[0,3] = (((- 0.3735 *  self.s__q_LH_HFE) + ((( 0.207 *  self.s__q_LH_HAA) -  0.08) *  self.c__q_LH_HFE) -  0.35) *  self.s__q_LH_KFE) + ((((( 0.207 *  self.s__q_LH_HAA) -  0.08) *  self.s__q_LH_HFE) + ( 0.3735 *  self.c__q_LH_HFE)) *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[1,1] =  self.c__q_LH_HAA;
        self.LH_foot_Xh_fr_trunk[1,2] = - self.s__q_LH_HAA;
        self.LH_foot_Xh_fr_trunk[1,3] = - 0.207 *  self.c__q_LH_HAA;
        self.LH_foot_Xh_fr_trunk[2,0] = ( self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[2,1] = ( self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[2,2] = ( self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.LH_foot_Xh_fr_trunk[2,3] = ((((( 0.207 *  self.s__q_LH_HAA) -  0.08) *  self.s__q_LH_HFE) + ( 0.3735 *  self.c__q_LH_HFE)) *  self.s__q_LH_KFE) + ((( 0.3735 *  self.s__q_LH_HFE) + (( 0.08 - ( 0.207 *  self.s__q_LH_HAA)) *  self.c__q_LH_HFE) +  0.35) *  self.c__q_LH_KFE) +  0.341;
        
        
        
        self.RH_foot_Xh_fr_trunk[0,0] = ( self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[0,1] = ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[0,2] = (- self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[0,3] = (((- 0.3735 *  self.s__q_RH_HFE) + ((( 0.207 *  self.s__q_RH_HAA) -  0.08) *  self.c__q_RH_HFE) -  0.35) *  self.s__q_RH_KFE) + ((((( 0.207 *  self.s__q_RH_HAA) -  0.08) *  self.s__q_RH_HFE) + ( 0.3735 *  self.c__q_RH_HFE)) *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[1,1] =  self.c__q_RH_HAA;
        self.RH_foot_Xh_fr_trunk[1,2] =  self.s__q_RH_HAA;
        self.RH_foot_Xh_fr_trunk[1,3] =  0.207 *  self.c__q_RH_HAA;
        self.RH_foot_Xh_fr_trunk[2,0] = ( self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[2,1] = ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[2,2] = ( self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.RH_foot_Xh_fr_trunk[2,3] = ((((( 0.207 *  self.s__q_RH_HAA) -  0.08) *  self.s__q_RH_HFE) + ( 0.3735 *  self.c__q_RH_HFE)) *  self.s__q_RH_KFE) + ((( 0.3735 *  self.s__q_RH_HFE) + (( 0.08 - ( 0.207 *  self.s__q_RH_HAA)) *  self.c__q_RH_HFE) +  0.35) *  self.c__q_RH_KFE) +  0.341;
        
        
        
        self.fr_trunk_Xh_LF_foot[0,0] = ( self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[0,2] = ( self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[0,3] = (- 0.341 *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( 0.341 *  self.s__q_LF_HFE *  self.c__q_LF_KFE) - ( 0.35 *  self.s__q_LF_HFE) +  0.3735;
        self.fr_trunk_Xh_LF_foot[1,0] = (- self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[1,1] =  self.c__q_LF_HAA;
        self.fr_trunk_Xh_LF_foot[1,2] = ( self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[1,3] = ( 0.341 *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( 0.341 *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( 0.35 *  self.s__q_LF_HAA *  self.c__q_LF_HFE) - ( 0.08 *  self.s__q_LF_HAA) +  0.207;
        self.fr_trunk_Xh_LF_foot[2,0] = (- self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[2,1] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_LF_foot[2,2] = ( self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE);
        self.fr_trunk_Xh_LF_foot[2,3] = ( 0.341 *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( 0.341 *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( 0.35 *  self.c__q_LF_HAA *  self.c__q_LF_HFE) - ( 0.08 *  self.c__q_LF_HAA);
        
        
        
        self.fr_trunk_Xh_fr_LF_HFE[1,0] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[1,2] =  self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[1,3] =  0.207 - ( 0.08 *  self.s__q_LF_HAA);
        self.fr_trunk_Xh_fr_LF_HFE[2,0] = - self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[2,2] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_HFE[2,3] = - 0.08 *  self.c__q_LF_HAA;
        
        
        
        self.fr_trunk_Xh_fr_LF_KFE[0,0] = - self.s__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[0,1] = - self.c__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[0,3] =  0.3735 - ( 0.35 *  self.s__q_LF_HFE);
        self.fr_trunk_Xh_fr_LF_KFE[1,0] = - self.s__q_LF_HAA *  self.c__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[1,1] =  self.s__q_LF_HAA *  self.s__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[1,2] =  self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_KFE[1,3] = (- 0.35 *  self.s__q_LF_HAA *  self.c__q_LF_HFE) - ( 0.08 *  self.s__q_LF_HAA) +  0.207;
        self.fr_trunk_Xh_fr_LF_KFE[2,0] = - self.c__q_LF_HAA *  self.c__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[2,1] =  self.c__q_LF_HAA *  self.s__q_LF_HFE;
        self.fr_trunk_Xh_fr_LF_KFE[2,2] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_KFE[2,3] = (- 0.35 *  self.c__q_LF_HAA *  self.c__q_LF_HFE) - ( 0.08 *  self.c__q_LF_HAA);
        
        
        
        self.fr_trunk_Xh_RF_foot[0,0] = ( self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[0,2] = ( self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[0,3] = (- 0.341 *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( 0.341 *  self.s__q_RF_HFE *  self.c__q_RF_KFE) - ( 0.35 *  self.s__q_RF_HFE) +  0.3735;
        self.fr_trunk_Xh_RF_foot[1,0] = ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[1,1] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_RF_foot[1,2] = ( self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[1,3] = (- 0.341 *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) + ( 0.341 *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) + ( 0.35 *  self.s__q_RF_HAA *  self.c__q_RF_HFE) + ( 0.08 *  self.s__q_RF_HAA) -  0.207;
        self.fr_trunk_Xh_RF_foot[2,0] = (- self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[2,1] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_RF_foot[2,2] = ( self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE);
        self.fr_trunk_Xh_RF_foot[2,3] = ( 0.341 *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( 0.341 *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( 0.35 *  self.c__q_RF_HAA *  self.c__q_RF_HFE) - ( 0.08 *  self.c__q_RF_HAA);
        
        
        
        self.fr_trunk_Xh_fr_RF_HFE[1,0] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[1,2] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[1,3] = ( 0.08 *  self.s__q_RF_HAA) -  0.207;
        self.fr_trunk_Xh_fr_RF_HFE[2,0] = - self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[2,2] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_HFE[2,3] = - 0.08 *  self.c__q_RF_HAA;
        
        
        
        self.fr_trunk_Xh_fr_RF_KFE[0,0] = - self.s__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[0,1] = - self.c__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[0,3] =  0.3735 - ( 0.35 *  self.s__q_RF_HFE);
        self.fr_trunk_Xh_fr_RF_KFE[1,0] =  self.s__q_RF_HAA *  self.c__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[1,1] = - self.s__q_RF_HAA *  self.s__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[1,2] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_KFE[1,3] = ( 0.35 *  self.s__q_RF_HAA *  self.c__q_RF_HFE) + ( 0.08 *  self.s__q_RF_HAA) -  0.207;
        self.fr_trunk_Xh_fr_RF_KFE[2,0] = - self.c__q_RF_HAA *  self.c__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[2,1] =  self.c__q_RF_HAA *  self.s__q_RF_HFE;
        self.fr_trunk_Xh_fr_RF_KFE[2,2] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_KFE[2,3] = (- 0.35 *  self.c__q_RF_HAA *  self.c__q_RF_HFE) - ( 0.08 *  self.c__q_RF_HAA);
        
        
        
        self.fr_trunk_Xh_LH_foot[0,0] = ( self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[0,2] = ( self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[0,3] = (- 0.341 *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( 0.341 *  self.s__q_LH_HFE *  self.c__q_LH_KFE) - ( 0.35 *  self.s__q_LH_HFE) -  0.3735;
        self.fr_trunk_Xh_LH_foot[1,0] = (- self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[1,1] =  self.c__q_LH_HAA;
        self.fr_trunk_Xh_LH_foot[1,2] = ( self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[1,3] = ( 0.341 *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( 0.341 *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( 0.35 *  self.s__q_LH_HAA *  self.c__q_LH_HFE) - ( 0.08 *  self.s__q_LH_HAA) +  0.207;
        self.fr_trunk_Xh_LH_foot[2,0] = (- self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[2,1] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_LH_foot[2,2] = ( self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE);
        self.fr_trunk_Xh_LH_foot[2,3] = ( 0.341 *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( 0.341 *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( 0.35 *  self.c__q_LH_HAA *  self.c__q_LH_HFE) - ( 0.08 *  self.c__q_LH_HAA);
        
        
        
        self.fr_trunk_Xh_fr_LH_HFE[1,0] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[1,2] =  self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[1,3] =  0.207 - ( 0.08 *  self.s__q_LH_HAA);
        self.fr_trunk_Xh_fr_LH_HFE[2,0] = - self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[2,2] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_HFE[2,3] = - 0.08 *  self.c__q_LH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_LH_KFE[0,0] = - self.s__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[0,1] = - self.c__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[0,3] = (- 0.35 *  self.s__q_LH_HFE) -  0.3735;
        self.fr_trunk_Xh_fr_LH_KFE[1,0] = - self.s__q_LH_HAA *  self.c__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[1,1] =  self.s__q_LH_HAA *  self.s__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[1,2] =  self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_KFE[1,3] = (- 0.35 *  self.s__q_LH_HAA *  self.c__q_LH_HFE) - ( 0.08 *  self.s__q_LH_HAA) +  0.207;
        self.fr_trunk_Xh_fr_LH_KFE[2,0] = - self.c__q_LH_HAA *  self.c__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[2,1] =  self.c__q_LH_HAA *  self.s__q_LH_HFE;
        self.fr_trunk_Xh_fr_LH_KFE[2,2] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_KFE[2,3] = (- 0.35 *  self.c__q_LH_HAA *  self.c__q_LH_HFE) - ( 0.08 *  self.c__q_LH_HAA);
        
        
        
        self.fr_trunk_Xh_RH_foot[0,0] = ( self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[0,2] = ( self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[0,3] = (- 0.341 *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( 0.341 *  self.s__q_RH_HFE *  self.c__q_RH_KFE) - ( 0.35 *  self.s__q_RH_HFE) -  0.3735;
        self.fr_trunk_Xh_RH_foot[1,0] = ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[1,1] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_RH_foot[1,2] = ( self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[1,3] = (- 0.341 *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) + ( 0.341 *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) + ( 0.35 *  self.s__q_RH_HAA *  self.c__q_RH_HFE) + ( 0.08 *  self.s__q_RH_HAA) -  0.207;
        self.fr_trunk_Xh_RH_foot[2,0] = (- self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[2,1] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_RH_foot[2,2] = ( self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE);
        self.fr_trunk_Xh_RH_foot[2,3] = ( 0.341 *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( 0.341 *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( 0.35 *  self.c__q_RH_HAA *  self.c__q_RH_HFE) - ( 0.08 *  self.c__q_RH_HAA);
        
        
        
        self.fr_trunk_Xh_fr_RH_HFE[1,0] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[1,2] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[1,3] = ( 0.08 *  self.s__q_RH_HAA) -  0.207;
        self.fr_trunk_Xh_fr_RH_HFE[2,0] = - self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[2,2] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_HFE[2,3] = - 0.08 *  self.c__q_RH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_RH_KFE[0,0] = - self.s__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[0,1] = - self.c__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[0,3] = (- 0.35 *  self.s__q_RH_HFE) -  0.3735;
        self.fr_trunk_Xh_fr_RH_KFE[1,0] =  self.s__q_RH_HAA *  self.c__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[1,1] = - self.s__q_RH_HAA *  self.s__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[1,2] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_KFE[1,3] = ( 0.35 *  self.s__q_RH_HAA *  self.c__q_RH_HFE) + ( 0.08 *  self.s__q_RH_HAA) -  0.207;
        self.fr_trunk_Xh_fr_RH_KFE[2,0] = - self.c__q_RH_HAA *  self.c__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[2,1] =  self.c__q_RH_HAA *  self.s__q_RH_HFE;
        self.fr_trunk_Xh_fr_RH_KFE[2,2] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_KFE[2,3] = (- 0.35 *  self.c__q_RH_HAA *  self.c__q_RH_HFE) - ( 0.08 *  self.c__q_RH_HAA);
        
        self.fr_LF_hipassembly_Xh_fr_trunk[0,1] = - self.s__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[0,3] =  0.207 *  self.s__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[1,1] = - self.c__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_LF_HAA;
        self.fr_LF_hipassembly_Xh_fr_trunk[1,3] =  0.207 *  self.c__q_LF_HAA;
        
        self.fr_trunk_Xh_fr_LF_hipassembly[1,0] = - self.s__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_hipassembly[1,1] = - self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_hipassembly[2,0] = - self.c__q_LF_HAA;
        self.fr_trunk_Xh_fr_LF_hipassembly[2,1] =  self.s__q_LF_HAA;
        
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[0,0] =  self.c__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[0,2] =  self.s__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[0,3] = - 0.08 *  self.c__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[1,0] = - self.s__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[1,2] =  self.c__q_LF_HFE;
        self.fr_LF_upperleg_Xh_fr_LF_hipassembly[1,3] =  0.08 *  self.s__q_LF_HFE;
        
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[0,0] =  self.c__q_LF_HFE;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[0,1] = - self.s__q_LF_HFE;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[2,0] =  self.s__q_LF_HFE;
        self.fr_LF_hipassembly_Xh_fr_LF_upperleg[2,1] =  self.c__q_LF_HFE;
        
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[0,0] =  self.c__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[0,1] =  self.s__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[0,3] = - 0.35 *  self.c__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[1,0] = - self.s__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[1,1] =  self.c__q_LF_KFE;
        self.fr_LF_lowerleg_Xh_fr_LF_upperleg[1,3] =  0.35 *  self.s__q_LF_KFE;
        
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[0,0] =  self.c__q_LF_KFE;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[0,1] = - self.s__q_LF_KFE;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[1,0] =  self.s__q_LF_KFE;
        self.fr_LF_upperleg_Xh_fr_LF_lowerleg[1,1] =  self.c__q_LF_KFE;
        
        self.fr_RF_hipassembly_Xh_fr_trunk[0,1] =  self.s__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[0,3] =  0.207 *  self.s__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[1,1] =  self.c__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_RF_HAA;
        self.fr_RF_hipassembly_Xh_fr_trunk[1,3] =  0.207 *  self.c__q_RF_HAA;
        
        self.fr_trunk_Xh_fr_RF_hipassembly[1,0] =  self.s__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_hipassembly[1,1] =  self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_hipassembly[2,0] = - self.c__q_RF_HAA;
        self.fr_trunk_Xh_fr_RF_hipassembly[2,1] =  self.s__q_RF_HAA;
        
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[0,0] =  self.c__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[0,2] = - self.s__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[0,3] = - 0.08 *  self.c__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[1,0] = - self.s__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[1,2] = - self.c__q_RF_HFE;
        self.fr_RF_upperleg_Xh_fr_RF_hipassembly[1,3] =  0.08 *  self.s__q_RF_HFE;
        
        
        
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[0,0] =  self.c__q_RF_HFE;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[0,1] = - self.s__q_RF_HFE;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[2,0] = - self.s__q_RF_HFE;
        self.fr_RF_hipassembly_Xh_fr_RF_upperleg[2,1] = - self.c__q_RF_HFE;
        
        
        
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[0,0] =  self.c__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[0,1] =  self.s__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[0,3] = - 0.35 *  self.c__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[1,0] = - self.s__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[1,1] =  self.c__q_RF_KFE;
        self.fr_RF_lowerleg_Xh_fr_RF_upperleg[1,3] =  0.35 *  self.s__q_RF_KFE;
        
        
        
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[0,0] =  self.c__q_RF_KFE;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[0,1] = - self.s__q_RF_KFE;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[1,0] =  self.s__q_RF_KFE;
        self.fr_RF_upperleg_Xh_fr_RF_lowerleg[1,1] =  self.c__q_RF_KFE;
        
        
        
        self.fr_LH_hipassembly_Xh_fr_trunk[0,1] = - self.s__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[0,3] =  0.207 *  self.s__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[1,1] = - self.c__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_LH_HAA;
        self.fr_LH_hipassembly_Xh_fr_trunk[1,3] =  0.207 *  self.c__q_LH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_LH_hipassembly[1,0] = - self.s__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_hipassembly[1,1] = - self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_hipassembly[2,0] = - self.c__q_LH_HAA;
        self.fr_trunk_Xh_fr_LH_hipassembly[2,1] =  self.s__q_LH_HAA;
        
        
        
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[0,0] =  self.c__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[0,2] =  self.s__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[0,3] = - 0.08 *  self.c__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[1,0] = - self.s__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[1,2] =  self.c__q_LH_HFE;
        self.fr_LH_upperleg_Xh_fr_LH_hipassembly[1,3] =  0.08 *  self.s__q_LH_HFE;
        
        
        
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[0,0] =  self.c__q_LH_HFE;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[0,1] = - self.s__q_LH_HFE;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[2,0] =  self.s__q_LH_HFE;
        self.fr_LH_hipassembly_Xh_fr_LH_upperleg[2,1] =  self.c__q_LH_HFE;
        
        
        
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[0,0] =  self.c__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[0,1] =  self.s__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[0,3] = - 0.35 *  self.c__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[1,0] = - self.s__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[1,1] =  self.c__q_LH_KFE;
        self.fr_LH_lowerleg_Xh_fr_LH_upperleg[1,3] =  0.35 *  self.s__q_LH_KFE;
        
        
        
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[0,0] =  self.c__q_LH_KFE;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[0,1] = - self.s__q_LH_KFE;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[1,0] =  self.s__q_LH_KFE;
        self.fr_LH_upperleg_Xh_fr_LH_lowerleg[1,1] =  self.c__q_LH_KFE;
        
        
        
        self.fr_RH_hipassembly_Xh_fr_trunk[0,1] =  self.s__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[0,2] = - self.c__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[0,3] =  0.207 *  self.s__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[1,1] =  self.c__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[1,2] =  self.s__q_RH_HAA;
        self.fr_RH_hipassembly_Xh_fr_trunk[1,3] =  0.207 *  self.c__q_RH_HAA;
        
        
        
        self.fr_trunk_Xh_fr_RH_hipassembly[1,0] =  self.s__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_hipassembly[1,1] =  self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_hipassembly[2,0] = - self.c__q_RH_HAA;
        self.fr_trunk_Xh_fr_RH_hipassembly[2,1] =  self.s__q_RH_HAA;
        
        
        
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[0,0] =  self.c__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[0,2] = - self.s__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[0,3] = - 0.08 *  self.c__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[1,0] = - self.s__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[1,2] = - self.c__q_RH_HFE;
        self.fr_RH_upperleg_Xh_fr_RH_hipassembly[1,3] =  0.08 *  self.s__q_RH_HFE;
        
        
        
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[0,0] =  self.c__q_RH_HFE;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[0,1] = - self.s__q_RH_HFE;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[2,0] = - self.s__q_RH_HFE;
        self.fr_RH_hipassembly_Xh_fr_RH_upperleg[2,1] = - self.c__q_RH_HFE;
        
        
        
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[0,0] =  self.c__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[0,1] =  self.s__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[0,3] = - 0.35 *  self.c__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[1,0] = - self.s__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[1,1] =  self.c__q_RH_KFE;
        self.fr_RH_lowerleg_Xh_fr_RH_upperleg[1,3] =  0.35 *  self.s__q_RH_KFE;
        
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
        self.fr_trunk_J_LF_foot[4-1,2-1] = ( 0.341 *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( 0.341 *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( 0.35 *  self.c__q_LF_HFE);
        self.fr_trunk_J_LF_foot[4-1,3-1] = ( 0.341 *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( 0.341 *  self.c__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_J_LF_foot[5-1,1-1] = ( 0.341 *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) - ( 0.341 *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) - ( 0.35 *  self.c__q_LF_HAA *  self.c__q_LF_HFE) - ( 0.08 *  self.c__q_LF_HAA);
        self.fr_trunk_J_LF_foot[5-1,2-1] = ( 0.341 *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( 0.341 *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE) + ( 0.35 *  self.s__q_LF_HAA *  self.s__q_LF_HFE);
        self.fr_trunk_J_LF_foot[5-1,3-1] = ( 0.341 *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( 0.341 *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);
        self.fr_trunk_J_LF_foot[6-1,1-1] = (- 0.341 *  self.s__q_LF_HAA *  self.s__q_LF_HFE *  self.s__q_LF_KFE) + ( 0.341 *  self.s__q_LF_HAA *  self.c__q_LF_HFE *  self.c__q_LF_KFE) + ( 0.35 *  self.s__q_LF_HAA *  self.c__q_LF_HFE) + ( 0.08 *  self.s__q_LF_HAA);
        self.fr_trunk_J_LF_foot[6-1,2-1] = ( 0.341 *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( 0.341 *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE) + ( 0.35 *  self.c__q_LF_HAA *  self.s__q_LF_HFE);
        self.fr_trunk_J_LF_foot[6-1,3-1] = ( 0.341 *  self.c__q_LF_HAA *  self.c__q_LF_HFE *  self.s__q_LF_KFE) + ( 0.341 *  self.c__q_LF_HAA *  self.s__q_LF_HFE *  self.c__q_LF_KFE);

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
        self.fr_trunk_J_RF_foot[4-1,2-1] = ( 0.341 *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( 0.341 *  self.c__q_RF_HFE *  self.c__q_RF_KFE) - ( 0.35 *  self.c__q_RF_HFE);
        self.fr_trunk_J_RF_foot[4-1,3-1] = ( 0.341 *  self.s__q_RF_HFE *  self.s__q_RF_KFE) - ( 0.341 *  self.c__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_J_RF_foot[5-1,1-1] = (- 0.341 *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) + ( 0.341 *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) + ( 0.35 *  self.c__q_RF_HAA *  self.c__q_RF_HFE) + ( 0.08 *  self.c__q_RF_HAA);
        self.fr_trunk_J_RF_foot[5-1,2-1] = (- 0.341 *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( 0.341 *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE) - ( 0.35 *  self.s__q_RF_HAA *  self.s__q_RF_HFE);
        self.fr_trunk_J_RF_foot[5-1,3-1] = (- 0.341 *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) - ( 0.341 *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);
        self.fr_trunk_J_RF_foot[6-1,1-1] = (- 0.341 *  self.s__q_RF_HAA *  self.s__q_RF_HFE *  self.s__q_RF_KFE) + ( 0.341 *  self.s__q_RF_HAA *  self.c__q_RF_HFE *  self.c__q_RF_KFE) + ( 0.35 *  self.s__q_RF_HAA *  self.c__q_RF_HFE) + ( 0.08 *  self.s__q_RF_HAA);
        self.fr_trunk_J_RF_foot[6-1,2-1] = ( 0.341 *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( 0.341 *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE) + ( 0.35 *  self.c__q_RF_HAA *  self.s__q_RF_HFE);
        self.fr_trunk_J_RF_foot[6-1,3-1] = ( 0.341 *  self.c__q_RF_HAA *  self.c__q_RF_HFE *  self.s__q_RF_KFE) + ( 0.341 *  self.c__q_RF_HAA *  self.s__q_RF_HFE *  self.c__q_RF_KFE);

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
        self.fr_trunk_J_LH_foot[4-1,2-1] = ( 0.341 *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( 0.341 *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( 0.35 *  self.c__q_LH_HFE);
        self.fr_trunk_J_LH_foot[4-1,3-1] = ( 0.341 *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( 0.341 *  self.c__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_J_LH_foot[5-1,1-1] = ( 0.341 *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) - ( 0.341 *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) - ( 0.35 *  self.c__q_LH_HAA *  self.c__q_LH_HFE) - ( 0.08 *  self.c__q_LH_HAA);
        self.fr_trunk_J_LH_foot[5-1,2-1] = ( 0.341 *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( 0.341 *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE) + ( 0.35 *  self.s__q_LH_HAA *  self.s__q_LH_HFE);
        self.fr_trunk_J_LH_foot[5-1,3-1] = ( 0.341 *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( 0.341 *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);
        self.fr_trunk_J_LH_foot[6-1,1-1] = (- 0.341 *  self.s__q_LH_HAA *  self.s__q_LH_HFE *  self.s__q_LH_KFE) + ( 0.341 *  self.s__q_LH_HAA *  self.c__q_LH_HFE *  self.c__q_LH_KFE) + ( 0.35 *  self.s__q_LH_HAA *  self.c__q_LH_HFE) + ( 0.08 *  self.s__q_LH_HAA);
        self.fr_trunk_J_LH_foot[6-1,2-1] = ( 0.341 *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( 0.341 *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE) + ( 0.35 *  self.c__q_LH_HAA *  self.s__q_LH_HFE);
        self.fr_trunk_J_LH_foot[6-1,3-1] = ( 0.341 *  self.c__q_LH_HAA *  self.c__q_LH_HFE *  self.s__q_LH_KFE) + ( 0.341 *  self.c__q_LH_HAA *  self.s__q_LH_HFE *  self.c__q_LH_KFE);

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
        self.fr_trunk_J_RH_foot[4-1,2-1] = ( 0.341 *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( 0.341 *  self.c__q_RH_HFE *  self.c__q_RH_KFE) - ( 0.35 *  self.c__q_RH_HFE);
        self.fr_trunk_J_RH_foot[4-1,3-1] = ( 0.341 *  self.s__q_RH_HFE *  self.s__q_RH_KFE) - ( 0.341 *  self.c__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_J_RH_foot[5-1,1-1] = (- 0.341 *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) + ( 0.341 *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) + ( 0.35 *  self.c__q_RH_HAA *  self.c__q_RH_HFE) + ( 0.08 *  self.c__q_RH_HAA);
        self.fr_trunk_J_RH_foot[5-1,2-1] = (- 0.341 *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( 0.341 *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE) - ( 0.35 *  self.s__q_RH_HAA *  self.s__q_RH_HFE);
        self.fr_trunk_J_RH_foot[5-1,3-1] = (- 0.341 *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) - ( 0.341 *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);
        self.fr_trunk_J_RH_foot[6-1,1-1] = (- 0.341 *  self.s__q_RH_HAA *  self.s__q_RH_HFE *  self.s__q_RH_KFE) + ( 0.341 *  self.s__q_RH_HAA *  self.c__q_RH_HFE *  self.c__q_RH_KFE) + ( 0.35 *  self.s__q_RH_HAA *  self.c__q_RH_HFE) + ( 0.08 *  self.s__q_RH_HAA);
        self.fr_trunk_J_RH_foot[6-1,2-1] = ( 0.341 *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( 0.341 *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE) + ( 0.35 *  self.c__q_RH_HAA *  self.s__q_RH_HFE);
        self.fr_trunk_J_RH_foot[6-1,3-1] = ( 0.341 *  self.c__q_RH_HAA *  self.c__q_RH_HFE *  self.s__q_RH_KFE) + ( 0.341 *  self.c__q_RH_HAA *  self.s__q_RH_HFE *  self.c__q_RH_KFE);

    def forward_kin(self, q):
        LF_foot = self.fr_trunk_Xh_LF_foot[0:2,3]
        RF_foot = self.fr_trunk_Xh_RF_foot[0:2,3]
        LH_foot = self.fr_trunk_Xh_LH_foot[0:2,3]
        RH_foot = self.fr_trunk_Xh_RH_foot[0:2,3]
        




