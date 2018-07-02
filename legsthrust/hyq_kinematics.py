# -*- coding: utf-8 -*-
"""
Created on Mon Jul  2 05:34:42 2018

@author: romeo orsolino
"""
import numpy as np

class HyQKinematics:
    def init_homogeneous(self):
        fr_LF_lowerleg_Xh_LF_foot = np.zeros((4,4));	
        fr_LF_lowerleg_Xh_LF_foot[0,2] = - 1.0;
        fr_LF_lowerleg_Xh_LF_foot[0,3] = 0.341;
        fr_LF_lowerleg_Xh_LF_foot[1,0] = - 1;
        fr_LF_lowerleg_Xh_LF_foot[2,1] = 1;
        fr_LF_lowerleg_Xh_LF_foot[3,3] = 1;	
        
        fr_RF_lowerleg_Xh_RF_foot = np.zeros((4,4));
        fr_RF_lowerleg_Xh_RF_foot[0,2] = - 1.0;
        fr_RF_lowerleg_Xh_RF_foot[0,3] = 0.341;
        fr_RF_lowerleg_Xh_RF_foot[1,0] = - 1;
        fr_RF_lowerleg_Xh_RF_foot[2,1] = 1;
        fr_RF_lowerleg_Xh_RF_foot[3,3] = 1;	
        
        fr_LH_lowerleg_Xh_LH_foot = np.zeros((4,4));
        fr_LH_lowerleg_Xh_LH_foot[0,2] = - 1.0;
        fr_LH_lowerleg_Xh_LH_foot[0,3] = 0.341;
        fr_LH_lowerleg_Xh_LH_foot[1,0] = - 1;
        fr_LH_lowerleg_Xh_LH_foot[2,1] = 1;
        fr_LH_lowerleg_Xh_LH_foot[3,3] = 1;	
        
        fr_RH_lowerleg_Xh_RH_foot = np.zeros((4,4));
        fr_RH_lowerleg_Xh_RH_foot[0,2] = - 1.0;
        fr_RH_lowerleg_Xh_RH_foot[0,3] = 0.341;
        fr_RH_lowerleg_Xh_RH_foot[1,0] = - 1;
        fr_RH_lowerleg_Xh_RH_foot[2,1] = 1;
        fr_RH_lowerleg_Xh_RH_foot[3,3] = 1;	
        
        LF_foot_Xh_fr_trunk = np.zeros((4,4));
        LF_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        RF_foot_Xh_fr_trunk = np.zeros((4,4));
        RF_foot_Xh_fr_trunk[3,3]= 1.0;	
        
        LH_foot_Xh_fr_trunk = np.zeros((4,4));
        LH_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        RH_foot_Xh_fr_trunk = np.zeros((4,4));
        RH_foot_Xh_fr_trunk[3,3] = 1.0;	
        
        fr_trunk_Xh_LF_foot = np.zeros((4,4));
        fr_trunk_Xh_LF_foot[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_LF_HAA = np.zeros((4,4));
        fr_trunk_Xh_fr_LF_HAA[0,2] = - 1.0;
        fr_trunk_Xh_fr_LF_HAA[0,3] = 0.3735;
        fr_trunk_Xh_fr_LF_HAA[1,1] = - 1.0;
        fr_trunk_Xh_fr_LF_HAA[1,3] = 0.207;
        fr_trunk_Xh_fr_LF_HAA[2,0] = - 1.0;
        fr_trunk_Xh_fr_LF_HAA[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_LF_HFE = np.zeros((4,4));
        fr_trunk_Xh_fr_LF_HFE[0,1] = - 1.0;
        fr_trunk_Xh_fr_LF_HFE[0,3] = 0.3735;
        fr_trunk_Xh_fr_LF_HFE[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_LF_KFE = np.zeros((4,4));
        fr_trunk_Xh_fr_LF_KFE[3,3] = 1.0;	  
        
        fr_trunk_Xh_RF_foot = np.zeros((4,4));
        fr_trunk_Xh_RF_foot[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_RF_HAA = np.zeros((4,4));
        fr_trunk_Xh_fr_RF_HAA[0,2] = 1.0;
        fr_trunk_Xh_fr_RF_HAA[0,3] = 0.3735;
        fr_trunk_Xh_fr_RF_HAA[1,1] = 1.0;
        fr_trunk_Xh_fr_RF_HAA[1,3] = - 0.207;
        fr_trunk_Xh_fr_RF_HAA[2,0] = - 1.0;
        fr_trunk_Xh_fr_RF_HAA[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_RF_HFE = np.zeros((4,4));
        fr_trunk_Xh_fr_RF_HFE[0,1] = - 1.0;
        fr_trunk_Xh_fr_RF_HFE[0,3] = 0.3735;
        fr_trunk_Xh_fr_RF_HFE[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_RF_KFE = np.zeros((4,4));
        fr_trunk_Xh_fr_RF_KFE[3,3] = 1.0;	
        
        fr_trunk_Xh_LH_foot = np.zeros((4,4));
        fr_trunk_Xh_LH_foot[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_LH_HAA = np.zeros((4,4));
        fr_trunk_Xh_fr_LH_HAA[0,2] = - 1.0;
        fr_trunk_Xh_fr_LH_HAA[0,3] = - 0.3735;
        fr_trunk_Xh_fr_LH_HAA[1,1] = - 1.0;
        fr_trunk_Xh_fr_LH_HAA[1,3] = 0.207;
        fr_trunk_Xh_fr_LH_HAA[2,0] = - 1.0;
        fr_trunk_Xh_fr_LH_HAA[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_LH_HFE = np.zeros((4,4));
        fr_trunk_Xh_fr_LH_HFE[0,0] = - 1.0;
        fr_trunk_Xh_fr_LH_HFE[0,3] = - 0.3735;
        fr_trunk_Xh_fr_LH_HFE[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_LH_KFE = np.zeros((4,4));
        fr_trunk_Xh_fr_LH_KFE[3,3] = 1.0;	
        
        fr_trunk_Xh_RH_foot =np.zeros((4,4));
        fr_trunk_Xh_RH_foot[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_RH_HAA =np.zeros((4,4));
        fr_trunk_Xh_fr_RH_HAA[0,2] = 1.0;
        fr_trunk_Xh_fr_RH_HAA[0,3] = - 0.3735;
        fr_trunk_Xh_fr_RH_HAA[1,1] = 1.0;
        fr_trunk_Xh_fr_RH_HAA[1,3] = - 0.207;
        fr_trunk_Xh_fr_RH_HAA[2,0] = - 1.0;
        fr_trunk_Xh_fr_RH_HAA[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_RH_HFE =np.zeros((4,4));
        fr_trunk_Xh_fr_RH_HFE[0,1] = - 1.0;
        fr_trunk_Xh_fr_RH_HFE[0,3] = - 0.3735;
        fr_trunk_Xh_fr_RH_HFE[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_RH_KFE =np.zeros((4,4));
        fr_trunk_Xh_fr_RH_KFE[3,3] = 1.0;	
        
        fr_LF_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        fr_LF_hipassembly_Xh_fr_trunk[2,0] = - 1.0;
        fr_LF_hipassembly_Xh_fr_trunk[2,3] = 0.3735;
        fr_LF_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_LF_hipassembly =np.zeros((4,4));
        fr_trunk_Xh_fr_LF_hipassembly[0,2] = - 1.0;
        fr_trunk_Xh_fr_LF_hipassembly[0,3] = 0.3735;
        fr_trunk_Xh_fr_LF_hipassembly[1,3] = 0.207;
        fr_trunk_Xh_fr_LF_hipassembly[3,3] = 1.0;	
            
        fr_LF_upperleg_Xh_fr_LF_hipassembly =np.zeros((4,4));
        fr_LF_upperleg_Xh_fr_LF_hipassembly[2,1] = - 1;
        fr_LF_upperleg_Xh_fr_LF_hipassembly[3,3] = 1.0;	
     
        fr_LF_hipassembly_Xh_fr_LF_upperleg =np.zeros((4,4));
        fr_LF_hipassembly_Xh_fr_LF_upperleg[0,3] = 0.08;
        fr_LF_hipassembly_Xh_fr_LF_upperleg[1,2] = - 1;
        fr_LF_hipassembly_Xh_fr_LF_upperleg[3,3] = 1;	
        
        fr_LF_lowerleg_Xh_fr_LF_upperleg =np.zeros((4,4));
        fr_LF_lowerleg_Xh_fr_LF_upperleg[2,2] = 1;
        fr_LF_lowerleg_Xh_fr_LF_upperleg[3,3] = 1.0;	
        
        fr_LF_upperleg_Xh_fr_LF_lowerleg =np.zeros((4,4));
        fr_LF_upperleg_Xh_fr_LF_lowerleg[0,3] = 0.35;
        fr_LF_upperleg_Xh_fr_LF_lowerleg[2,2] = 1;
        fr_LF_upperleg_Xh_fr_LF_lowerleg[3,3] = 1;	
        
        fr_RF_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        fr_RF_hipassembly_Xh_fr_trunk[2,0] = 1.0;
        fr_RF_hipassembly_Xh_fr_trunk[2,3] = - 0.3735;
        fr_RF_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_RF_hipassembly =np.zeros((4,4));
        fr_trunk_Xh_fr_RF_hipassembly[0,2] = 1.0;
        fr_trunk_Xh_fr_RF_hipassembly[0,3] = 0.3735;
        fr_trunk_Xh_fr_RF_hipassembly[1,3] = - 0.207;
        fr_trunk_Xh_fr_RF_hipassembly[3,3] = 1.0;	
        
        fr_RF_upperleg_Xh_fr_RF_hipassembly =np.zeros((4,4));
        fr_RF_upperleg_Xh_fr_RF_hipassembly[2,1] = 1;
        fr_RF_upperleg_Xh_fr_RF_hipassembly[3,3] = 1.0;	
        
        fr_RF_hipassembly_Xh_fr_RF_upperleg =np.zeros((4,4));
        fr_RF_hipassembly_Xh_fr_RF_upperleg[0,3] = 0.08;
        fr_RF_hipassembly_Xh_fr_RF_upperleg[1,2] = 1;
        fr_RF_hipassembly_Xh_fr_RF_upperleg[3,3] = 1;	
        
        fr_RF_lowerleg_Xh_fr_RF_upperleg =np.zeros((4,4));
        fr_RF_lowerleg_Xh_fr_RF_upperleg[2,2] = 1;
        fr_RF_lowerleg_Xh_fr_RF_upperleg[3,3] = 1.0;	
        
        fr_RF_upperleg_Xh_fr_RF_lowerleg =np.zeros((4,4));
        fr_RF_upperleg_Xh_fr_RF_lowerleg[0,3] = 0.35;
        fr_RF_upperleg_Xh_fr_RF_lowerleg[2,2] = 1;
        fr_RF_upperleg_Xh_fr_RF_lowerleg[3,3] = 1;	
        
        fr_LH_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        fr_LH_hipassembly_Xh_fr_trunk[2,0] = - 1.0;
        fr_LH_hipassembly_Xh_fr_trunk[2,3] = - 0.3735;
        fr_LH_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_LH_hipassembly =np.zeros((4,4));
        fr_trunk_Xh_fr_LH_hipassembly[0,2] = - 1.0;
        fr_trunk_Xh_fr_LH_hipassembly[0,3] = - 0.3735;
        fr_trunk_Xh_fr_LH_hipassembly[1,3] = 0.207;
        fr_trunk_Xh_fr_LH_hipassembly[3,3] = 1.0;	
        
        fr_LH_upperleg_Xh_fr_LH_hipassembly =np.zeros((4,4));
        fr_LH_upperleg_Xh_fr_LH_hipassembly[2,1] = - 1;
        fr_LH_upperleg_Xh_fr_LH_hipassembly[3,3] = 1.0;	
        
        fr_LH_hipassembly_Xh_fr_LH_upperleg =np.zeros((4,4));
        fr_LH_hipassembly_Xh_fr_LH_upperleg[0,3] = 0.08;
        fr_LH_hipassembly_Xh_fr_LH_upperleg[1,2] = - 1;
        fr_LH_hipassembly_Xh_fr_LH_upperleg[3,3] = 1;	
        
        fr_LH_lowerleg_Xh_fr_LH_upperleg =np.zeros((4,4));
        fr_LH_lowerleg_Xh_fr_LH_upperleg[2,2]= 1;
        fr_LH_lowerleg_Xh_fr_LH_upperleg[3,3]= 1.0;	
        
        fr_LH_upperleg_Xh_fr_LH_lowerleg =np.zeros((4,4));
        fr_LH_upperleg_Xh_fr_LH_lowerleg[0,3] = 0.35;
        fr_LH_upperleg_Xh_fr_LH_lowerleg[2,2] = 1;
        fr_LH_upperleg_Xh_fr_LH_lowerleg[3,3] = 1;	
        
        fr_RH_hipassembly_Xh_fr_trunk =np.zeros((4,4));
        fr_RH_hipassembly_Xh_fr_trunk[2,0] = 1.0;
        fr_RH_hipassembly_Xh_fr_trunk[2,3] = 0.3735;
        fr_RH_hipassembly_Xh_fr_trunk[3,3] = 1.0;	
        
        fr_trunk_Xh_fr_RH_hipassembly =np.zeros((4,4));
        fr_trunk_Xh_fr_RH_hipassembly[0,2]= 1.0;
        fr_trunk_Xh_fr_RH_hipassembly[0,3]= - 0.3735;
        fr_trunk_Xh_fr_RH_hipassembly[1,3]= - 0.207;
        fr_trunk_Xh_fr_RH_hipassembly[3,3]= 1.0;	
        
        fr_RH_upperleg_Xh_fr_RH_hipassembly =np.zeros((4,4));
        fr_RH_upperleg_Xh_fr_RH_hipassembly[2,1]= 1;
        fr_RH_upperleg_Xh_fr_RH_hipassembly[3,3]= 1.0;	
        
        fr_RH_hipassembly_Xh_fr_RH_upperleg =np.zeros((4,4));
        fr_RH_hipassembly_Xh_fr_RH_upperleg[0,3] = 0.08;
        fr_RH_hipassembly_Xh_fr_RH_upperleg[1,2] = 1;
        fr_RH_hipassembly_Xh_fr_RH_upperleg[3,3] = 1;	
        
        fr_RH_lowerleg_Xh_fr_RH_upperleg =np.zeros((4,4));
        fr_RH_lowerleg_Xh_fr_RH_upperleg[2,2] = 1;
        fr_RH_lowerleg_Xh_fr_RH_upperleg[3,3] = 1.0;
        
        fr_RH_upperleg_Xh_fr_RH_lowerleg =np.zeros((4,4));
        fr_RH_upperleg_Xh_fr_RH_lowerleg[0,3] = 0.35;
        fr_RH_upperleg_Xh_fr_RH_lowerleg[2,2] = 1;
        fr_RH_upperleg_Xh_fr_RH_lowerleg[3,3] = 1;
        
        return fr_LF_lowerleg_Xh_LF_foot, fr_RF_lowerleg_Xh_RF_foot 