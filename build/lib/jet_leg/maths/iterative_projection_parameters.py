# -*- coding: utf-8 -*-
"""
Created on Wed Nov 14 15:07:45 2018

@author: Romeo Orsolino
"""
import numpy as np
from jet_leg.maths.math_tools import Math

class IterativeProjectionParameters:
    def __init__(self):
        self.math = Math()
        self.q = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.] 
        self.comPositionBF = [0., 0., 0.] #var used only for IK inside constraints.py
        self.comPositionWF = [0., 0., 0.]
        self.footPosWLF = [0.3, 0.2, -.0]
        self.footPosWRF = [0.3, -0.2, -.0]
        self.footPosWLH = [-0.3, 0.2, -.0]
        self.footPosWRH = [-0.3, -0.2, -.0]
        self.externalForceWF = [0., 0., 0.]

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
    
        self.LF_tau_lim = [50.0, 50.0, 50.0]
        self.RF_tau_lim = [50.0, 50.0, 50.0]
        self.LH_tau_lim = [50.0, 50.0, 50.0]
        self.RH_tau_lim = [50.0, 50.0, 50.0]
        self.torque_limits = np.array([self.LF_tau_lim, self.RF_tau_lim, self.LH_tau_lim, self.RH_tau_lim])
        
        self.state_machineLF = True
        self.state_machineRF = True
        self.state_machineLH = True
        self.state_machineRH = True
        self.stanceFeet = [0, 0, 0, 0]
        self.numberOfContacts = 0
#        self.contactsHF = np.zeros((4,3))
        self.contactsBF = np.zeros((4,3))
        self.contactsWF = np.zeros((4,3))

        axisZ= np.array([[0.0], [0.0], [1.0]])
        n1 = np.transpose(np.transpose(self.math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n2 = np.transpose(np.transpose(self.math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n3 = np.transpose(np.transpose(self.math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        n4 = np.transpose(np.transpose(self.math.rpyToRot(0.0,0.0,0.0)).dot(axisZ))
        # %% Cell 2
        self.normals = np.vstack([n1, n2, n3, n4])
        self.constraintMode = ['FRICTION_AND_ACTUATION',
                               'FRICTION_AND_ACTUATION',
                               'FRICTION_AND_ACTUATION',
                               'FRICTION_AND_ACTUATION']
                               
        self.friction = 0.8
        self.robotMass = 85 #Kg
        self.numberOfGenerators = 4
                
        self.actual_swing = 0


    def setContactsPosBF(self, contactsBF):
        self.contactsBF = contactsBF

    def setContactsPosWF(self, contactsWF):
        self.contactsWF = contactsWF
        
    def setCoMPosWF(self, comWF):
        self.comPositionWF = comWF
    
    def setTorqueLims(self, torqueLims):
        self.torque_limits = torqueLims
        
    def setActiveContacts(self, activeContacts):
        self.stanceFeet = activeContacts
        
    def setContactNormals(self, normals):
        self.normals = normals
        
    def setConstraintModes(self, constraintMode):
        self.constraintMode = constraintMode
    
    def setFrictionCoefficient(self, mu):
        self.friction = mu
        
    def setNumberOfFrictionConesEdges(self, ng):
        self.numberOfGenerators = ng
        
    def setTotalMass(self, mass):
        self.robotMass = mass

    def getContactsPosWF(self):
        return self.contactsWF     
        
    def getContactsPosBF(self):# used only for IK inside constraints.py
        return self.contactsBF
        
    def getCoMPosWF(self):
        return self.comPositionWF
        
    def getCoMPosBF(self):
        return self.comPositionBF        
        
    def getTorqueLims(self):
        return self.torque_limits
        
    def getStanceFeet(self):
        return self.stanceFeet
        
    def getNormals(self):
        return self.normals

    def getOrientation(self):
        return self.roll, self.pitch, self.yaw
        
    def getConstraintModes(self):
        return self.constraintMode
        
    def getFrictionCoefficient(self):
        return self.friction
        
    def getNumberOfFrictionConesEdges(self):
        return self.numberOfGenerators
        
    def getTotalMass(self):
        return self.robotMass

    def getStanceIndex(self, stanceLegs):
        stanceIdx = []
        for iter in range(0, 4):
            if stanceLegs[iter] == 1:
                stanceIdx = np.hstack([stanceIdx, iter])
        return stanceIdx

    def getCurrentFeetPos(self, received_data):
        num_of_elements = np.size(received_data.data)
        for j in range(0,num_of_elements):
            if str(received_data.name[j]) == str("footPosDesLFx"):
                self.footPosWLF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesLFy"):
                self.footPosWLF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesLFz"):
                self.footPosWLF[2] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRFx"):
                self.footPosWRF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRFy"):
                self.footPosWRF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRFz"):
                self.footPosWRF[2] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesLHx"):
                self.footPosWLH[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesLHy"):
                self.footPosWLH[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesLHz"):
                self.footPosWLH[2] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRHx"):
                self.footPosWRH[0] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRHy"):
                self.footPosWRH[1] = received_data.data[j]
            if str(received_data.name[j]) == str("footPosDesRHz"):
                self.footPosWRH[2] = received_data.data[j]

        self.contactsWF = np.array([self.footPosWLF, self.footPosWRF, self.footPosWLH, self.footPosWRH])

    def getParamsFromRosDebugTopic(self, received_data):
        
        num_of_elements = np.size(received_data.data)
        for j in range(0,num_of_elements):
            if str(received_data.name[j]) == str("LF_HAAmaxVar"):
                self.LF_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("LF_HFEmaxVar"):
                self.LF_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("LF_KFEmaxVar"):
                self.LF_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_HAAmaxVar"):
                self.RF_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("RF_HFEmaxVar"):
                self.RF_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("RF_KFEmaxVar"):
                self.RF_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_HAAmaxVar"):
                self.LH_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("LH_HFEmaxVar"):
                self.LH_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("LH_KFEmaxVar"):
                self.LH_tau_lim[2] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_HAAmaxVar"):
                self.RH_tau_lim[0] = received_data.data[j]           
            if str(received_data.name[j]) == str("RH_HFEmaxVar"):
                self.RH_tau_lim[1] = received_data.data[j]
            if str(received_data.name[j]) == str("RH_KFEmaxVar"):
                self.RH_tau_lim[2] = received_data.data[j]
                
            self.torque_limits = np.array([self.LF_tau_lim, 
                                           self.RF_tau_lim, 
                                           self.LH_tau_lim, 
                                           self.RH_tau_lim, ])
 
# the inputs are all in the WF this way we can compute generic regions for generic contact sets and generic com position 
            
            if str(received_data.name[j]) == str("contact_setWLFx"):
                self.footPosWLF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWLFy"):
                self.footPosWLF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWLFz"):
                self.footPosWLF[2] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRFx"):
                self.footPosWRF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRFy"):
                self.footPosWRF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRFz"):
                self.footPosWRF[2] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWLHx"):
                self.footPosWLH[0] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWLHy"):
                self.footPosWLH[1] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWLHz"):
                self.footPosWLH[2] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRHx"):
                self.footPosWRH[0] =received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRHy"):
                self.footPosWRH[1] = received_data.data[j]
            if str(received_data.name[j]) == str("contact_setWRHz"):
                self.footPosWRH[2] = received_data.data[j]

            self.contactsWF = np.array([self.footPosWLF, self.footPosWRF, self.footPosWLH, self.footPosWRH])

            if str(received_data.name[j]) == str("actual_CoMX"):
                self.comPositionWF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("actual_CoMY"):
                self.comPositionWF[1] = received_data.data[j]
            if str(received_data.name[j]) == str("actual_CoMZ"):
                self.comPositionWF[2] = received_data.data[j]
                
            if str(received_data.name[j]) == str("offCoMX"):
                self.comPositionBF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("offCoMY"):
                self.comPositionBF[1] = received_data.data[j]                       
            if str(received_data.name[j]) == str("offCoMZ"):
                self.comPositionBF[2] = received_data.data[j]       
                
            # external wrench
            if str(received_data.name[j]) == str("extPerturbForceX"):
                self.externalForceWF[0] = received_data.data[j]
            if str(received_data.name[j]) == str("extPerturbForceY"):
                self.externalForceWF[1] = received_data.data[j]                       
            if str(received_data.name[j]) == str("extPerturbForceZ"):
                self.externalForceWF[2] = received_data.data[j]                
            if str(received_data.name[j]) == str("LF_HAA_th"):
                self.q[0] = received_data.data[j]  
            if str(received_data.name[j]) == str("LF_HFE_th"):
                self.q[1] = received_data.data[j] 
            if str(received_data.name[j]) == str("LF_KFE_th"):
                self.q[2] = received_data.data[j]  
            if str(received_data.name[j]) == str("RF_HAA_th"):
                self.q[3] = received_data.data[j]  
            if str(received_data.name[j]) == str("RF_HFE_th"):
                self.q[4] = received_data.data[j] 
            if str(received_data.name[j]) == str("RF_KFE_th"):
                self.q[5] = received_data.data[j]  
            if str(received_data.name[j]) == str("LH_HAA_th"):
                self.q[6] = received_data.data[j]  
            if str(received_data.name[j]) == str("LH_HFE_th"):
                self.q[7] = received_data.data[j] 
            if str(received_data.name[j]) == str("LH_KFE_th"):
                self.q[8] = received_data.data[j]  
            if str(received_data.name[j]) == str("RH_HAA_th"):
                self.q[9] = received_data.data[j]  
            if str(received_data.name[j]) == str("RH_HFE_th"):
                self.q[10] = received_data.data[j] 
            if str(received_data.name[j]) == str("RH_KFE_th"):
                self.q[11] = received_data.data[j]  
                
            #they are in WF
            if str(received_data.name[j]) == str("normalLFx"):
                self.normals[0,0] = received_data.data[j]  
            if str(received_data.name[j]) == str("normalLFy"):
                self.normals[0,1] = received_data.data[j] 
            if str(received_data.name[j]) == str("normalLFz"):
                self.normals[0,2] = received_data.data[j]                  
                                                 
            if str(received_data.name[j]) == str("normalRFx"):
                self.normals[1,0] = received_data.data[j]
            if str(received_data.name[j]) == str("normalRFy"):
                self.normals[1,1] = received_data.data[j] 
            if str(received_data.name[j]) == str("normalRFz"):
                self.normals[1,2] = received_data.data[j]                 
                                                 
            if str(received_data.name[j]) == str("normalLHx"):
                self.normals[2,0] = received_data.data[j] 
            if str(received_data.name[j]) == str("normalLHy"):
                self.normals[2,1] = received_data.data[j] 
            if str(received_data.name[j]) == str("normalLHz"):
                self.normals[2,2] = received_data.data[j]                
                                                 
            if str(received_data.name[j]) == str("normalRHx"):
                self.normals[3,0] = received_data.data[j]  
            if str(received_data.name[j]) == str("normalRHy"):
                self.normals[3,1] = received_data.data[j]  
            if str(received_data.name[j]) == str("normalRHz"):
                self.normals[3,2] = received_data.data[j]                      
            if str(received_data.name[j]) == str("robotMass"):
                self.robotMass = received_data.data[j] 
           
            if str(received_data.name[j]) == str("muEstimate"):
                self.friction = received_data.data[j]

            if str(received_data.name[j]) == str("roll"):
                self.roll = received_data.data[j]             
            if str(received_data.name[j]) == str("pitch"):
                self.pitch = received_data.data[j]    
            if str(received_data.name[j]) == str("yaw"):
                self.yaw = received_data.data[j]   
                
            if str(received_data.name[j]) == str("actual_swing"):  
                self.actual_swing = int(received_data.data[j])        
            
                
        self.robotMass -= self.externalForceWF[2]/9.81
                                   
          
    def getFutureStanceFeetFlags(self, received_data):

        num_of_elements = np.size(received_data.data)         
        for j in range(0,num_of_elements):
            if str(received_data.name[j]) == str("future_stance_LF"):
                self.stanceFeet[0] = int(received_data.data[j])
            if str(received_data.name[j]) == str("future_stance_RF"):
                self.stanceFeet[1] = int(received_data.data[j])
            if str(received_data.name[j]) == str("future_stance_LH"):
                self.stanceFeet[2] = int(received_data.data[j])
            if str(received_data.name[j]) == str("future_stance_RH"):
                self.stanceFeet[3] = int(received_data.data[j])  
        print('stance feet ', self.stanceFeet)
        self.numberOfContacts = np.sum(self.stanceFeet)
        
    def getCurrentStanceFeetFlags(self, received_data):
        
        num_of_elements = np.size(received_data.data)
        for j in range(0,num_of_elements):
            if str(received_data.name[j]) == str("state_machineLF"):
                self.state_machineLF = int(received_data.data[j])
            if str(received_data.name[j]) == str("state_machineRF"):
                self.state_machineRF = int(received_data.data[j])
            if str(received_data.name[j]) == str("state_machineLH"):
                self.state_machineLH = int(received_data.data[j])
            if str(received_data.name[j]) == str("state_machineRH"):
                self.state_machineRH = int(received_data.data[j]) 
                
            if self.state_machineLF < 4.0:
                self.stanceFeet[0] = 1
            else:
                self.stanceFeet[0] = 0
                
            if self.state_machineRF < 4.0:
                self.stanceFeet[1] = 1
            else:
                self.stanceFeet[1] = 0
                
            if self.state_machineLH < 4.0:
                self.stanceFeet[2] = 1
            else:
                self.stanceFeet[2] = 0
                
            if self.state_machineRH < 4.0:
                self.stanceFeet[3] = 1                
            else:
                self.stanceFeet[3] = 0
            
        self.numberOfContacts = np.sum(self.stanceFeet)

