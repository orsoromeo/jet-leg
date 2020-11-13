import numpy as np

class HyqModel:
    def __init__(self):

        self.trunkMass = 75
        self.trunkInertia = np.array([[0.946438, -0.000938112, 0.00595386],
                                              [-0.000938112, 1.94478, 0.00146328],
                                              [0.00595386, 0.00146328, 2.01835]])
        ''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
        HAA = Hip Abduction Adduction
        HFE = Hip Flextion Extension
        KFE = Knee Flextion Extension
        '''
        LF_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        RF_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        LH_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        RH_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        self.joint_torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])
        self.contact_torque_limits = np.array([-4, 4])