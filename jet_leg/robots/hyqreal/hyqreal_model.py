import numpy as np

class HyqrealModel:
    def __init__(self):

        self.trunkMass = 120

        ''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
        HAA = Hip Abduction Adduction
        HFE = Hip Flextion Extension
        KFE = Knee Flextion Extension
        '''
        LF_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        RF_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        LH_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        RH_tau_lim = [50.0, 100.0, 100.0]  # HAA, HFE, KFE
        self.torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])