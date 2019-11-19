import numpy as np

class AnymalModel:
    def __init__(self):

        self.trunkMass = 35

        ''' torque limits for each leg (this code assumes three joints per leg)
        HAA = Hip Abduction Adduction
        HFE = Hip Flextion Extension
        KFE = Knee Flextion Extension
        '''
        LF_tau_lim = [40.0, 40.0, 40.0]  # HAA, HFE, KFE
        RF_tau_lim = [40.0, 40.0, 40.0]  # HAA, HFE, KFE
        LH_tau_lim = [40.0, 40.0, 40.0]  # HAA, HFE, KFE
        RH_tau_lim = [40.0, 40.0, 40.0]  # HAA, HFE, KFE
        self.torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])