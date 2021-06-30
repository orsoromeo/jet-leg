import numpy as np

class HyqrealModel:
    def __init__(self):

        self.trunkMass = 52
        self.trunkInertia = np.array([[2.13993, 0.00902021, 0.0958655],
                                      [0.00902021, 6.19139, 0.000206023],
                                      [0.0958655, 0.000206023, 6.26289]])

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
        self.contact_torque_limits = np.array([-1.5, 1.5])

        x_nominal_b = 0.36
        y_nominal_b = 0.2
        z_nominal_b = -0.4
        self.nominal_stance_LF = [x_nominal_b, y_nominal_b, z_nominal_b]
        self.nominal_stance_RF = [x_nominal_b, -y_nominal_b, z_nominal_b]
        self.nominal_stance_LH = [-x_nominal_b, y_nominal_b, z_nominal_b]
        self.nominal_stance_RH = [-x_nominal_b, -y_nominal_b, z_nominal_b]
        self.max_dev_from_nominal = [0.1, 0.1, 0.1]