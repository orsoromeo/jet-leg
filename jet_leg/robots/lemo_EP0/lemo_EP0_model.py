import numpy as np

class LemoEP0Model:
    def __init__(self):

        self.trunkMass = 30
        self.trunkInertia = np.array([[2.13993, 0.00902021, 0.0958655],
                                      [0.00902021, 6.19139, 0.000206023],
                                      [0.0958655, 0.000206023, 6.26289]])

        ''' torque limits for each leg (this code assumes a hyq-like design, i.e. three joints per leg)
        HAA = Hip Abduction Adduction
        HFE = Hip Flextion Extension
        KFE = Knee Flextion Extension
        '''
        LF_tau_lim = [33.0, 33.0, 50.0]  # HAA, HFE, KFE
        RF_tau_lim = [33.0, 33.0, 50.0]  # HAA, HFE, KFE
        LH_tau_lim = [33.0, 33.0, 50.0]  # HAA, HFE, KFE
        RH_tau_lim = [33.0, 33.0, 50.0]  # HAA, HFE, KFE
        self.joint_torque_limits = np.array([LF_tau_lim, RF_tau_lim, LH_tau_lim, RH_tau_lim])

        ''' Add a fake small torque at the foot to enable the computation of the feasible region when only
        two point contacts with the environment '''
        self.contact_torque_limits = np.array([-1.5, 1.5])

        ''' Nominal foot position'''
        x_nominal_b = 0.3
        y_nominal_b = 0.2
        z_nominal_b = -0.5
        self.nominal_stance_LF = [x_nominal_b, y_nominal_b, z_nominal_b]
        self.nominal_stance_RF = [x_nominal_b, -y_nominal_b, z_nominal_b]
        self.nominal_stance_LH = [-x_nominal_b, y_nominal_b, z_nominal_b]
        self.nominal_stance_RH = [-x_nominal_b, -y_nominal_b, z_nominal_b]

        ''' Boundaries of the box constraint on the foot position relative to base frame'''
        self.max_dev_from_nominal = [0.1, 0.1, 0.1]