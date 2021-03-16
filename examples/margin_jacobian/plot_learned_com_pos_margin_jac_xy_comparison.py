import matplotlib.pyplot as plt
import numpy as np

class LearnedMargin:

    def plot_learned_margin(self, filename, idx1, idx2):
        path = '/home/rorsolino/catkin_ws/build/jet_leg_learn_ported/anymal_c/com_position/'
        iter, ee_offset_x, ee_offset_y, ee_offset_z, \
        mx, my, mz, \
        ee_jac_xx, ee_jac_xx_fin_diff, ee_jac_xy, ee_jac_xy_fin_diff, ee_jac_xz, ee_jac_xz_fin_diff, \
        ee_jac_yx, ee_jac_yx_fin_diff, ee_jac_yy, ee_jac_yy_fin_diff, ee_jac_yz, ee_jac_yz_fin_diff, \
        ee_jac_zx, ee_jac_zx_fin_diff, ee_jac_zy, ee_jac_zy_fin_diff, ee_jac_zz, ee_jac_zz_fin_diff \
            = np.loadtxt(path + filename,
            delimiter=',', unpack=True)

        y_lim = 1.5
        lb_lim_m = -0.15
        ub_lim_m = 0.25

        plt.subplot(idx1)
        plt.plot(ee_offset_x, mx, 'b-o', markersize=2, label='learned')

        plt.subplot(idx2)
        plt.plot(ee_offset_y, my, 'b-o', markersize=2, label='learned')

    def set_plot_properties(self, lb_lim_m = -0.15, ub_lim_m = 0.25, x_lim = 0.3, y_lim = 1.5):

        plt.subplot(321)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((0.05, 0.25))
        plt.ylabel("$m$ [m] \n 4 stance feet")
        plt.title("CoM, x")
        plt.legend()

        plt.subplot(322)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((0.05, 0.25))
        plt.title("CoM, y")

        plt.subplot(323)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((lb_lim_m, ub_lim_m))
        plt.ylabel("$m$ [m] \n LH in swing")

        plt.subplot(324)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((lb_lim_m, ub_lim_m))

        plt.subplot(325)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((lb_lim_m, ub_lim_m))
        plt.xlabel("$c_{x}$ [m]")
        plt.ylabel("$m$ [m] \n RF in swing")

        plt.subplot(326)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((lb_lim_m, ub_lim_m))
        plt.xlabel("$c_{y}$ [m]")

#learned_margin = LearnedMargin()
#fig1 = plt.figure(1)
#fig1.suptitle("Learned stability margin network")
#learned_margin.plot_learned_margin('com_jacobian_anymal_c_1111stance.txt')
#learned_margin.set_plot_properties()
#plt.show()
