import matplotlib.pyplot as plt
import numpy as np

class LearnedMargin:

    def plot_learned_margin(self, filename):
        path = '/home/rorsolino/catkin_ws/build/jet_leg_learn_ported/anymal_c/base_orient/'
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

        plt.subplot(321)
        plt.plot(ee_offset_x, mx, 'b-o', markersize=2, label='learned')
    
        plt.subplot(322)
        plt.plot(ee_offset_y, my, 'b-o', markersize=2, label='learned')
    
        plt.subplot(323)
        plt.plot(ee_offset_x, ee_jac_xx, 'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(324)
        plt.plot(ee_offset_y, ee_jac_yx, 'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(325)
        plt.plot(ee_offset_x, ee_jac_xy, 'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(326)
        plt.plot(ee_offset_y, ee_jac_yy,  'b-o', markersize=2, label='learned (backprop)')

    def set_plot_properties(self, lb_lim_m = -0.15, ub_lim_m = 0.25, x_lim = 0.3, y_lim = 1.5):

        plt.subplot(321)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((lb_lim_m, ub_lim_m))
        plt.xlabel("$c_{x}$ [m]")
        plt.ylabel("margin $m$ [m]")
        plt.title("CoM, x")
        plt.legend()

        plt.subplot(322)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((lb_lim_m, ub_lim_m))
        plt.xlabel("$c_{y}$ [m]")
        #plt.ylabel("margin $m$ [m]")
        plt.title("CoM, y")
        #plt.legend()

        plt.subplot(323)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{x}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{x}$")
        #plt.legend()

        plt.subplot(324)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{y}$ [m]")
        #plt.ylabel("$\delta m/  \delta c_{x}$")
        #plt.legend()

        plt.subplot(325)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{x}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{y}$")
        #plt.legend()

        plt.subplot(326)
        plt.grid()
        plt.xlim((-x_lim, x_lim))
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{y}$ [m]")
        #plt.ylabel("$\delta m/  \delta c_{y}$")
        #plt.legend()

#learned_margin = LearnedMargin()
#fig1 = plt.figure(1)
#fig1.suptitle("Learned stability margin network")
#learned_margin.plot_learned_margin('com_jacobian_anymal_c_1111stance.txt')
#learned_margin.set_plot_properties()
#plt.show()
