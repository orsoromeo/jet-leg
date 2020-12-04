import matplotlib.pyplot as plt
import numpy as np

class LearnedMargin:

    def plot_learned_margin(self, filename):
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

        plt.subplot(431)
        plt.plot(ee_offset_x, mx, 'b-o', markersize=2, label='learned (forward pass)')
    
        plt.subplot(432)
        plt.plot(ee_offset_y, my, 'b-o', markersize=2, label='learned (forward pass)')
    
        plt.subplot(433)
        plt.plot(ee_offset_z, mz, 'b-o', markersize=2,label='learned (forward pass)')
    
        plt.subplot(434)
        plt.plot(ee_offset_x, ee_jac_xx, 'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(435)
        plt.plot(ee_offset_y, ee_jac_yx, 'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(436)
        plt.plot(ee_offset_z, ee_jac_zx,  'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(437)
        plt.plot(ee_offset_x, ee_jac_xy, 'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(438)
        plt.plot(ee_offset_y, ee_jac_yy,  'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(439)
        plt.plot(ee_offset_z, ee_jac_zy, 'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(4, 3, 10)
        plt.plot(ee_offset_x, ee_jac_xz,  'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(4, 3, 11)
        plt.plot(ee_offset_y, ee_jac_zz, 'b-o', markersize=2, label='learned (backprop)')
    
        plt.subplot(4, 3, 12)
        plt.plot(ee_offset_z, ee_jac_zz,  'b-o', markersize=2, label='learned (backprop)')

    def set_plot_properties(self):

        y_lim = 1.5
        lb_lim_m = -0.15
        ub_lim_m = 0.25

        plt.subplot(431)
        plt.grid()
        plt.ylim((lb_lim_m, ub_lim_m))
        plt.xlabel("$c_{x}$ [m]")
        plt.ylabel("margin $m$ [m]")
        plt.title("CoM, x")
        plt.legend()

        plt.subplot(432)
        plt.grid()
        plt.ylim((lb_lim_m, ub_lim_m))
        plt.xlabel("$c_{y}$ [m]")
        plt.ylabel("margin $m$ [m]")
        plt.title("CoM, y")
        plt.legend()

        plt.subplot(433)
        plt.grid()
        plt.ylim((lb_lim_m, ub_lim_m))
        plt.xlabel("$c_{z}$ [m]")
        plt.ylabel("margin $m$ [m]")
        plt.title("CoM, z")
        plt.legend()

        plt.subplot(434)
        plt.grid()
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{x}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{x}$")
        plt.legend()

        plt.subplot(435)
        plt.grid()
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{y}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{x}$")
        plt.legend()

        plt.subplot(436)
        plt.grid()
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{z}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{x}$")
        plt.legend()

        plt.subplot(437)
        plt.grid()
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{x}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{y}$")
        plt.legend()

        plt.subplot(438)
        plt.grid()
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{y}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{y}$")
        plt.legend()

        plt.subplot(439)
        plt.grid()
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{z}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{y}$")
        plt.legend()

        plt.subplot(4, 3, 10)
        plt.grid()
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{x}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{z}$")
        plt.legend()

        plt.subplot(4, 3, 11)
        plt.grid()
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{y}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{z}$")
        plt.legend()

        plt.subplot(4, 3, 12)
        plt.grid()
        plt.ylim((-y_lim, y_lim))
        plt.xlabel("$c_{z}$ [m]")
        plt.ylabel("$\delta m/  \delta c_{z}$")
        plt.legend()

learned_margin = LearnedMargin()
fig1 = plt.figure(1)
fig1.suptitle("Learned stability margin network")
learned_margin.plot_learned_margin('com_jacobian_anymal_c_1111stance.txt')
learned_margin.set_plot_properties()
plt.show()
