'''
======================
3D surface (color map)
======================

Demonstrates plotting a 3D surface colored with the coolwarm color map.
The surface is made opaque by using antialiased=False.

Also demonstrates using the LinearLocator and custom formatting for the
z axis tick labels.
'''
import context
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib as mpl
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import random as rnd
from jet_leg.bilinear_constraints import BilinearConstraints


'''MAIN '''
makePlots = False
bilinearConstraint = BilinearConstraints()
x_val = 1.5#rnd.random()
X, F, p_hat, p_hat_relaxation, q_hat = bilinearConstraint.make_convex(x_val)



''' Plot the results '''
if makePlots:
    plt.close("all")
    fig = plt.figure(1)
    
    ax = fig.gca(projection='3d')
    Tau = bilinearConstraint.get_torque()
    surf = ax.plot_surface(X, F, Tau, color="r", alpha=0.4)
    #surf = ax.plot_surface(X, F, p_hat, color="r", alpha=0.4)
    #surf = ax.plot_surface(X, F, p_hat_relaxation, color="b", alpha=0.3)
    #surf = ax.plot_surface(X, Y, Conv_minus, color="y", alpha=0.3)
    #surf = ax.plot_surface(X, Y, Tau_approx, color="g", alpha=0.3)
    # Customize the z axis.
    ax.set_zlim(-10.01, 10.01)
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    
    ax.set_xlim(-10.01, 10.01)
    ax.set_ylim(-10.01, 10.01)
    fake2Dline = mpl.lines.Line2D([0],[0], linestyle="none", c='r', marker = 's', alpha=0.4)
    ax.legend([fake2Dline], ['bilinear constraint'], numpoints = 1, loc="upper left", markerscale=5.)
    ax.set_xlabel('position [m]', labelpad=20)
    ax.set_ylabel('force [N]', labelpad=20)
    ax.set_zlabel('torque [Nm]', labelpad=20)
    mpl.rcParams.update({'font.size': 22})
    #mpl.rcParams['xtick.labelsize'] =15
    #plt.tick_params(labelsize=1)
    #ax.set_xticklabels(ax1_x, fontsize=15)
    fig.savefig('../figs/bilinear_constraints/bilinear_constraint.pdf')
    fig.savefig('../figs/bilinear_constraints/bilinear_constraint.png')
    
    fig = plt.figure(2)
    ax = fig.gca(projection='3d')
    Tau = bilinearConstraint.get_torque()
    #surf = ax.plot_surface(X, F, Tau, color="b", alpha=0.7)
    surf = ax.plot_surface(X, F, 0.25*p_hat, color="r", alpha=0.6)
    surf = ax.plot_surface(X, F, -0.25*q_hat, color="r", alpha=0.6)
    #surf = ax.plot_surface(X, F, p_hat_relaxation, color="b", alpha=0.3)
    #surf = ax.plot_surface(X, Y, Conv_minus, color="y", alpha=0.3)
    #surf = ax.plot_surface(X, Y, Tau_approx, color="g", alpha=0.3)
    # Customize the z axis.
    ax.set_zlim(-10.01, 10.01)
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    
    ax.set_xlim(-10.01, 10.01)
    ax.set_ylim(-10.01, 10.01)
    fake2Dline = mpl.lines.Line2D([0],[0], linestyle="none", c='r', marker = 's', alpha=0.4)
    ax.legend([fake2Dline], ['quadratic terms'], numpoints = 1, loc="upper left", markerscale=5.)
    ax.set_xlabel('position [m]', labelpad=20)
    ax.set_ylabel('force [N]', labelpad=20)
    ax.set_zlabel('torque [Nm]', labelpad=20)
    mpl.rcParams.update({'font.size': 22})
    #mpl.rcParams['xtick.labelsize'] =15
    #plt.tick_params(labelsize=1)
    #ax.set_xticklabels(ax1_x, fontsize=15)
    fig.savefig('../figs/bilinear_constraints/quadratic_contraints.pdf')
    fig.savefig('../figs/bilinear_constraints/quadratic_contraints.png')
    
    fig = plt.figure(3)
    ax = fig.gca(projection='3d')
    #surf = ax.plot_surface(X, Y, Tau, color="r", alpha=0.4)
    surf = ax.plot_surface(X, F, p_hat, color="r", alpha=0.4)
    surf = ax.plot_surface(X, F, p_hat_relaxation, color="b", alpha=0.3)
    #surf = ax.plot_surface(X, Y, Conv_minus, color="y", alpha=0.3)
    #surf = ax.plot_surface(X, Y, Tau_approx, color="g", alpha=0.3)
    # Customize the z axis.
    ax.set_zlim(-10.01, 10.01)
    ax.zaxis.set_major_locator(LinearLocator(10))
    ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
    
    ax.set_xlim(-10.01, 10.01)
    ax.set_ylim(-10.01, 10.01)
    fake2Dline1 = mpl.lines.Line2D([0],[0], linestyle="none", c='r', marker = 's', alpha=0.4)
    fake2Dline2 = mpl.lines.Line2D([0],[0], linestyle="none", c='b', marker = 's', alpha=0.3)
    ax.legend([fake2Dline1, fake2Dline2], ['quadratic soft constraint', 'linear constraint'], numpoints = 1, loc="upper left", markerscale=5.)
    ax.set_xlabel('position [m]', labelpad=20)
    ax.set_ylabel('force [N]', labelpad=20)
    ax.set_zlabel('torque [Nm]', labelpad=20)
    mpl.rcParams.update({'font.size': 15})
    #mpl.rcParams['xtick.labelsize'] =15
    #plt.tick_params(labelsize=1)
    #ax.set_xticklabels(ax1_x, fontsize=15)
    fig.savefig('../figs/bilinear_constraints/linear_constraints.pdf')
    fig.savefig('../figs/bilinear_constraints/linear_constraints.png')
    
    
    fig = plt.figure(4)
    plt.plot(X[len(X)/2.0,:],np.diagonal(p_hat),'r',linewidth=3, label = 'quadratic soft constraint')
    plt.plot(X[len(X)/2.0,:],np.diagonal(p_hat_relaxation),'b',linewidth=3, label = 'linear approximation')
    plt.plot(x_val,0.0,'ro')
    plt.xlim(-20, 20)
    plt.ylim(-5, 35)
    plt.grid()
    plt.legend()
    plt.xlabel('position [m]')
    plt.ylabel('torque [Nm]')
    mpl.rcParams.update({'font.size': 15})
    fig.savefig('../figs/bilinear_constraints/linear_constraints_2D.pdf')
    fig.savefig('../figs/bilinear_constraints/linear_constraints_2D.png')