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
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import random as rnd
from legsthrust.bilinear_constraints import BilinearConstraints


'''MAIN '''
bilinearConstraint = BilinearConstraints()
x_val = 1.5#rnd.random()
bilinearConstraint.make_convex(x_val)


''' Plot the results '''

plt.close("all")
fig = plt.figure(1)
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
# Add a color bar which maps values to colors.
#fig.colorbar(surf, shrink=0.5, aspect=5)

#plt.show()

plt.figure(2)
plt.plot(x,np.diagonal(p_hat))
plt.plot(x,np.diagonal(p_hat_relaxation),'bo',x,np.diagonal(p_hat_relaxation),'k')
plt.plot(x_val,0.0,'ro')
plt.xlim(-20, 20)
plt.ylim(-5, 35)
plt.grid()
#plt.show()