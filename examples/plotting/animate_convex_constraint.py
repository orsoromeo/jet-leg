# -*- coding: utf-8 -*-
"""
Created on Sun Aug  5 19:03:32 2018

@author: Romeo Orsolino

Matplotlib Animation Example

author: Jake Vanderplas
email: vanderplas@astro.washington.edu
website: http://jakevdp.github.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""
import context
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from jet_leg.bilinear_constraints import BilinearConstraints

# First set up the figure, the axis, and the plot element we want to animate
fig = plt.figure()
ax = fig.gca(projection='3d')


surf = ax.plot_surface([], [], [], color="b", alpha=0.3)

# Customize the z axis.
ax.set_zlim(-10.01, 10.01)
ax.zaxis.set_major_locator(LinearLocator(10))
ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

ax.set_xlim(-10.01, 10.01)
ax.set_ylim(-10.01, 10.01)

# initialization function: plot the background of each frame
def init():
    surf.set_data([], [], [])
    return surf,

# animation function.  This is called sequentially
def animate(i):
    #x = np.linspace(0, 2, 1000)
    #y = np.sin(2 * np.pi * (x - 0.01 * i))
    #line.set_data(x, y)
    x = np.sin(i) 
    x, f, relaxation = bilinearConstraint.make_convex(x)
    surf.set_data([x], [f], [relaxation])
    return surf,

# call the animator.  blit=True means only re-draw the parts that have changed.
bilinearConstraint = BilinearConstraints()
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=200, interval=20, blit=True)

# save the animation as an mp4.  This requires ffmpeg or mencoder to be
# installed.  The extra_args ensure that the x264 codec is used, so that
# the video can be embedded in html5.  You may need to adjust this for
# your system: for more information, see
# http://matplotlib.sourceforge.net/api/animation_api.html
anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

plt.show()