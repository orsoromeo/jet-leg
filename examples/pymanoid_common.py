#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2018 Stephane Caron <stephane.caron@lirmm.fr>
#
# This file is part of jet-leg <https://github.com/orsoromeo/jet-leg>.
#
# jet-leg is free software: you can redistribute it and/or modify it under the
# terms of the GNU Lesser General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option) any
# later version.
#
# jet-leg is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
# details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with jet-leg. If not, see <http://www.gnu.org/licenses/>.

from numpy import arange, array, hstack, vstack, zeros
from scipy.linalg import block_diag

import pymanoid

from pymanoid.misc import norm
from pymanoid.gui import draw_polygon
from pymanoid.sim import gravity_const
from pypoman import project_polytope


class CoMPolygonDrawer(pymanoid.Process):

    """
    Draw the static-equilibrium polygon of a contact set.

    Parameters
    ----------
    stance : Stance
        Contacts and COM position of the robot.
    """

    def __init__(self, stance, height=1.5):
        super(CoMPolygonDrawer, self).__init__()
        self.contact_poses = {}
        self.handle = None
        self.height = height
        self.stance = stance
        #
        self.update_contact_poses()
        self.update_handle()

    def on_tick(self, sim):
        if self.handle is None:
            self.update_handle()
        for contact in self.stance.contacts:
            if norm(contact.pose - self.contact_poses[contact.name]) > 1e-10:
                self.update_contact_poses()
                self.update_handle()
                break

    def update_contact_poses(self):
        for contact in self.stance.contacts:
            self.contact_poses[contact.name] = contact.pose

    def update_handle(self):
        self.handle = None
        try:
            vertices = self.stance.compute_static_equilibrium_polygon(
                method="cdd")
            self.handle = draw_polygon(
                [(x[0], x[1], self.height) for x in vertices],
                normal=[0, 0, 1], color='g')
        except Exception as e:
            print("CoMPolygonDrawer: {}".format(e))


def compute_actuation_dependent_polygon(robot, contacts):
    """
    Compute constraint matrices of the problem:

        A * w_all  <=  b
        C * w_all  ==  d

    and output variables are given by:

        [x_com y_com]  =  E * w_all + f

    where w_all is the stacked vector of external contact wrenches.

    Returns
    -------
    vertices : list of arrays
        2D vertices of the static-equilibrium polygon.
    """
    g = robot.compute_static_gravity_torques()
    J_contact = robot.compute_contact_jacobian(contacts)

    # Friction limits:
    #
    #     A_fric * w_all <= b_fric
    #
    A_fric = block_diag(*[c.wrench_inequalities for c in contacts.contacts])
    b_fric = zeros((A_fric.shape[0],))

    # Torque limits:
    #
    #     |tau| <= robot.tau_max
    #
    # where tau == g - J_c^T w_all in static equilibrium.
    #
    A_act = vstack([-J_contact.T[:-6], +J_contact.T[:-6]])
    b_act = hstack([(robot.tau_max - g)[:-6], (robot.tau_max + g)[:-6]])

    # Net wrench constraint:
    #
    #     w_0.force = -mass * gravity
    #     w_0.moment.z = 0
    #
    # where w_0 = G_0 * w_all is the contact wrench at the origin of the
    # inertial frame, with G_0 the corresponding grasp matrix.
    #
    G_0 = contacts.compute_grasp_matrix([0., 0., 0.])
    C = G_0[(0, 1, 2, 5), :]
    d = array([0., 0., robot.mass * gravity_const, 0.])

    # Output matrix: CoM horizontal coordinates
    E = 1. / (robot.mass * 9.81) * vstack([-G_0[4, :], +G_0[3, :]])
    f = zeros(2)

    A = vstack([A_fric, A_act])
    b = hstack([b_fric, b_act])
    return project_polytope(
        ineq=(A, b), eq=(C, d), proj=(E, f), method="bretl", max_iter=100, init_angle=0)


def generate_point_grid(xlim, ylim, zlim, xres, yres):
    assert xres % 2 == 0 and yres % 2 == 0
    p = zeros(2)
    dx = (xlim[1] - xlim[0]) / xres
    dy = (ylim[1] - ylim[0]) / yres
    x_avg = (xlim[1] + xlim[0]) / 2.
    y_avg = (ylim[1] + ylim[0]) / 2.
    dx_sign = +1
    dy_sign = +1
    p[0] = x_avg - dx * dx_sign * (1 + xres / 2)
    output = []
    zrange = arange(zlim[0], zlim[1], 0.03)
    for height in zrange:
        points = []
        p[1] = y_avg - dy * dy_sign * (1 + yres / 2)
        for _ in xrange(xres + 1):
            p[0] += dx * dx_sign
            for _ in xrange(yres + 1):
                p[1] += dy * dy_sign
                points.append(array([p[0], p[1]]))
            p[1] += dy * dy_sign
            dy_sign *= -1.
        p[0] += dx * dx_sign
        dx_sign *= -1.
        output.append((height, points))
    return output
