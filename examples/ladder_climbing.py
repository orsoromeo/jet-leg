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

import IPython
import numpy
# import pylab

from numpy import array, hstack, ones, vstack, zeros
from scipy.linalg import block_diag

import pymanoid

from pymanoid import Stance
from pymanoid.gui import StaticEquilibriumWrenchDrawer
from pymanoid.gui import draw_point, draw_polygon, draw_polytope
from pymanoid.misc import norm
from pymanoid.sim import gravity_const
from pypoman import project_polytope


class CoMPolygonDrawer(pymanoid.Process):

    """
    Draw the static-equilibrium polygon of a contact set.

    Parameters
    ----------
    stance : Stance
        Contacts and COM position of the robot.
    color : tuple or string, optional
        Area color.
    """

    def __init__(self, stance, height, color):
        if color is None:
            color = (0., 0.5, 0., 0.5)
        if type(color) is str:
            from pymanoid.misc import matplotlib_to_rgb
            color = matplotlib_to_rgb(color) + [0.5]
        super(CoMPolygonDrawer, self).__init__()
        self._method = "bretl"
        self.color = color
        self.contact_poses = {}
        self.handle = None
        self.height = height
        self.stance = stance
        #
        self.update_contact_poses()
        self.update_polygon()

    def on_tick(self, sim):
        if self.handle is None:
            self.update_polygon()
        for contact in self.stance.contacts:
            if norm(contact.pose - self.contact_poses[contact.name]) > 1e-10:
                self.update_contact_poses()
                self.update_polygon()
                break

    def update_contact_poses(self):
        for contact in self.stance.contacts:
            self.contact_poses[contact.name] = contact.pose

    def update_polygon(self):
        self.handle = None
        try:
            vertices = self.stance.compute_static_equilibrium_polygon(
                method=self._method)
            self.handle = draw_polygon(
                [(x[0], x[1], self.height) for x in vertices],
                normal=[0, 0, 1], color=self.color)
        except Exception as e:
            print("CoMPolygonDrawer: {}".format(e))

    def method(self, method):
        self._method = method
        self.update_polygon()


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
        ineq=(A, b), eq=(C, d), proj=(E, f), method="bretl")


class InstantaneousActuationDependentPolytopeDrawer(CoMPolygonDrawer):

    """
    Draw the static-equilibrium polygon of a contact set.

    Parameters
    ----------
    stance : Stance
        Contacts and COM position of the robot.
    color : tuple or string, optional
        Area color.
    """

    def __init__(self, robot, stance, height, color):
        self.last_com = robot.com
        self.robot = robot
        self.last_vertices = None
        # parent constructor is called after
        super(InstantaneousActuationDependentPolytopeDrawer, self).__init__(
            stance, height, color)

    def on_tick(self, sim):
        if norm(self.robot.com - self.last_com) > 1e-2:
            self.last_com = self.robot.com
            self.update_polygon()
        super(InstantaneousActuationDependentPolytopeDrawer, self).on_tick(sim)

    def draw_polytope_slice(self):
        vertices_2d = compute_actuation_dependent_polygon(
            self.robot, self.stance)
        vertices = [(x[0], x[1], robot.com[2]) for x in vertices_2d]
        if self.last_vertices is not None:
            self.handle.append(draw_polytope(
                self.last_vertices + vertices, color=[0.3, 0.6, 0.6, 0.6]))
        self.last_vertices = vertices

    def draw_polygon(self):
        vertices_2d = compute_actuation_dependent_polygon(
            self.robot, self.stance)
        vertices = [(x[0], x[1], robot.com[2]) for x in vertices_2d]
        self.handle.append(draw_polygon(
            vertices, normal=[0, 0, 1], color=[0.3, 0.3, 0.6, 0.5]))

    def update_polygon(self):
        self.handle = []
        self.last_vertices = None
        robot.show_com()
        with sim.env:
            com_height = self.stance.com.z
            q_init = self.robot.q
            for height in numpy.arange(0.63, 0.95, 0.03):
                self.stance.com.set_z(height)
                self.robot.ik.solve(warm_start=True, impr_stop=1e-3)
                self.draw_polytope_slice()
                self.handle.append(draw_point(robot.com))
            self.stance.com.set_z(com_height)
            self.robot.set_dof_values(q_init)


def set_torque_limits(robot):
    robot.tau_max = 5. * ones(robot.nb_dofs)
    robot.tau_max[robot.R_HIP_Y] = 100.
    robot.tau_max[robot.R_HIP_R] = 50.
    robot.tau_max[robot.R_HIP_P] = 100.
    robot.tau_max[robot.R_KNEE_P] = 50.
    robot.tau_max[robot.R_ANKLE_P] = 100.
    robot.tau_max[robot.R_ANKLE_R] = 100.
    robot.tau_max[robot.L_HIP_Y] = 100.
    robot.tau_max[robot.L_HIP_R] = 50.
    robot.tau_max[robot.L_HIP_P] = 100.
    robot.tau_max[robot.L_KNEE_P] = 50.
    robot.tau_max[robot.L_ANKLE_P] = 100.
    robot.tau_max[robot.L_ANKLE_R] = 100.
    robot.tau_max[robot.CHEST_P] = 100.
    robot.tau_max[robot.CHEST_Y] = 100.
    # robot.tau_max[robot.WAIST_R] = 100.
    robot.tau_max[robot.R_SHOULDER_P] = 50.
    robot.tau_max[robot.R_SHOULDER_R] = 50.
    robot.tau_max[robot.R_SHOULDER_Y] = 50.
    robot.tau_max[robot.R_ELBOW_P] = 50.
    robot.tau_max[robot.L_SHOULDER_P] = 50.
    robot.tau_max[robot.L_SHOULDER_R] = 50.
    robot.tau_max[robot.L_SHOULDER_Y] = 50.
    robot.tau_max[robot.L_ELBOW_P] = 50.
    robot.tau_max[robot.TRANS_X] = 1000
    robot.tau_max[robot.TRANS_Y] = 1000
    robot.tau_max[robot.TRANS_Z] = 1000
    robot.tau_max[robot.ROT_R] = 1000
    robot.tau_max[robot.ROT_P] = 1000
    robot.tau_max[robot.ROT_Y] = 1000


if __name__ == "__main__":
    sim = pymanoid.Simulation(dt=0.03)
    robot = pymanoid.robots.JVRC1()
    set_torque_limits(robot)

    q_max = robot.q_max.copy()
    q_max[robot.CHEST_P] = 0
    q_max[robot.ROT_P] = 0.5
    robot.set_dof_limits(robot.q_min, q_max)

    q_fingers = array([+1., -1.5, -1.5, +0.1, -0.5, -0.5])
    robot.set_dof_values(
        q_fingers,
        [robot.L_LTHUMB, robot.L_LINDEX, robot.L_LLITTLE, robot.L_UTHUMB,
         robot.L_UINDEX, robot.L_ULITTLE])
    robot.set_dof_values(
        -q_fingers,
        [robot.R_LTHUMB, robot.R_LINDEX, robot.R_LLITTLE, robot.R_UTHUMB,
         robot.R_UINDEX, robot.R_ULITTLE])

    sim.set_viewer()
    # sim.set_camera_left(x=0.2, y=4)
    sim.set_camera_transform(array([
        [-0.75318301, -0.33670976,  0.56510344, -1.27825475],
        [-0.65389426,  0.28962469, -0.69895625,  1.3535881],
        [0.07167748, -0.89595987, -0.43831297, 1.87996328],
        [0.,  0., 0.,  1.]]))
    robot.set_transparency(0.25)

    stance = Stance.from_json('jvrc1_ladder.json')
    stance.bind(robot)
    # stance.com.hide()
    robot.ik.maximize_margins = True
    robot.ik.verbosity = 0
    robot.ik.solve(impr_stop=1e-3)

    polygon_height = 2.  # [m]

    polygon_drawer = CoMPolygonDrawer(
        stance, polygon_height, color='g')
    iap_drawer = InstantaneousActuationDependentPolytopeDrawer(
        robot, stance, polygon_height, color='b')
    wrench_drawer = StaticEquilibriumWrenchDrawer(stance)

    sim.schedule(robot.ik)
    # sim.schedule_extra(polygon_polygon_drawer)
    # sim.schedule_extra(iap_polygon_drawer)
    sim.schedule_extra(wrench_drawer)
    sim.start()

    if IPython.get_ipython() is None:
        IPython.embed()
