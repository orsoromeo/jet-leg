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

from numpy import array, hstack, ones, vstack, zeros
from scipy.linalg import block_diag

import pymanoid

from pymanoid import Stance
from pymanoid.gui import StaticEquilibriumWrenchDrawer
from pymanoid.gui import draw_point, draw_polygon
from pymanoid.misc import norm
from pymanoid.sim import gravity_const
from pypoman import project_polytope


class COMSync(pymanoid.Process):

    """
    Update stance CoM from the GUI handle in polygon above the robot.

    Parameters
    ----------
    stance : pymanoid.Stance
        Contacts and COM position of the robot.
    com_above : pymanoid.Cube
        CoM handle in static-equilibrium polygon.
    """

    def __init__(self, stance, com_above):
        super(COMSync, self).__init__()
        self.com_above = com_above
        self.stance = stance

    def on_tick(self, sim):
        self.stance.com.set_x(self.com_above.x)
        self.stance.com.set_y(self.com_above.y)


class UnconstrainedPolygonDrawer(pymanoid.Process):

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
        super(UnconstrainedPolygonDrawer, self).__init__()
        self._method = "cdd"
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
            print("UnconstrainedPolygonDrawer: {}".format(e))

    def method(self, method):
        self._method = method
        self.update_polygon()


def compute_actuation_dependent_polygon(robot, contacts, tau_scale, method):
    """
    Compute constraint matrices of the problem:

        A * w_all  <=  b
        C * w_all  ==  d

    and output variables are given by:

        [x_com y_com]  =  E * w_all + f

    where w_all is the stacked vector of external contact wrenches.
    """
    g = robot.compute_static_gravity_torques()
    J_contact = robot.compute_contact_jacobian(contacts)
    tau_max = tau_scale * robot.tau_max

    # Friction limits:
    #
    #     A_fric * w_all <= b_fric
    #
    A_fric = block_diag(*[c.wrench_inequalities for c in contacts.contacts])
    b_fric = zeros((A_fric.shape[0],))

    # Torque limits:
    #
    #     |tau| <= tau_max
    #
    # where tau == g - J_c^T w_all in static equilibrium.
    #
    A_act = vstack([-J_contact.T[:-6], +J_contact.T[:-6]])
    b_act = hstack([(tau_max - g)[:-6], (tau_max + g)[:-6]])

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
        ineq=(A, b), eq=(C, d), proj=(E, f), method=method)


class ActuationDependentPolygonDrawer(UnconstrainedPolygonDrawer):

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
        self._tau_scale = 1.0
        self.last_com = robot.com
        self.robot = robot
        # parent constructor is called after
        super(ActuationDependentPolygonDrawer, self).__init__(
            stance, height, color)

    def on_tick(self, sim):
        if norm(self.robot.com - self.last_com) > 1e-2:
            self.last_com = self.robot.com
            self.update_polygon()
        super(ActuationDependentPolygonDrawer, self).on_tick(sim)

    def update_polygon(self):
        self.handle = None
        try:
            vertices = compute_actuation_dependent_polygon(
                self.robot, self.stance, self._tau_scale, method=self._method)
            self.handle = draw_polygon(
                [(x[0], x[1], self.height) for x in vertices],
                normal=[0, 0, 1], color=self.color)
        except Exception as e:
            print("ActuationDependentPolygonDrawer: {}".format(e))

    def tau_scale(self, tau_scale):
        self._tau_scale = min(1., max(0., tau_scale))
        self.update_polygon()


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
    robot.tau_max[robot.WAIST_R] = 100.
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
    sim.set_viewer()
    sim.set_camera_top(x=0., y=0., z=2.8)
    robot.set_transparency(0.25)

    polygon_height = 2.  # [m]
    com_above = pymanoid.Cube(0.02, [0.05, 0.04, polygon_height], color='b')

    stance = Stance.from_json('jvrc1_double_support.json')
    stance.bind(robot)
    stance.com.hide()
    robot.ik.solve()

    com_sync = COMSync(stance, com_above)
    uncons_polygon_drawer = UnconstrainedPolygonDrawer(
        stance, polygon_height, color='g')
    act_polygon_drawer = ActuationDependentPolygonDrawer(
        robot, stance, polygon_height, color='b')
    wrench_drawer = StaticEquilibriumWrenchDrawer(stance)

    uncons_polygon_drawer.method("cdd")
    act_polygon_drawer.method("cdd")
    act_polygon_drawer.tau_scale(1.0)

    feasible_coms = []
    feasible_coms.append(draw_point(com_above.p, pointsize=0.01))

    sim.schedule(robot.ik)
    sim.schedule_extra(com_sync)
    sim.schedule_extra(uncons_polygon_drawer)
    sim.schedule_extra(act_polygon_drawer)
    sim.schedule_extra(wrench_drawer)
    sim.start()

    if IPython.get_ipython() is None:
        IPython.embed()
