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

from numpy import array

import pymanoid

from pymanoid import Stance
# from pymanoid.gui import StaticEquilibriumWrenchDrawer
from pymanoid.gui import draw_polygon, draw_polytope
from pymanoid.misc import norm

from pymanoid_common import ActuationDependentArea
from pymanoid_common import CoMPolygonDrawer
from pymanoid_common import compute_geom_reachable_polygon
from pymanoid_common import compute_local_actuation_dependent_polygon
from pymanoid_common import draw_polygon_at_height
from pymanoid_common import set_torque_limits


class ActuationDependentPolytopeDrawer(CoMPolygonDrawer):

    """
    Draw the static-equilibrium polygon of a contact set under instantaneous
    actuation constraints.

    Parameters
    ----------
    stance : Stance
        Contacts and COM position of the robot.
    """

    def __init__(self, robot, stance):
        self.last_com = robot.com
        self.robot = robot
        self.last_vertices = None
        # parent constructor is called after
        super(ActuationDependentPolytopeDrawer, self).__init__(
            stance)

    def on_tick(self, sim):
        if norm(self.robot.com - self.last_com) > 1e-2:
            self.last_com = self.robot.com
            self.update_handle()
        super(ActuationDependentPolytopeDrawer, self).on_tick(sim)

    def draw_polytope_slice(self):
        vertices_2d = compute_local_actuation_dependent_polygon(
            self.robot, self.stance)
        vertices = [(x[0], x[1], robot.com[2]) for x in vertices_2d]
        if self.last_vertices is not None:
            self.handle.append(draw_polytope(
                self.last_vertices + vertices, color=[0.3, 0.6, 0.6, 0.6]))
        self.last_vertices = vertices

    def draw_polygon(self):
        vertices_2d = compute_local_actuation_dependent_polygon(
            self.robot, self.stance)
        vertices = [(x[0], x[1], robot.com[2]) for x in vertices_2d]
        self.handle.append(draw_polygon(
            vertices, normal=[0, 0, 1], color=[0.3, 0.3, 0.6, 0.5]))

    def update_handle(self):
        self.handle = []
        self.last_vertices = None


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
    sim.set_camera_transform(array([
        [-0.75318301, -0.33670976,  0.56510344, -1.27825475],
        [-0.65389426,  0.28962469, -0.69895625,  1.3535881],
        [0.07167748, -0.89595987, -0.43831297, 1.87996328],
        [0.,  0., 0.,  1.]]))
    robot.set_transparency(0.25)

    stance = Stance.from_json('jvrc1_ladder_climbing.json')
    stance.bind(robot)

    robot.ik.maximize_margins = True
    robot.ik.verbosity = 0
    robot.ik.solve(impr_stop=1e-3)

    del robot.ik.tasks['left_hand_palm']
    del robot.ik.tasks['right_hand_palm']
    from pymanoid.tasks import AxisAngleContactTask
    lh_task = AxisAngleContactTask(
        robot, robot.left_hand, stance.left_hand, weight=1, gain=0.8)
    rh_task = AxisAngleContactTask(
        robot, robot.right_hand, stance.right_hand, weight=1, gain=0.8)
    lh_task.doc_mask = array([1., 1., 1., 1., 0.1, 1.])
    rh_task.doc_mask = array([1., 1., 1., 1., 0.1, 1.])
    robot.ik.add(lh_task)
    robot.ik.add(rh_task)

    uncons_polygon = stance.compute_static_equilibrium_polygon(method="bretl")

    CHECK_FULL_GEOM = False
    if CHECK_FULL_GEOM:
        geom_polygon = compute_geom_reachable_polygon(
            robot, stance, xlim=(0.0, 0.2), ylim=(-0.2, 0.2),)
    else:
        x_min, x_max = -0.15, 0.00
        y_min, y_max = -0.17, 0.17
        geom_polygon = [
            (x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]

    polygon_height = stance.com.z
    h1 = draw_polygon_at_height(uncons_polygon, polygon_height, color='g')
    h2 = draw_polygon_at_height(geom_polygon, polygon_height, color='m')

    sim.schedule(robot.ik)
    # sim.schedule_extra(wrench_drawer)
    sim.start()

    ada = ActuationDependentArea(robot, stance)
    working_set = geom_polygon
    actdep_area = ada.compute(working_set)
    h3 = draw_polygon_at_height(actdep_area, polygon_height, color='b')

    if IPython.get_ipython() is None:
        IPython.embed()
