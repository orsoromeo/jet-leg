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

from numpy import array, ones

import pymanoid

from pymanoid import Stance
from pymanoid.gui import StaticEquilibriumWrenchDrawer
from pymanoid.gui import draw_point, draw_polygon
from pymanoid.misc import norm

from pymanoid_common import CoMPolygonDrawer
from pymanoid_common import compute_actuation_dependent_polygon
from pymanoid_common import shrink_polygon


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

    def __init__(self, robot, stance, com_above):
        super(COMSync, self).__init__()
        com_above.set_transparency(0.2)
        self.com_above = com_above
        self.stance = stance
        self.robot_com = None
        self.robot = robot

    def on_tick(self, sim):
        self.stance.com.set_x(self.com_above.x)
        self.stance.com.set_y(self.com_above.y)
        self.robot_com = draw_point(
            [self.robot.com[0], self.robot.com[1], self.com_above.z],
            color='m')


class ActuationDependentPolygonDrawer(CoMPolygonDrawer):

    """
    Draw the static-equilibrium polygon of a contact set.

    Parameters
    ----------
    stance : Stance
        Contacts and COM position of the robot.
    """

    def __init__(self, robot, stance, height):
        self.last_com = robot.com
        self.robot = robot
        # parent constructor is called after
        super(ActuationDependentPolygonDrawer, self).__init__(
            stance, height)

    def on_tick(self, sim):
        if norm(self.robot.com - self.last_com) > 1e-2:
            self.last_com = self.robot.com
            self.update_polygon()
        super(ActuationDependentPolygonDrawer, self).on_tick(sim)

    def update_polygon(self):
        self.handle = None
        try:
            vertices = compute_actuation_dependent_polygon(
                self.robot, self.stance)
            self.handle = draw_polygon(
                [(x[0], x[1], self.height) for x in vertices],
                normal=[0, 0, 1])
        except Exception as e:
            print("ActuationDependentPolygonDrawer: {}".format(e))


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
    com_above = pymanoid.Cube(0.02, [0.05, 0.04, polygon_height])

    stance = Stance.from_json('jvrc1_double_support.json')
    stance.bind(robot)
    stance.com.hide()
    robot.ik.solve()

    com_sync = COMSync(robot, stance, com_above)
    act_polygon_drawer = ActuationDependentPolygonDrawer(
        robot, stance, polygon_height)
    wrench_drawer = StaticEquilibriumWrenchDrawer(stance)

    uncons_polygon_drawer = CoMPolygonDrawer(stance, polygon_height)
    uncons_polygon_drawer.update()
    working_polygon = shrink_polygon(
        uncons_polygon_drawer.vertices, shrink_ratio=0.75, res=10)
    h1 = draw_polygon(
        [(v[0], v[1], polygon_height) for v in working_polygon],
        normal=[0, 0, 1], combined='m-#')
    h2 = [draw_point(
        [v[0], v[1], polygon_height], color='m') for v in working_polygon]

    sim.schedule(robot.ik)
    sim.schedule_extra(com_sync)
    # sim.schedule_extra(uncons_polygon_drawer)
    sim.schedule_extra(act_polygon_drawer)
    # sim.schedule_extra(wrench_drawer)
    sim.start()

    for vertex in working_polygon:
        com_above = array([vertex[0], vertex[1], polygon_height])
        com_sync.com_above.set_pos(com_above)
        raw_input()

    if IPython.get_ipython() is None:
        IPython.embed()
