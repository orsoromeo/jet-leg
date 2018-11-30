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

import pymanoid

from pymanoid import Stance
from pymanoid.gui import StaticEquilibriumWrenchDrawer
from pymanoid.gui import draw_point

from pymanoid_common import ActuationDependentArea
from pymanoid_common import LocalActuationDependentPolygonDrawer
from pymanoid_common import draw_horizontal_polygon
from pymanoid_common import set_torque_limits


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

    uncons_polygon = stance.compute_static_equilibrium_polygon(method="cdd")
    h1 = draw_horizontal_polygon(uncons_polygon, polygon_height, color='g')

    sim.schedule(robot.ik)
    sim.start()

    ada = ActuationDependentArea(robot, stance)
    ada.sample_working_set(uncons_polygon, "sample", 10)
    h2 = ada.draw_at_height(polygon_height)

    PLAY_WITH_LOCAL_POLYGONS = True
    if PLAY_WITH_LOCAL_POLYGONS:
        com_sync = COMSync(robot, stance, com_above)
        local_polygon_drawer = LocalActuationDependentPolygonDrawer(
            robot, stance, polygon_height)
        wrench_drawer = StaticEquilibriumWrenchDrawer(stance)
        sim.schedule_extra(com_sync)
        sim.schedule_extra(local_polygon_drawer)
        sim.schedule_extra(wrench_drawer)

    if IPython.get_ipython() is None:
        IPython.embed()
