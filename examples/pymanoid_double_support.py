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
import pymanoid

from pymanoid import Stance
from pymanoid.gui import StaticEquilibriumWrenchDrawer

from pymanoid_common import ActuationDependentArea
from pymanoid_common import LocalActuationDependentPolygonDrawer
from pymanoid_common import draw_horizontal_polygon
from pymanoid_common import set_torque_limits


if __name__ == "__main__":
    sim = pymanoid.Simulation(dt=0.03)
    robot = pymanoid.robots.JVRC1()
    set_torque_limits(robot)
    robot.show_com()

    sim.set_viewer()
    sim.set_camera_transform(numpy.array([
        [-0.75166831, -0.47792323, 0.45451529, -0.99502754],
        [-0.65817994, 0.49930137, -0.563469, 1.14903212],
        [0.04235481, -0.72269463, -0.68986849, 2.35493684],
        [0.,  0.,  0.,  1.]]))
    robot.set_transparency(0.25)

    stance = Stance.from_json('jvrc1_double_support.json')
    stance.bind(robot)
    robot.ik.solve()

    polygon_height = stance.com.z  # [m]
    uncons_polygon = stance.compute_static_equilibrium_polygon(method="cdd")
    h1 = draw_horizontal_polygon(uncons_polygon, polygon_height, color='g')

    sim.schedule(robot.ik)
    sim.start()

    ada = ActuationDependentArea(robot, stance)
    ada.sample_working_set(uncons_polygon, "sample", 20)

    PLAY_WITH_LOCAL_POLYGONS = False
    if PLAY_WITH_LOCAL_POLYGONS:
        local_polygon_drawer = LocalActuationDependentPolygonDrawer(
            robot, stance, polygon_height)
        wrench_drawer = StaticEquilibriumWrenchDrawer(stance)
        sim.schedule_extra(local_polygon_drawer)
        sim.schedule_extra(wrench_drawer)

    DRAW_VOLUME = True
    if DRAW_VOLUME:
        h2 = ada.draw_volume(
            min_height=0.5, max_height=1.0, dh=0.05, hull=True)
    else:  # not DRAW_VOLUME
        h2 = ada.draw_at_height(polygon_height)

    if IPython.get_ipython() is None:
        IPython.embed()
