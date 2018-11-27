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
# import numpy
import pylab

from numpy import array, ones
from scipy.spatial import ConvexHull

import pymanoid

from pymanoid import Stance
from pymanoid.gui import StaticEquilibriumWrenchDrawer
from pymanoid.gui import draw_point, draw_polygon, draw_polytope
from pymanoid.misc import norm

from pymanoid_common import CoMPolygonDrawer
from pymanoid_common import compute_local_actuation_dependent_polygon
from pymanoid_common import generate_point_grid


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
        robot.show_com()
        init_com = self.stance.com.p
        q_init = self.robot.q
        self.stance.com.hide()
        grid = generate_point_grid(
            xlim=(0.0, 0.2), ylim=(-0.2, 0.2), zlim=(0.7, 0.85),
            xres=4, yres=6)
        for (height, points) in grid:
            self.stance.com.set_z(height)
            feasible_points = []
            for point in points:
                self.stance.com.set_x(init_com[0] + point[0])
                self.stance.com.set_y(init_com[1] + point[1])
                self.__com_target = draw_point(
                    self.stance.com.p, pointsize=0.01, color='m')
                self.robot.ik.solve(warm_start=True)
                # self.draw_polytope_slice()
                # self.draw_polygon()
                if pylab.norm(self.robot.com - self.stance.com.p) < 0.02:
                    self.handle.append(draw_point(robot.com, pointsize=5e-4))
                    feasible_points.append(robot.com)
                # self.handle.append(draw_point(self.stance.com.p))
            feasible_2d = [array([p[0], p[1]]) for p in feasible_points]
            hull = ConvexHull(feasible_2d)
            vertices_2d = [feasible_2d[i] for i in hull.vertices]
            z_avg = pylab.mean([p[2] for p in feasible_points])
            vertices = [array([v[0], v[1], z_avg]) for v in vertices_2d]
            self.handle.extend([draw_point(v) for v in vertices])
            break
        self.stance.com.set_pos(init_com)
        self.stance.com.show()
        self.robot.set_dof_values(q_init)
        self.__com_target = None


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

    uncons_drawer = CoMPolygonDrawer(stance)
    act_drawer = ActuationDependentPolytopeDrawer(robot, stance)
    wrench_drawer = StaticEquilibriumWrenchDrawer(stance)

    sim.schedule(robot.ik)
    # sim.schedule_extra(polygon_drawer)
    # sim.schedule_extra(polytope_drawer)
    sim.schedule_extra(wrench_drawer)
    sim.start()

    if IPython.get_ipython() is None:
        IPython.embed()
