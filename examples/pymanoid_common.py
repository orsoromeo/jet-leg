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

import pylab

from numpy import arange, array, hstack, ones, vstack, zeros
from numpy import cos, dot, sin, pi, sqrt
from numpy import linspace, random
from scipy.linalg import block_diag
from scipy.spatial import ConvexHull

import pymanoid
import pypoman

from pymanoid.gui import draw_horizontal_polygon
from pymanoid.gui import draw_point, draw_polygon, draw_polytope
from pymanoid.misc import norm
from pymanoid.sim import gravity_const
from pypoman import compute_chebyshev_center
from pypoman import compute_polytope_halfspaces
from pypoman import project_polytope


def compute_local_actuation_dependent_polygon(robot, contacts, method="bretl"):
    """
    Compute constraint matrices of the problem:

        A * w_all  <=  b
        C * w_all  ==  d

    and output variables are given by:

        [x_com y_com]  =  E * w_all + f

    where w_all is the stacked vector of external contact wrenches.

    Parameters
    ----------
    robot : pymanoid.Robot
        Robot model.
    contacts : pymanoid.ContactSet
        Contacts with the environment.
    method : string, optional
        Choice between 'bretl' and 'cdd'.

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
        ineq=(A, b), eq=(C, d), proj=(E, f),
        method=method, max_iter=100)


def generate_2d_grid(xlim, ylim, xres, yres):
    assert xres % 2 == 0 and yres % 2 == 0
    p = zeros(2)
    dx = (xlim[1] - xlim[0]) / xres
    dy = (ylim[1] - ylim[0]) / yres
    x_avg = (xlim[1] + xlim[0]) / 2.
    y_avg = (ylim[1] + ylim[0]) / 2.
    dy_sign = +1
    points = []
    p[0] = x_avg - dx * (1 + xres / 2)
    p[1] = y_avg - dy * dy_sign * (1 + yres / 2)
    for _ in xrange(xres + 1):
        p[0] += dx
        for _ in xrange(yres + 1):
            p[1] += dy * dy_sign
            points.append(array([p[0], p[1]]))
        p[1] += dy * dy_sign
        dy_sign *= -1.
    return points


def generate_3d_grid(xlim, ylim, zlim, xres, yres):
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


def shrink_polygon(vertices, shrink_ratio, res=10):
    """
    Shrink polygon from its Chebyshev center.

    Parameters
    ----------
    vertices : list of arrays
        Vertices of the initial polygon.
    shrink_ratio : scalar
        Scalar factor between 0 and 1.
    res : int
        Number of vertices for the shrunk polygon.

    Returns
    -------
    new_vertices : list of arrays
        Vertices of the new polygon.
    """
    assert 0. <= shrink_ratio <= 1. and type(res) is int and res > 0
    A, b = compute_polytope_halfspaces(vertices)
    c = compute_chebyshev_center(A, b)
    v = b - dot(A, c)
    n = len(v)
    new_vertices = []
    for ray_index in xrange(res):
        theta = ray_index * (2. * pi / res)
        r = array([cos(theta), sin(theta)])
        u = dot(A, r)
        lambda_ = min([v[i] / u[i] for i in xrange(n) if u[i] > 1e-10])
        new_vertices.append(c + shrink_ratio * lambda_ * r)
    return new_vertices


def sample_points_from_polygon(vertices, nb_points):
    A, b = compute_polytope_halfspaces(vertices)
    vertices = array(vertices)
    p_min = vertices.min(axis=0)
    p_max = vertices.max(axis=0)
    points = []
    while len(points) < nb_points:
        p = random.random(2) * (p_max - p_min) + p_min
        if (dot(A, p) <= b).all():
            points.append(p)
    return points


def sample_grid_from_polygon(vertices, res=None, xres=None, yres=None):
    xres = res if xres is None else xres
    yres = res if yres is None else yres
    A, b = compute_polytope_halfspaces(vertices)
    vertices = array(vertices)
    p_min = vertices.min(axis=0)
    p_max = vertices.max(axis=0)
    points = []
    for x in linspace(p_min[0], p_max[0], xres):
        for y in linspace(p_min[1], p_max[1], yres):
            p = array([x, y])
            if (dot(A, p) <= b).all():
                points.append(p)
    return points


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


class LocalActuationDependentPolygonDrawer(pymanoid.Process):

    """
    Draw the static-equilibrium polygon of a contact set.

    Parameters
    ----------
    stance : Stance
        Contacts and COM position of the robot.
    """

    def __init__(self, robot, stance, height, method="bretl"):
        super(LocalActuationDependentPolygonDrawer, self).__init__()
        self.handle = None
        self.height = height
        self.last_com = robot.com
        self.method = method
        self.robot = robot
        self.stance = stance
        self.vertices = None
        #
        self.update_handle()

    def on_tick(self, sim):
        if norm(self.robot.com - self.last_com) > 1e-2:
            self.last_com = self.robot.com
            self.update_polygon()
        if self.handle is None:
            self.update_handle()

    def update(self):
        self.update_handle()

    def update_polygon(self):
        self.handle = None
        try:
            vertices = compute_local_actuation_dependent_polygon(
                self.robot, self.stance)
            self.handle = draw_polygon(
                [(x[0], x[1], self.height) for x in vertices],
                normal=[0, 0, 1], color='m')
        except Exception as e:
            print("ActuationDependentPolygonDrawer: {}".format(e))

    def update_handle(self):
        self.handle = None
        try:
            self.vertices = self.stance.compute_static_equilibrium_polygon(
                method=self.method)
            self.handle = draw_polygon(
                [(x[0], x[1], self.height) for x in self.vertices],
                normal=[0, 0, 1], color='g')
        except Exception as e:
            print("CoMPolygonDrawer: {}".format(e))


class ActuationDependentArea(object):

    def __init__(self, robot, stance):
        self.all_vertices = []
        self.polygons = []
        self.robot = robot
        self.sample_handles = []
        self.stance = stance
        self.working_set = None

    def sample_working_set(self, polygon, ws_type, nb_points):
        if ws_type == "shrink":
            self.working_set = shrink_polygon(
                polygon, shrink_ratio=0.5, res=nb_points)
        elif ws_type == "sample":
            self.working_set = sample_points_from_polygon(polygon, nb_points)
        elif ws_type == "grid":
            res = int(sqrt(nb_points))
            self.working_set = sample_grid_from_polygon(polygon, res=res)

    def compute(self, draw_height=None):
        assert len(self.working_set) > 0 and len(self.working_set[0]) == 2
        self.all_vertices = []
        for i_cur, p_cur in enumerate(self.working_set):
            p_cur = array(p_cur)
            A_voronoi, b_voronoi = [], []
            for i_other, p_other in enumerate(self.working_set):
                if i_other == i_cur:
                    continue
                p_other = array(p_other)
                p_mid = 0.5 * (p_other + p_cur)
                a = p_other - p_cur
                A_voronoi.append(a)
                b_voronoi.append(dot(a, p_mid))
            A_voronoi = vstack(A_voronoi)
            b_voronoi = hstack(b_voronoi)

            self.stance.com.set_x(p_cur[0])
            self.stance.com.set_y(p_cur[1])
            self.robot.ik.solve(warm_start=True)
            proj_vertices = compute_local_actuation_dependent_polygon(
                self.robot, self.stance, method="bretl")
            A_proj, b_proj = compute_polytope_halfspaces(proj_vertices)
            A = vstack([A_proj, A_voronoi])
            b = hstack([b_proj, b_voronoi])
            if draw_height is not None and (dot(A, p_cur) > b).any():
                self.sample_handles.append(draw_point(
                    [p_cur[0], p_cur[1], draw_height], color='r',
                    pointsize=5e-3))
                continue
            elif draw_height is not None:
                self.sample_handles.append(draw_point(
                    [p_cur[0], p_cur[1], draw_height], color='g',
                    pointsize=5e-3))
            vertices = pypoman.compute_polytope_vertices(A, b)
            if draw_height is not None:
                self.polygons.append(draw_horizontal_polygon(
                    vertices, draw_height, combined='b-#'))
            self.all_vertices.extend(vertices)
        return self.all_vertices

    def draw_at_height(self, height):
        """
        Draw actuation-dependent CoM area.

        Parameters
        ----------
        height : scalar
            Drawing height.
        """
        if len(self.all_vertices) < 1:
            self.compute(height)
        return draw_horizontal_polygon(self.all_vertices, height, color='b')

    def draw_volume(self, min_height, max_height, dh, hull=False):
        """
        Draw actuation-dependent CoM volume.

        Parameters
        ----------
        min_height : scalar
            Minimum CoM height in [m].
        max_height : scalar
            Maximum CoM height in [m].
        dh : scalar, optional
            Height step in [m].
        hull : bool, optional
            Return convex hull of all actuation-dependent areas.
        """
        all_points = []
        handles = []
        last_area = None
        for height in arange(min_height, max_height, dh):
            self.stance.com.set_z(height)
            cur_area = self.compute()
            points = [[p[0], p[1], height] for p in cur_area]
            all_points.extend(points)
            if last_area is not None:
                last_points = [[p[0], p[1], height - dh] for p in last_area]
                handles.append(draw_polytope(points + last_points, color='b'))
            last_area = cur_area
        if hull:
            return draw_polytope(all_points, color='b')  # we know it's convex
        return handles


def compute_geom_reachable_polygon(robot, stance, xlim, ylim, draw=True):
    init_com = stance.com.p
    feasible_points = []
    handles = []
    points = generate_2d_grid(xlim, ylim, xres=4, yres=6)
    for point in points:
        stance.com.set_x(init_com[0] + point[0])
        stance.com.set_y(init_com[1] + point[1])
        robot.ik.solve(warm_start=True)
        # self.draw_polytope_slice()
        # draw_polygon()
        if norm(robot.com - stance.com.p) < 0.02:
            feasible_points.append(robot.com)
            if draw:
                handles.append(draw_point(robot.com, pointsize=5e-3))
        # handle.append(draw_point(stance.com.p))
    feasible = [array([p[0], p[1]]) for p in feasible_points]
    hull = ConvexHull(feasible)
    vertices = [feasible[i] for i in hull.vertices]
    if draw:
        z_avg = pylab.mean([p[2] for p in feasible_points])
        vertices_3d = [array([v[0], v[1], z_avg]) for v in vertices]
        handles.extend([draw_point(v) for v in vertices_3d])
    stance.com.set_pos(init_com)
    return vertices
