#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2018 Romeo Orsolino <romeo.orsolino@iit.it>
#
# This file is part of jet_leg.
#
# jet_leg is free software: you can redistribute it and/or modify it under the
# terms of the GNU Lesser General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option) any
# later version.
#
# jet_leg is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
# details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with jet_leg. If not, see <http://www.gnu.org/licenses/>.

from setuptools import find_packages, setup


setup(
    name='jet_leg',
    version='0.2.0',
    description="Python Polyhedron Manipulation",
    url="https://github.com/orsoromeo/jet-leg",
    author="Romeo Orsolino",
    author_email="orso.romeo@gmail.com",
    license="LGPL",
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)',
        'Operating System :: POSIX :: Linux',
        'Programming Language :: Python :: 3.8',
        'Topic :: Scientific/Engineering :: Mathematics'],
    packages=find_packages()
)
