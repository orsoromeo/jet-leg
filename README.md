<img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/feasible_region.png" alt="hyqgreen" width="400"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/four_stance.png" alt="planning" width="400"/>
<img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/force_polygons.png" alt="hyqgreen" width="400"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/foothold_planning.png" alt="planning" width="400"/>


# Feasible Region: an Actuation-Aware Extension of the Support Region
This python library contains the code used for the motion planning formulation proposed in this [preprint](https://arxiv.org/abs/1903.07999#). In here you can also find the code used to generate the figures and plots of the manuscript. 

<img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/3contacts_F%26A.png" alt="hyqgreen" width="200"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/3contacts_onlyA.png" alt="planning" width="200"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/4contacts_F%26A.png" alt="hyqgreen" width="200"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/4contacts_onlyA.png" alt="planning" width="200"/>

Jet-leg performs common operations over [convex
polyhedra](https://en.wikipedia.org/wiki/Convex_polyhedron) in higher dimensions in order to assess the problem of stability and motion feasibility of legged robots.

## What you can do with Jet-leg:
- test an implamentation of Iterative Projection algorithms for the solution of geometrical problems related to legged locomotion stability and feasibility analysis. This includes the computation of the [Feasible Region](https://arxiv.org/abs/1903.07999#).
- compute force polytopes of legged robots;
- test various formulations of linear, convex or nonlinear optimization problems;


## Dependencies
APT dependencies:
- CVXOPT
- GLPK
- Cython

ROS dependencies:
```
sudo apt-get  install ros-kinetic-graph-msgs
```

Python dependencies:
- Numpy
- Scipy
- Pycddlib
- Matplotlib
- [Pypoman](https://github.com/stephane-caron/pypoman) for the manipulation of polyhedrical object

The above dependencies can be installed with the following commands:
```
sudo apt-get install cython libglpk-dev python python-dev python-pip python-scipy
CVXOPT_BUILD_GLPK=1 pip install cvxopt --user
pip install pycddlib --user
pip install pypoman
```
You can remove all ``--user`` arguments to install these Python modules system-wide.

## Optional dependencies:

- [Ipopt](https://projects.coin-or.org/Ipopt) and its Python interface [Pypi](https://pypi.org/project/ipopt/) for the solution of large-scale nonlinear optimization problems
- [ffmpeg](https://www.ffmpeg.org/) for the generation of Matplotlib animations
```
sudo apt-get install ffmpeg
```
- [unittest](https://docs.python.org/3/library/unittest.html) for testing of dependencies installation and for development


<!--## Installation (no longer used)

Finally, clone this repository and run its setup script:
```
git clone git@gitlab.advr.iit.it:rorsolino/jet-leg.git
cd jet-leg
python setup.py build
python setup.py install --user
```
-->

## Testing the library
Jet-Leg comes with a number of unit tests intended to check the proper installation of the packages and make sure that all the mandatory depedencies are properly found. For this purpose, after completing the installation navigate to the [unit_test_main.py](https://gitlab.advr.iit.it/rorsolino/jet-leg/blob/master/unit_tests/unit_test_main.py) and run it.

After that, navigate to the [examples folder](https://gitlab.advr.iit.it/rorsolino/jet-leg/tree/master/examples) to find more explanation on the usage of the package. Check for example:

- [single_iterative_projection](https://github.com/orsoromeo/jet-leg/blob/master/examples/single_iterative_projection_example.py) can be used to see how to set up an iterative projection problem in order to compute the friction/actuation/feasible region;
- [single_LP_example](https://github.com/orsoromeo/jet-leg/blob/master/examples/single_LP_example.py) can be used to see how to solve a feasibility problem that checks whether the CoM projection belongs to the friction/actuation/feasible region or not

## Troubleshooting

- if CVXOPT is not found even after trying the pip-installation, we then suggest to try install the version 1.1.4 of CVXOPT using Synaptic or to clone and install it manually after building.
- IMPORTANTE NOTE: delete every previous installation of cvxopt that is in the system using locate cvxopt (after sudo updatedb)

## See also

- The [pypoman](https://github.com/stephane-caron/pypoman) and [pymanoid](https://github.com/stephane-caron/pymanoid) libraries developed by Stéphane Caron
- Komei Fukuda's [Frequently Asked Questions in Polyhedral Computation](http://www.cs.mcgill.ca/~fukuda/soft/polyfaq/polyfaq.html)
- The
  [Polyhedron](http://doc.sagemath.org/html/en/reference/discrete_geometry/sage/geometry/polyhedron/constructor.html) class in [Sage](http://www.sagemath.org/)
- The [StabiliPy](https://github.com/haudren/stabilipy) package provides a more
  general recursive projection method
