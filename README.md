<img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/feasible_region.png" alt="hyqgreen" width="400"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/four_stance.png" alt="planning" width="400"/>
<img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/force_polygons.png" alt="hyqgreen" width="400"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/foothold_planning.png" alt="planning" width="400"/>


# Feasible Region: an Actuation-Aware Extension of the Support Region
This python library contains the code used for the motion planning formulation proposed in this [preprint](https://arxiv.org/abs/1903.07999#). In here you can also find the code used to generate the figures and plots of the manuscript. 

<img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/3contacts_F%26A.png" alt="hyqgreen" width="200"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/3contacts_onlyA.png" alt="planning" width="200"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/4contacts_F%26A.png" alt="hyqgreen" width="200"/>  <img src="https://github.com/orsoromeo/jet-leg/blob/master/figs/4contacts_onlyA.png" alt="planning" width="200"/>

Jet-leg performs common operations over [convex
polyhedra](https://en.wikipedia.org/wiki/Convex_polyhedron) in higher dimensions in order to assess the problem of stability and motion feasibility of legged robots.

## What you can do with Jet-leg:
- compute the Support region of legged robots as in [Bretl. et al. 2008](https://ieeexplore.ieee.org/abstract/document/4598894); 
- compute the Feasible region of legged robots as in [Orsolino. et al. 2019](https://arxiv.org/abs/1903.07999#);
- compute force polytopes of legged robots given their URDF;
- compare different leg designs and understand their consequences on the robot's balancing capabilities; 
- test various formulations of linear, convex or nonlinear trajectory optimization problems;

## Quick installation
You can directly install the pre-built library usign pip. First, create a virtual environment:
```
python -m venv my_project
cd source/bin/activate
```
Then, install the required dependencies:
```
sudo apt-get install -y cython libglpk-dev python3-tk
pip install pytest numpy scipy matplotlib pycddlib pypoman pin 
```
and finally run:
```
pip install jet-leg
```
To make sure that the installation worked as expected, you can make sure that the following command runs without errors:
```
python3 -c "import jet_leg"
```
Have a look to the [this](https://github.com/orsoromeo/jet-leg/tree/master/examples) folder for a few examples on how to use the library. 

## Build from source
For active development of the library a source installation is recommended. For this, you will need to clone the repo first. Then, you can build the package using the provided Docker environment. Do so by running the following command:
```
cd jet-leg
./build.sh
```
This will create a docker image with all the required dependencies. You can then attach the image by running:
```
./run.sh
```
To make sure that the installation worked successfully, you can run, for example:
```
python3 examples/iterative_projection/single_iterative_projection_example.py
```
The example above should generate two figures representing the feasible region of the Anymal robot in a default configuration in 2D and 3D.

## Python dependencies:
- Numpy
- Scipy
- Pycddlib
- Matplotlib
- [Pypoman](https://github.com/stephane-caron/pypoman) for the manipulation of polyhedrical object
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 

<!-- The above dependencies can be installed with the following commands:
```
sudo apt-get install cython libglpk-dev python python-dev python-pip python-scipy
CVXOPT_BUILD_GLPK=1 pip install cvxopt --user
pip install pycddlib --user
pip install pypoman
```
You can remove all ``--user`` arguments to install these Python modules system-wide. -->

## Optional dependencies:

- [Ipopt](https://projects.coin-or.org/Ipopt) and its Python interface [Pypi](https://pypi.org/project/ipopt/) for the solution of large-scale nonlinear optimization problems
- [ffmpeg](https://www.ffmpeg.org/) for the generation of Matplotlib animations
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

<!-- ## Testing the library
After completing the installation navigate to the [examples](https://gitlab.advr.iit.it/rorsolino/jet-leg/tree/master/examples) folder:

- [single_iterative_projection_example.py](https://github.com/orsoromeo/jet-leg/blob/master/examples/iterative_projection/single_iterative_projection_example.py) can be used to see how to set up an iterative projection problem in order to compute the friction/actuation/feasible region;
- [check_stability_lp_example.py](https://github.com/orsoromeo/jet-leg/blob/master/examples/static_equilibrium_check/check_stability_lp_example.py) can be used to quickly check whether the given robot configuration is statically stable or not (without explicitly computing the feasible region);
- [plotIPstatistics.py](https://github.com/orsoromeo/jet-leg/blob/master/examples/figures_code/plotIPstatistics.py) can be used to generate some statistics about the computation time of the IP algorithm for random feet positions (see Fig. 6 of the [preprint](https://arxiv.org/abs/1903.07999#));
- [plotInstantaneousActuationRegionVariableMass.py](https://github.com/orsoromeo/jet-leg/blob/master/examples/figures_code/plotInstantaneousActuationRegionVariableMass.py) can be used to generate a plot that shows how the feasible regions can changes depending on the gravitational force acting on the robot's center of mass (see Fig. 8 of the [preprint](https://arxiv.org/abs/1903.07999#))  -->

## Troubleshooting

- Jet-leg has been tested using Python 3.8.17 and Docker;
- If CVXOPT is not found even after trying the pip-installation, we then suggest to try install the version 1.1.4 of CVXOPT using Synaptic or to clone and install it manually after building. Note: delete every previous installation of cvxopt that is in the system using locate cvxopt (after sudo updatedb)

## See also

- The [pypoman](https://github.com/stephane-caron/pypoman) and [pymanoid](https://github.com/stephane-caron/pymanoid) libraries developed by Stéphane Caron
- Komei Fukuda's [Frequently Asked Questions in Polyhedral Computation](http://www.cs.mcgill.ca/~fukuda/soft/polyfaq/polyfaq.html)
- The
  [Polyhedron](http://doc.sagemath.org/html/en/reference/discrete_geometry/sage/geometry/polyhedron/constructor.html) class in [Sage](http://www.sagemath.org/)
- The [StabiliPy](https://github.com/haudren/stabilipy) package provides a more
  general recursive projection method
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) is used to compute the Jacobian matrix and to solve the Inverse Kinematics (IK) problem.
