# Python library for equilibrium of legged robots

This library uses common operations over [convex
polyhedra](https://en.wikipedia.org/wiki/Convex_polyhedron) in higher dimensions in order to assess the problem of stability and motion feasibility of legged robots.

## Installation
APT dependencies:
- CVXOPT
- GLPK
- Cython

Python dependencies:
- numpy
- scipy
- pycddlib
- matplotlib

Install dependencies by:
```
sudo apt-get install cython libglpk-dev python python-dev python-pip python-scipy
CVXOPT_BUILD_GLPK=1 pip install cvxopt --user
pip install pycddlib --user
```
You can remove all ``--user`` arguments to install these Python modules system-wide.

Optional dependencies:

- ipopt
- ffmpeg for matplotlib animations
- unittest

For installing the optional dependencies run the following command:

```
sudo apt-get install ffmpeg
```
Finally, clone this repository and run its setup script:
```
git clone git@github.com:orsoromeo/legsthrust.git
cd legsthrust
python setup.py build
python setup.py install --user
```

## See also

- The [pypoman](https://github.com/stephane-caron/pypoman) and [pymanoid](https://github.com/stephane-caron/pymanoid) libraries developed by Stéphane Caron
- Komei Fukuda's [Frequently Asked Questions in Polyhedral Computation](http://www.cs.mcgill.ca/~fukuda/soft/polyfaq/polyfaq.html)
- The
  [Polyhedron](http://doc.sagemath.org/html/en/reference/discrete_geometry/sage/geometry/polyhedron/constructor.html) class in [Sage](http://www.sagemath.org/)
- The [StabiliPy](https://github.com/haudren/stabilipy) package provides a more
  general recursive projection method
