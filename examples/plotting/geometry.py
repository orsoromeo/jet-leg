from jet_leg.computational_geometry.computational_geometry import ComputationalGeometry
import numpy as np

IP_points = np.array([[1,1], [5,5], [3,1], [1,3]])
print IP_points, np.shape(IP_points)
point2check = [4,4]
compGeom = ComputationalGeometry()
facets = compGeom.compute_halfspaces_convex_hull(IP_points)
print "facets", facets

isPointFeasible = compGeom.isPointRedundant(facets, point2check)