#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from moveit_python import PlanningSceneInterface, MoveGroupInterface
import moveit_commander       
from jointcontroller_host import CollisionObject

import numpy
import numpy.random
from numpy import zeros, ones, arange, asarray, concatenate
from scipy.optimize import linprog

from scipy.spatial import ConvexHull

def pnt_in_cvex_hull_1(hull, pnt):
    '''
    Checks if `pnt` is inside the convex hull.
    `hull` -- a QHull ConvexHull object
    `pnt` -- point array of shape (3,)
    '''
    new_hull = ConvexHull(concatenate((hull.points, [pnt])))
    if numpy.array_equal(new_hull.vertices, hull.vertices): 
        return True
    return False


def pnt_in_cvex_hull_2(hull_points, pnt):
    '''
    Given a set of points that defines a convex hull, uses simplex LP to determine
    whether point lies within hull.
    `hull_points` -- (N, 3) array of points defining the hull
    `pnt` -- point array of shape (3,)
    '''
    N = hull_points.shape[0]
    c = ones(N)
    A_eq = concatenate((hull_points, ones((N,1))), 1).T   # rows are x, y, z, 1
    b_eq = concatenate((pnt, (1,)))
    result = linprog(c, A_eq=A_eq, b_eq=b_eq)
    if result.success and c.dot(result.x) == 1.:
        return True
    return False


if __name__ == '__main__':

    rospy.init_node('check_3D_point', anonymous=True)
    # the scene is a dictionary
    scene = moveit_commander.PlanningSceneInterface()
    print scene.get_objects().keys()
    # for value in scene.get_attached_objects().values(): 
    #     print value._type
    #     print value.__slots__

    object1 = scene.get_objects()["fridge_front"]
    door = CollisionObject(object1)
    print door.ori
    moveit_commander.os._exit(0)

    # Define initial parameters.

    points = numpy.random.rand(8, 3)
    hull = ConvexHull(points, incremental=True)
    hull_points = hull.points[hull.vertices, :]
    new_points = 1. * numpy.random.rand(1000, 3)
    print new_points