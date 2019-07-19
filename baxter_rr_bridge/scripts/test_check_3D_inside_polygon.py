#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
# from visualization_msgs.msg import Marker
# from visualization_msgs.msg import MarkerArray

from moveit_python import PlanningSceneInterface, MoveGroupInterface
import moveit_commander       
from jointcontroller_host import CollisionObject
import publish_pose
import thread
import threading
import tf


import numpy
import numpy.random
import numpy as np
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


def publish_worker():
    rate = rospy.Rate(2) # Hz
    pub = rospy.Publisher('Marker', MarkerArray, queue_size=1000)
    global running
    # running = True
    while running:
        pub.publish(markerArray)
        rate.sleep()


if __name__ == '__main__':

    rospy.init_node('create_rectangle', anonymous=True)
    # the scene is a dictionary
    scene = moveit_commander.PlanningSceneInterface()
    pub = rospy.Publisher('Marker', MarkerArray, queue_size=1000)
    rate = rospy.Rate(2) # Hz
    # robot = moveit_commander.RobotCommander()

    '''
        generate a box
        not work currently. work later. 
    '''
    # name = "box1"
    # p = PoseStamped()
    # p.header.frame_id = "/base" #robot.get_planning_frame()
    # pos = tf.transformations.random_vector(3)
    # ori = tf.transformations.random_quaternion()
    # dim = tf.transformations.random_vector(3)
    # p.pose.position.x = pos[0]
    # p.pose.position.y = pos[1]
    # p.pose.position.z = pos[2]
    # # p.pose.orientation.w = ori[0]
    # # p.pose.orientation.x = ori[1]
    # # p.pose.orientation.y = ori[2]
    # # p.pose.orientation.z = ori[3]
    # scene.add_box(name, p, dim)
    # rospy.sleep(2)

    object1 = scene.get_objects()["fridge_front"]
    door = CollisionObject(object1)

    # homogeneous transformation matrix from quaternion
    H_matrix = tf.transformations.quaternion_matrix(door.quat)
    vertex = door.vertex_with_orientation
    hull = ConvexHull(vertex, incremental=True)
    # hull_points = hull.points[hull.vertices, :]

    # random generate points
    new_points = np.array(door.dim) * 2 * (numpy.random.rand(1000, 3)-0.5) 
    new_points = np.dot( new_points, numpy.linalg.inv(H_matrix[0:3,0:3])) + door.pos
    in_hull_1 = asarray([pnt_in_cvex_hull_1(hull, pnt) for pnt in new_points], dtype=bool)

    markerArray = MarkerArray()
    markerArray.markers.append(publish_pose.create_marker( door.pos, True))
    for i in range(len(new_points)):
        markerArray.markers.append(publish_pose.create_marker(new_points[i], in_hull_1[i]))
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1
    global running
    running = True

    publish = threading.Thread(target=publish_worker)
    publish.start()


    raw_input("press enter to quit...\r\n")
    running = False 
    publish.join()
    moveit_commander.os._exit(0)

    # Define initial parameters.

