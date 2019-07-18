#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from moveit_python import PlanningSceneInterface, MoveGroupInterface
import moveit_commander       
from jointcontroller_host import CollisionObject

# resource 
# https://answers.ros.org/question/11135/plotting-a-markerarray-of-spheres-with-rviz/
# http://wiki.ros.org/rviz/DisplayTypes/Marker


def create_marker(pos):
    marker = Marker()
    marker.header.frame_id = "base"
    marker.type = marker.ARROW
    marker.action = 0
    # Pivot point is around the tip of its tail. Identity orientation points it along the +X axis. 
    # scale.x is the arrow length, 
    # scale.y is the arrow width and 
    # scale.z is the arrow height. 
    marker.scale.x = 0.1
    marker.scale.y = 0.02
    marker.scale.z = 0.01
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    return marker


def create_arrow(pos, ori):
    p = Pose()
    p.position.x = pos[0]
    p.position.y = pos[1]
    p.position.z = pos[2]

    p.orientation.x = ori[0]
    p.orientation.y = ori[1]
    p.orientation.z = ori[2]
    p.orientation.w = ori[3]
    
    return p

# normal publisher
def publisher1():
    # sample:
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = "panda_leftfinger"
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.z = 0.07 # slightly above the end effector
    # box_name = "box"
    # scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))
    pub = rospy.Publisher('Marker', MarkerArray, queue_size=10)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(2) # Hz
    poseArray = PoseArray()        
    poseArray.header.frame_id = 'base'

    # delete all the marker
    markerArray = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "/base"
    marker.action = 3
    markerArray.markers.append(marker)
    pub.publish(markerArray)

    markerArray = MarkerArray()
    while not rospy.is_shutdown():
        # this is Array of arrow
        # p1 = create_arrow([0.6155, -0.4195, -0.2159], [0, 0, 0, 1])
        # p2 = create_arrow([0.6155, -0.4195, 0.2159], [0, 0, 0, 1])
        # poseArray.poses = [p1, p2]

        marker = create_marker([0.6155, -0.4195, -0.2159])
        markerArray.markers.append(marker)
        marker = create_marker([0.6155, -0.4195, 0.2159])
        markerArray.markers.append(marker)
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

        pub.publish(markerArray)
        rate.sleep()
        # print 123

# read from the environment
def publisher2():
    rospy.init_node('test_attach_box', anonymous=True)
    scene = moveit_commander.PlanningSceneInterface()
    for value in scene.get_objects().values(): 
        object1 = CollisionObject(value)
        print object1.bound_pos
    # print objects

if __name__ == '__main__':
    try:
        publisher1()
    except rospy:
        pass
    moveit_commander.os._exit(0)
