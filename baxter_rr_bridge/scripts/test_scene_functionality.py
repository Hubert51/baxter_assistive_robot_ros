#!/usr/bin/env python
# -*- coding: iso-8859-1 -*-
import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import moveit_commander
import moveit_msgs.msg
import baxter_interface
import geometry_msgs.msg
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import Range
from sensor_msgs.msg import JointState

import rospkg
import geometry_msgs.msg
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)



## @brief teach how to get the information from scenes in the environment
## @param name: name of the scene
def tutorial1():
	# message infomration of a scene
	'''
	header: 
	  seq: 0
	  stamp: 
	    secs: 0
	    nsecs:         0
	  frame_id: /base
	id: fridge_lower
	type: 
	  key: ''
	  db: ''
	primitives: 
	  - 
	    type: 1
	    dimensions: [0.8, 0.48, 0.01]
	primitive_poses: 
	  - 
	    position: 
	      x: 1.3055035396
	      y: 0.0346599057222
	      z: 0.0313475223488
	    orientation: 
	      x: 0.0
	      y: 0.0
	      z: 0.0
	      w: 1.0
	meshes: []
	mesh_poses: []
	planes: []
	plane_poses: []
	operation: 0
	'''
	rospy.init_node('test_attach_box', anonymous=True)
	# the scene is a dictionary
    scene = moveit_commander.PlanningSceneInterface()
    print scene.get_objects().keys()
    for value in scene.get_attached_objects().values(): 
    	print value._type
        print value.__slots__

# from jointcontroller_host import Baxter_impl

# Define initial parameters.
rospy.init_node('test_attach_box', anonymous=True)

# robot = moveit_commander.RobotCommander()

# # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot:

# scene = moveit_commander.PlanningSceneInterface()

# groups = ['both_arms', 'left_arm', 'left_hand', 'right_arm', 'right_hand']

# for my_group in groups:
#     group = moveit_commander.MoveGroupCommander(my_group)

#     # We can get the name of the reference frame for this robot:
#     planning_frame = group.get_planning_frame()
#     print "============ Reference frame: %s" % planning_frame

#     # We can also print the name of the end-effector link for this group:
#     eef_link = group.get_end_effector_link()
#     print "============ End effector: %s" % eef_link

#     # We can get a list of all the groups in the robot:
#     group_names = robot.get_group_names()
#     print "============ Robot Groups:", robot.get_group_names()

#     # Sometimes for debugging it is useful to print the entire state of the
#     # robot:
#     print "============ Printing robot state"
#     print group.get_current_state()
#     print ""

# moveit_commander.os._exit(0)

scene = moveit_commander.PlanningSceneInterface()
print scene.get_objects()
# print scene

# robot = moveit_commander.RobotCommander()
# group = moveit_commander.MoveGroupCommander("left_hand")

# rospy.sleep(2)

# # print group.get_end_effector_link()

# p = PoseStamped()
# p.header.frame_id = robot.get_planning_frame()
# p.pose.position.x = 0.
# p.pose.position.y = 0.
# p.pose.position.z = 0.
# scene.add_box("table", p, (0.5, 1.5, 0.6))

# my_group = "left_arm"
# group = moveit_commander.MoveGroupCommander(my_group)

# p = moveit_commander.PlanningSceneInterface()
# # # robot = moveit_commander.RobotCommander()
# rospy.sleep(2)
# # print robot.get_planning_frame()

# box_pose = geometry_msgs.msg.PoseStamped()
# box_pose.header.frame_id = "base"
# box_pose.pose.position.x = 0.2
# box_pose.pose.position.y = 0.4
# box_pose.pose.position.z = 0.6
# box_pose.pose.orientation.w = 1.0
# box_name = "boxbox1"
# p.add_box(box_name, box_pose, size=(0.5, 0.5, 0.5))
# rospy.sleep(4)
# # # 
# # p.attach_box("base", "box")
# # lists = p.get_objects()
# # print lists

# # p.remove_world_object("boxbox1")

# # # p.remove_attached_object("base")
# rospy.sleep(2)

# moveit_commander.os._exit(0)

# p = PlanningSceneInterface("right_gripper")
# rospy.sleep(2)

# # link = "right_w2"
# # name = "front_door_fridge"

# # print p.getKnownAttachedObjects()

# # p.waitForSync()

# # p.robot_state.is_diff = True

# # p.removeAttachedObject("microwave_door1")
# # p.waitForSync()

# # p.clear()
# # p = PlanningSceneInterface("right_w2")
# # p.addCube("my_cube", 0.1, 1, 0, 0.5)

# # p.attachBox("microwave_door1", 0.01,0.34, 0.34, 0.6014, 0.13, 0.33, "right_gripper")
# # rospy.sleep(2)


# # do something

# # remove the cube
# print p._attached_objects
# p.removeAttachedObject("microwave_door1")
# # p.waitForSync()
# rospy.sleep(2)

# p.attachBox("front_door_fridge", 0.01, 0.48, 0.255, 0.8965, 0.0142, 0.1836, "right_w2")
# p.attach_box("base", name)
# time.sleep(3)
moveit_commander.os._exit(0)



########################################################################################

######## successfully add the box into the world

# p = moveit_commander.PlanningSceneInterface()
# robot = moveit_commander.RobotCommander()
# rospy.sleep(2)

# # group = moveit_commander.MoveGroupCommander("left_arm")

# # eef_link = group.get_end_effector_link()

# # print eef_link
# print robot.get_planning_frame()

# box_pose = geometry_msgs.msg.PoseStamped()
# box_pose.header.frame_id = robot.get_planning_frame()
# box_pose.pose.orientation.w = 1.0
# box_name = "box"
# # p.add_box(box_name, box_pose, size=(1, 1, 1))


# p.attach_box("left_gripper", "box")
# lists = p.get_objects()
# print lists

# moveit_commander.os._exit(0)

###################################################################################