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

import utils
from jointcontroller_host import Baxter_impl




rospy.init_node('test_gripper', anonymous=True)
leftgripper = baxter_interface.Gripper('left')
i = 0
while True:
	leftgripper.command_suction(timeout=100)
	leftgripper.set_vacuum_threshold(100)
	print "good"
	# leftgripper.stop()
	l_vacuum_sensor = baxter_interface.AnalogIO('left_vacuum_sensor_analog')
	print l_vacuum_sensor.state()
	# print leftgripper.type()
	# print leftgripper.blowing()
	# print leftgripper.sucking()
	i += 1
	if i > 5:
		break


time.sleep(30)
print "stop"
leftgripper.stop()

moveit_commander.os._exit(0)

