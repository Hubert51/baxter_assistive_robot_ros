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

from jointcontroller_host import Baxter_impl

# Define initial parameters.
rospy.init_node('pnp', anonymous=True)

p = PlanningSceneInterface("base")
p.removeCollisionObject("test_scene")
# p.addBox("microwave_door", 0.0.01, 0.34, 0.6014, 0.13, 0.33)
p.addBox("test_scene", 0.4, 0.43, 0.02, 1.2, 0.13, 0.34)
p.waitForSync()

moveit_commander.os._exit(0)
