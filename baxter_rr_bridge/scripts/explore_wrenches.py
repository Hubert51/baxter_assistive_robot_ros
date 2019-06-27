#!/usr/bin/env python
import rospy
import baxter_interface
import time

while True:
    rospy.init_node('wrenches_test')
    right = baxter_interface.Limb('right')
    r_wrench = right.endpoint_effort()
    print r_wrench['force']
    time.sleep(0.8)
