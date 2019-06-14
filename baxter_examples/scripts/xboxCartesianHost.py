#!/usr/bin/env python
"""
Created on Thu Feb 11 16:58:12 2016

@author: Andrew Cunningham
"""

import sys
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

class xboxControl:
    def __init__(self):
        self.lxAxis = 0
        self.lyAxis = 0
        self.rxAxis = 0
        self.ryAxis = 0
        rospy.Subscriber("/joy", Joy,  self.controlCall)
        self.leftArmPositionPub = rospy.Publisher("left_arm_commands", Pose, queue_size=2)
        self.rightArmPositionPub = rospy.Publisher("right_arm_commands", Pose, queue_size=2)
        
        self.leftArmCurrentPose = Pose(
                position=Point(
                    x=0.657579481614,
                    y=0.851981417433,
                    z=0.0388352386502,
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                )
                )
                
        self.rightArmCurrentPose = Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                )
                )

    def controlCall(self,data):
        self.lxAxis = data.axes[0]
        self.lyAxis = data.axes[1]
        self.rxAxis = data.axes[3]
        self.ryAxis = data.axes[4]

    def update(self):
        if abs(self.lyAxis) > .1:
            self.rightArmCurrentPose.position.x+=self.lyAxis/10
        if abs(self.lxAxis) > .1:
            self.rightArmCurrentPose.position.y+=self.lxAxis/10
        self.rightArmPositionPub.publish(self.rightArmCurrentPose)
        
        

def cartesianController():
    global xJoy, yJoy, newData
    rospy.init_node('xboxCartesianControl', anonymous=True)
    myXboxControl = xboxControl()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
       myXboxControl.update()
       rate.sleep()

if __name__ == '__main__':
    cartesianController()