#!/usr/bin/env python
import rospy
import math
import serial
import time
import sys
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def controlCall(data):
    global xJoy, yJoy, newData
    xJoy = data.axes[0]
    yJoy = data.axes[1]
    newData = True

class xbox_base_controller():
    def __init__(self):
        rospy.init_node('xbox_control', anonymous=True)

        self.x_joy = 0
        self.y_joy = 0

        self.pub = rospy.Publisher('/cmd_vel', Twist)
        rospy.Subscriber('/xbox_joy', Joy, self.control_callback)

        self.last_command = time.time() # send 0 command if its 
                                        # been too long
    
    def control_callback(self, data):
        self.last_command = time.time()
        self.x_joy = data.axes[0]
        self.y_joy = data.axes[1]

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            cmd = Twist()
            dt = time.time() - self.last_command
            if dt < 4 and (abs(self.x_joy) > .2 or abs(self.y_joy) > .2):
                cmd.linear.x = self.y_joy
                cmd.angular.z = self.x_joy
            else:
                cmd.linear.x = 0
                cmd.angular.z = 0

            self.pub.publish(cmd)    
            r.sleep()
        
if __name__ == '__main__':
    my_node = xbox_base_controller()
    my_node.spin()
