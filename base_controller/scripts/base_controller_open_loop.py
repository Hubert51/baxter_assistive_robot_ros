#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 21 10:46:10 2015

@author: Andrew Cunningham
@description:  A super simple base "controller" that uses
open loop control to implement linear and angular velocitiy 
commands issued to the base.  
"""

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import time
from discrete_pid import PID
import roboclaw
from std_msgs.msg import Int16

class RoboClaw:
    """ 
    used to control the roboclaw
    """
    def __init__(self):
        # Roboclaw parameters
        self.m1Duty = 0
        self.m2Duty = 0
        #largest absolute value that the motors can be set to
        self.dutyMax = 6000
        
        self.address = 0x80
        roboclawPort = '/dev/ttyACM0'
        roboclaw.Open(roboclawPort,115200)

        
    def writeM1M2(self,m1Duty,m2Duty):
        if abs(m1Duty) > self.dutyMax:
            self.m1Duty = self.dutyMax * cmp(m1Duty, 0)
        else:
            self.m1Duty = m1Duty
            
        if abs(m2Duty) > self.dutyMax:
            self.m2Duty = self.dutyMax * cmp(m2Duty, 0)
        else:
            self.m2Duty = m2Duty
         
        #print self.m1Duty,self.m2Duty
        roboclaw.DutyAccelM1(self.address,5000,self.m1Duty)
        roboclaw.DutyAccelM2(self.address,5000,self.m2Duty)
        
            
class BaseController:
    """
    coordinates motor commands with imu odometry to follow velocity commands
    """
    def __init__(self):
        # Roboclaw
        self.myRoboclaw = RoboClaw()
        
        # ROS subscribers
        rospy.Subscriber("/cmd_vel",  Twist,  self.commandCallback)

        # Desired linear and angular velocities
        self.left_pwm = 0
        self.right_pwm = 0
                
    
    def commandCallback(self, data):
        """
        Will set the PWM's according to command using the fit model
        """
        print "****NEW COMMAND RECEIVED***"
        linearV=data.linear.x
        angularV=data.angular.z

        L=.5
        R=.15
        left_wheel_vd = ( (2*linearV - angularV*L)/(2*R))
        right_wheel_vd = ( (2*linearV + angularV*L)/(2*R))

        if abs(left_wheel_vd) > .05:
            self.left_pwm = int(left_wheel_vd/1.0 * 1000 + 3000*left_wheel_vd/abs(left_wheel_vd))
        else:
            self.left_pwm = 0
        
        if abs(right_wheel_vd) > .05:
            self.right_pwm = int(right_wheel_vd/1.0 * 1000 + 3000*right_wheel_vd/abs(right_wheel_vd))
        else:
            self.right_pwm = 0

        
    def shutdown(self):
        """
        To be called if something went wrong, stop the roboclaw and exit out
        """
        print "shutting down"
        self.myRoboclaw.writeM1M2(0,0)
        exit()
        
    def commandLoop(self):
        """
        Main loop that writes the commands to the roboclaw
        """
        
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.myRoboclaw.writeM1M2(self.right_pwm,self.left_pwm) # MAKE SURE THESE ARE IN THE CORRECT ORDER!!!!
            r.sleep()            

def main():
    rospy.init_node('base_controller', anonymous=True)
    
    myBaseController = BaseController()
    myBaseController.commandLoop()
    rospy.spin()
            
if __name__ == '__main__':
    main()
