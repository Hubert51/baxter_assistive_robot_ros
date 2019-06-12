#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tuesday June 19 16:45:00 2018

@author: Jiawei Xu, Zidi Tao
@description: A Robot Raconteur host for BOW chair wheels.

"""
import sys, argparse
import struct
import time
import thread
import threading
import numpy

import rospy
import roslib
import baxter_interface
roslib.load_manifest('baxter_rr_bridge')
import RobotRaconteur as RR

import roboclaw
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from discrete_pid import PID
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

baxter_servicedef="""
#Service to provide simple interface to BOW wheels
service BOWChair_Interface

option version 0.1

struct speed
	field double[] linear
	field double[] angular
end struct

object BOWbase

function void setSpeed(speed data)
		
end object

"""



class MovingAverage:
    """
    used to filter imu data
    
    """
    def __init__(self):
        # array of readings
        self.readings = [0,0]
        # index of last reading added
        self.index = 0
        
    def get(self):
        """
        gets an averaged reading from the pool of collected data
        """
        return sum(self.readings) / float(len(self.readings))
        
    def add(self, new):
        """
        incorporate a new reading into the moving average
        """
        self.readings[self.index] = new
        self.index = self.index + 1
        if self.index >= len(self.readings):
            self.index = 0

class roboclawController:
    """ 
    used to control the roboclaw
    """
    def __init__(self):
        # Roboclaw parameters
        self.m1Duty = 0 # stop running
        self.m2Duty = 0 # stop running
        # largest absolute value that the motors can be set to
        self.dutyMax = 6000
        
        self.address = 0x80
        roboclawPort = '/dev/ttyACM0'
        roboclaw.Open(roboclawPort,115200)
        self.pub_left = rospy.Publisher('left_voltage_pwm', Int16, queue_size = 1)
        self.pub_right = rospy.Publisher('right_voltage_pwm', Int16, queue_size = 1)

    # setting the PWM of both wheels
    def writeM1M2(self,m1Duty,m2Duty):
        if abs(m1Duty) > self.dutyMax:
            self.m1Duty = self.dutyMax * cmp(m1Duty, 0)
        else:
            self.m1Duty = m1Duty
            
        if abs(m2Duty) > self.dutyMax:
            self.m2Duty = self.dutyMax * cmp(m2Duty, 0)
        else:
            self.m2Duty = m2Duty
        
        # debug:
        # print self.m1Duty,self.m2Duty
        self.pub_left.publish(Int16(self.m1Duty))
        self.pub_right.publish(Int16(self.m2Duty))

        roboclaw.DutyAccelM1(self.address,5000,self.m1Duty)
        roboclaw.DutyAccelM2(self.address,5000,self.m2Duty)

class BOWbase_impl:
    """
    coordinates motor commands with imu odometry to follow velocity commands
    """
    def __init__(self):
        rospy.init_node('BOWbase_controller', anonymous=True)
        # Set PID structures to intial values
        self.rightPID = PID(a=.95, b=2*780, c=-2*480, d=0)
        self.rightPID.setPoint(0)
        self.leftPID = PID(a=.95, b=2*780, c=-2*480, d=0)
        self.leftPID.setPoint(0)

        self.leftFeedforward = 0
        self.rightFeedforward = 0
        
        self.somethingWentWrong = False
        
        # Ensure that data is fresh from both wheels
        self.newDataM1 = False
        self.newDataM2 = False
        self.newData = False
        
        # IMU structures
        self.currentLeftV = MovingAverage()
        self.currentRightV = MovingAverage()
        
        # Roboclaw
        self.myRoboclaw = roboclawController()
        
        # ROS subscribers
        rospy.Subscriber("/cmd_vel",  Twist,  self.setSpeed)
        rospy.Subscriber("/imu1", Imu, self.imu1Callback)
        rospy.Subscriber("/imu2", Imu, self.imu2Callback)
        
        time.sleep(1)
        
        self.lastDataTime = time.time()
        
        self.repeatCount = 0
        self.currentM1Measurement = 0
        self.currentM2Measurement = 0
        self.lastM1Measurement = 0
        self.lastM2Measurement = 0

    def imu1Callback(self,data):
        self.newDataM1 = True
        self.currentM1Measurement = -data.angular_velocity.x
        self.currentLeftV.add(-data.angular_velocity.x)
        
    def imu2Callback(self,data):
        self.newDataM2 = True
        self.currentM2Measurement = -data.angular_velocity.x
        self.currentRightV.add(-data.angular_velocity.x)
                
    def setSpeed(self, data):
        """
        Will set the internal PID controller's setpoints according to the command
        """
        print "****NEW COMMAND RECEIVED***"
        linearV=data.linear[0]
        angularV=data.angular[2]

        L=.5
        R=.15
        self.leftPID.setPoint( (2*linearV - angularV*L)/(2*R))
        self.rightPID.setPoint( (2*linearV + angularV*L)/(2*R))

        if abs(self.leftPID.getPoint()) > 0:
            self.leftFeedforward = self.leftPID.getPoint()/1.0 * 1000 + 3000*self.leftPID.getPoint()/abs(self.leftPID.getPoint())
        
        if abs(self.rightPID.getPoint()) > 0:
            self.rightFeedforward = self.rightPID.getPoint()/1.0 * 1000 + 3000*self.rightPID.getPoint()/abs(self.rightPID.getPoint())

        #self.myRoboclaw.writeM1M2(self.rightFeedforward,self.leftFeedforward)# MAKE SURE THESE ARE IN THE CORRECT ORDER!!!!

        
    def shutdown(self):
        """
        To be called if something went wrong, stop the roboclaw and exit out
        """
        print "shutting down"
        self.myRoboclaw.writeM1M2(0,0)
        exit()
        
    def commandLoop(self):
        """
        Main loop that writes the commands to the roboclaw.  If new data is received
        at a rate lower than 4Hz, then the wheelchair will shutdown
        """
        
        # Thigns to check:
        #  1) We got new data at the correct frequency
        #  2) We are not getting repeat data from our imus (imus are still working)
        
        # run loop 20 times a second
        newLeft = 0
        newRight = 0
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            # Wait until we get new data from both motors
            if self.newDataM1 and self.newDataM2:
                self.newData = True
                
            if not self.newData:
                if time.time() - self.lastDataTime > .5:
                    print "Too long between messages!"
                    self.shutdown()
                else:
                    continue
            
            # We have data, ensure that it isnt repeated data
            else:
                self.lastDataTime = time.time()
                self.newDataM1 = False
                self.newDataM2 = False
                self.newData = False
                
                # Check the number of repeat readings
                if self.currentM1Measurement == self.lastM1Measurement or self.currentM2Measurement == self.lastM2Measurement:
                    self.repeatCount +=1
                
                else:
                    self.repeatCount = 0
                
                # If we have too many repeats, shut the system down
                if self.repeatCount > 6000:
                    print "Too many repeats!"
                    self.shutdown()
                    
                self.lastM1Measurement= self.currentM1Measurement
                self.lastM2Measurement= self.currentM2Measurement           
                
                newLeft = int(self.leftPID.update(self.currentLeftV.get()) + self.leftFeedforward)
                newRight = int(self.rightPID.update(self.currentRightV.get()) + self.rightFeedforward)

                #catch special case of not moving
                if abs(self.leftPID.getPoint()) < 0.002 and abs(self.rightPID.getPoint()) < 0.002:
                    newLeft = 0
                    newRight = 0
                if abs(newLeft) > 0:                
                    print "LEFT setpoint,  measurement, update",'%1.2f' % self.leftPID.getPoint(), '%1.2f' % self.currentLeftV.get(), '%1.2f' % newLeft, '%1.2f' % int(self.leftPID.update(self.currentLeftV.get()))
                    print "RIGHT setpoint, measurement, update",'%1.2f' % self.rightPID.getPoint(), '%1.2f' % self.currentRightV.get(),'%1.2f' % newRight, '%1.2f' % int(self.rightPID.update(self.currentRightV.get()))
                self.myRoboclaw.writeM1M2(newRight,newLeft) # MAKE SURE THESE ARE IN THE CORRECT ORDER!!!!
        
                
            r.sleep()

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(
                            description='Initialize BOW Controller.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on' + \
                           '(will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the Node name
    RR.RobotRaconteurNode.s.NodeName="BOWbaseServer"

    #Initialize object
    base_obj = BOWbase_impl()
    

    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
                         RR.IPNodeDiscoveryFlags_LINK_LOCAL | 
                         RR.IPNodeDiscoveryFlags_SITE_LOCAL)

    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()

    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(baxter_servicedef)
    RR.RobotRaconteurNode.s.RegisterService("BOWbase",
                      "BOWChair_Interface.BOWbase",
                                          base_obj)
    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/BOWbase_controller/BOWbase"

    #raw_input("press enter to quit...\r\n")

    base_obj.commandLoop()

    rospy.spin()

    base_obj.shutdown()
    
    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()
            
if __name__ == '__main__':
    main(sys.argv[1:])
