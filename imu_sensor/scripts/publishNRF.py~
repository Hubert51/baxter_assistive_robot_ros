#!/usr/bin/env python
import rospy
import serial
import struct
import time
import std_msgs.msg
import math
from sensor_msgs.msg import Imu 

serial_port_name="/dev/ttyACM0"

class NrfImuInterface(object):
    def __init__(self):
        self._serial=serial.Serial(serial_port_name,baudrate=57600)

    def IMU1_read(self):
        dat=struct.pack("BB",50,0)
        self._serial.write(dat)
        raw = self._serial.read(size = 14)
        unConverted = struct.unpack( "hhhhhhh", raw )
        converted = [unConverted[0]/16384.0, unConverted[1]/16384.0,unConverted[2]/16384.0,unConverted[3]/131.0,unConverted[4]/131.0,unConverted[5]/131.0, unConverted[6]]
        return converted

    def IMU2_read(self):
        dat=struct.pack("BB",50,1)
        self._serial.write(dat)
        raw = self._serial.read(size = 14)
        unConverted = struct.unpack( "hhhhhhh", raw )
        converted = [unConverted[0]/16384.0, unConverted[1]/16384.0,unConverted[2]/16384.0,unConverted[3]/131.0,unConverted[4]/131.0,unConverted[5]/131.0, unConverted[6]]
        return converted
		     
# take an array of data and an imuMsg and fill out the
# imuMsg with the data
def buildImuMsg(data, imuMsg):
    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    imuMsg.header = h
        
    imuMsg.linear_acceleration.x = data[0]
    imuMsg.linear_acceleration.y = data[1]
    imuMsg.linear_acceleration.z = data[2]

    imuMsg.angular_velocity.x = data[3] / 180 * math.pi
    imuMsg.angular_velocity.y = data[4] / 180 * math.pi
    imuMsg.angular_velocity.z = data[5] / 180 * math.pi

# function to be spun, spits out data
def talker():
    myGateway = NrfImuInterface()
    rospy.init_node('nrfImus', anonymous=True)
    imu1Pub = rospy.Publisher('imu1', Imu, queue_size=3)
    imu2Pub = rospy.Publisher('imu2', Imu, queue_size=3)
    rate = rospy.Rate(60) # 60Hz publish rate

    imu1Msg = Imu()
    imu2Msg = Imu()
    while not rospy.is_shutdown():
        # construct messages to be sent
        buildImuMsg(myGateway.IMU1_read(), imu1Msg)
        buildImuMsg(myGateway.IMU2_read(), imu2Msg)
        # flip axis for the other imu to have them match
        #imu2Msg.angular_velocity.z *= -1
        #imu2Msg.linear_acceleration.x *= -1
        # publish messages
        imu1Pub.publish(imu1Msg)
        imu2Pub.publish(imu2Msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
