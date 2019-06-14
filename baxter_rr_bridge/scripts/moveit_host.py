#!/usr/bin/env python
import roslib
roslib.load_manifest('baxter_rr_bridge')
import rospy
import baxter_interface
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import time

import sys, argparse
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
import traceback
import cv2
import cv2.aruco as aruco

import moveit_commander
import moveit_msgs.msg
import baxter_interface
import geometry_msgs.msg
from moveit_python import PlanningSceneInterface, MoveGroupInterface



baxter_servicedef="""
#Service to provide simple interface to Baxter
service BaxterMoveit_interface

option version 0.4

struct BaxterImage
    field int32 width
    field int32 height
    field int32 step
    field uint8[] data
end struct

struct CameraIntrinsics
    field double[] K
    field double[] D
end struct

struct ImageHeader
    field int32 width
    field int32 height
    field int32 step
end struct

struct ARtagInfo
    field double[] tmats
    field int32[] ids
end struct

object BaxterMoveit

    property uint8 camera_open

    # camera control functions
    function void openCamera()
    function void closeCamera()
    function void setExposure(int16 exposure)
    function void setGain(int16 gain)
    function void setWhiteBalance(int16 red, int16 green, int16 blue)
    function void setFPS(double fps)
    function void setCameraIntrinsics(CameraIntrinsics data)
    function void setMarkerSize(double markerSize)
    
    # functions to acquire data on the image
    function BaxterImage getCurrentImage()
    function ImageHeader getImageHeader()
    function CameraIntrinsics getCameraIntrinsics()
    function double getMarkerSize()
    function ARtagInfo ARtag_Detection()
    
    # pipe to stream images through
    pipe BaxterImage ImageStream
    
end object

"""


def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(description='Initialize Baxter moveit module.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on (will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the RobotRaconteur Node name
    RR.RobotRaconteurNode.s.NodeName="BaxterMoveitServer"

    #Create transport, register it, and start the server
    print "Registering Transport"
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
        RR.IPNodeDiscoveryFlags_LINK_LOCAL | RR.IPNodeDiscoveryFlags_SITE_LOCAL)
    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(args.port)
    port = args.port
    if (port == 0):
        port = t.GetListenPort()
    
    #Register the service type and the service
    print "Starting Service"
    RR.RobotRaconteurNode.s.RegisterServiceType(baxter_servicedef)
    
    #Initialize object
    baxter_obj = moveit_commander.MoveGroupCommander('both_arms')
    
    RR.RobotRaconteurNode.s.RegisterService("moveit_host", 
				"BaxterMoveit_interface.BaxterMoveit", baxter_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/BaxterMoveitServer" 
    raw_input("press enter to quit...\r\n")

    # baxter_obj.closeCamera()

    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])