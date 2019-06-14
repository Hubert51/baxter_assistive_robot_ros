#!/usr/bin/env python

import roslib
roslib.load_manifest('baxter_rr_bridge')
import rospy
import sys, argparse
import RobotRaconteur as RR
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()

baxter_servicedef="""
#service to connect to camera
service camera_interface

function img get_image
"""
def get_image ():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/cameras/left_hand_camera/image"
    # Set up your subscriber and define its callback
    image = rospy.Subscriber(image_topic, Image, image_callback)
    return image

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('camera_image.jpeg', cv2_img)
    return cv2_img

def main(argv):
   
    # parse command line arguments
    parser = argparse.ArgumentParser(
                            description='Initialize Joint Controller.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on' + \
                           '(will auto-generate if not specified)')
    args = parser.parse_args(argv)
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
    RR.RobotRaconteurNode.s.RegisterService("Baxter",
                      "BaxterJoint_Interface.Baxter",
                                          baxter_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/BaxterJointServer/Baxter"
    raw_input("press enter to quit...\r\n")
    
    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])