#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Empty

import sys, argparse
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
import math
import tf
from tf.transformations import euler_from_quaternion
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Twist, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers

wheelchair_servicedef="""
#Service to provide simple interface to Baxter
service Wheelchair_Interface

option version 0.4

object Wheelchair
property double[] current_pose
property uint8 goal_completed
property uint8[] current_marker_state
property uint8 current_marker

event goal_inactive()
event goal_reached()

event ar_marker_0_active()
event ar_marker_1_active()
event ar_marker_2_active()
event ar_marker_3_active()
event ar_marker_4_active()
event ar_marker_0_inactive()
event ar_marker_1_inactive()
event ar_marker_2_inactive()
event ar_marker_3_inactive()
event ar_marker_4_inactive()
 
function void setWVWheelchair(double[] command)
function void go_to_goal(double[] pose)
function void go_to_cancel()
function void calculate_goal(double motion, string direction)
function void cancel_goal()

end object

"""
class Wheelchair_impl(object):
    def __init__(self):
        print "Initializing Node"
        rospy.init_node('wheelchair')
        
        # data initializations
	self._wv_wheelchair = [0]*2;
	self._cmd_vel_msg = Twist()
	self._goal_active = False
	self._goal_completed = False
	self.goal_trans = numpy.zeros(3)
	self.goal_yaw = numpy.zeros(1)
	self._track_goal = False
	self._track_trans = False
	self._track_rot = False
	self._markers = [0]*5
	self._markers_dist = [float("inf")]*5

	# Intialize the goal
        self.goal = MoveBaseGoal()
        
        # initial joint command is current pose
	self.setWVWheelchair(self._wv_wheelchair)

	# Wheelchair cmd_vel publisher to manually control the robot
	self.WVWheelchair_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

	# TF listener to get current robot pose
	self.tf_listener = tf.TransformListener()

	# Hook event goal_inactive
	self.goal_inactive = RR.EventHook()

	# Hook event goal_reached
	self.goal_reached = RR.EventHook()

	# Hook ar_marker active and inactive events
	self.ar_marker_0_active = RR.EventHook()
	self.ar_marker_1_active = RR.EventHook()
	self.ar_marker_2_active = RR.EventHook()
	self.ar_marker_3_active = RR.EventHook()
	self.ar_marker_4_active = RR.EventHook()
	self.ar_marker_0_inactive = RR.EventHook()
	self.ar_marker_1_inactive = RR.EventHook()
	self.ar_marker_2_inactive = RR.EventHook()
	self.ar_marker_3_inactive = RR.EventHook()
	self.ar_marker_4_inactive = RR.EventHook()

	# Subscribe to ar track alvar msgs
	rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.markers_sub)
                
        # Start background threads
        self._running = True
        self._t_command = threading.Thread(target=self.command_worker)
        self._t_command.daemon = True
        self._t_command.start()
        self._g_command = threading.Thread(target=self.track_goal)
        self._g_command.daemon = True
        self._g_command.start()

    def close(self):
        self._running = False
        self._t_command.join()
        self._g_command.join()

    def markers_sub(self, data):
	for i in range(5): # considering only 5 different markers
	    visible = False
	    for j in range(len(data.markers)):
	    	if i == data.markers[j].id:
		    visible = True
		    self._markers_dist[i] = math.hypot(data.markers[j].pose.pose.position.x, data.markers[j].pose.pose.position.y)
		    if self._markers[i] == 0:
		    	self.fire_marker_visible(i)
		    break
	    if not(visible): 
		if self._markers[i] == 1:
		    self._markers_dist[i] = float("inf")
		    self.fire_marker_hidden(i)

    def fire_marker_visible(self, marker_id):
	self._markers[marker_id] = 1
	if marker_id == 0:
	    self.ar_marker_0_active.fire()
	if marker_id == 1:
	    self.ar_marker_1_active.fire()
	if marker_id == 2:
	    self.ar_marker_2_active.fire()
	if marker_id == 3:
	    self.ar_marker_3_active.fire()
	if marker_id == 4:
	    self.ar_marker_4_active.fire()
	rospy.loginfo("AR tag " + str(marker_id) + " visible")

    def fire_marker_hidden(self, marker_id):
	self._markers[marker_id] = 0
	if marker_id == 0:
	    self.ar_marker_0_inactive.fire()
	if marker_id == 1:
	    self.ar_marker_1_inactive.fire()
	if marker_id == 2:
	    self.ar_marker_2_inactive.fire()
	if marker_id == 3:
	    self.ar_marker_3_inactive.fire()
	if marker_id == 4:
	    self.ar_marker_4_inactive.fire()
	rospy.loginfo("AR tag " + str(marker_id) + " hidden")

    @property 
    def current_marker_state(self):
	return self._markers

    @property 
    def current_marker(self):
	return numpy.argmin(self._markers_dist)

    def setWVWheelchair(self, command):
	self._wv_wheelchair = command
	self._cmd_vel_msg.linear.x = command[1]
	self._cmd_vel_msg.linear.y = 0.0
	self._cmd_vel_msg.linear.z = 0.0
	self._cmd_vel_msg.angular.x = 0.0
	self._cmd_vel_msg.angular.y = 0.0
	self._cmd_vel_msg.angular.z = command[0]
    
    # worker function to continuously issue commands to wheelchair
    def command_worker(self):
        while self._running:
            t1 = time.time()
	    if not(self._goal_active):
	    	self.WVWheelchair_pub.publish(self._cmd_vel_msg)
	    else:
        	# Start the robot moving toward the goal
		self.move()
            while (time.time() - t1 < 0.01):
                # idle
                time.sleep(0.001)

    def fire_goal_inactive(self):
	self.goal_inactive.fire()

    def go_to_goal(self, pose):
        # Use the map frame to define goal poses
        self.goal.target_pose.header.frame_id = 'map'
            
        # Set the time stamp to "now"
        self.goal.target_pose.header.stamp = rospy.Time.now()
            
        # Set the goal pose
  	self.goal.target_pose.pose.position = Point(pose[0], pose[1], pose[2]);
  	self.goal.target_pose.pose.orientation = Quaternion(pose[3], pose[4], pose[5], pose[6]);

	self._goal_active = True
	self._goal_completed = False

    def move(self):
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(self.goal)
            
        # Allow 3 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(180)) 
            
        # If we don't get there in time, abort the goal
        if not finished_within_time:
	    self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
		self._goal_completed = True
	    else:
                rospy.loginfo("Failed to reach goal for some reason.")

	self._goal_active = False
	self.fire_goal_inactive()

    def go_to_cancel(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.WVWheelchair_pub.publish(Twist())
        rospy.sleep(1)
	self._goal_active = False

    def fire_goal_reached(self):
	# reached motion goal
	self.goal_reached.fire()

    def cancel_goal(self):
	# cancel motion goal tracking
	self._track_goal = False
	self._track_trans = False
	self._track_rot = False
	rospy.loginfo("Motion goal tracking cancelled")

    def getGoalPositionDifference(self, trans, goal_trans):
	return math.hypot(goal_trans[0]-trans[0], goal_trans[1]-trans[1])

    def getGoalOrientationDifference(self, yaw, goal_yaw):
	return abs(math.atan2(math.sin(yaw - goal_yaw), math.cos(yaw - goal_yaw)))

    def track_goal(self):
        while self._running:
            t1 = time.time()
	
	    # if goal tracking active, fire goal_reached on reaching goal
	    if self._track_goal:
	    	(trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
	    	(roll,pitch,yaw) = euler_from_quaternion(rot)

		# calculate difference from goal position
		if self._track_trans:
	    	    if (self.getGoalPositionDifference(trans, self.goal_trans) <= 0.1): # 1 cm
			self.fire_goal_reached()

		# calculate difference from goal orientation
		if self._track_rot:	
		    if (self.getGoalOrientationDifference(yaw, self.goal_yaw) <= 0.02): # 1 degree
		    	self.fire_goal_reached()

            while (time.time() - t1 < 0.01):
                # idle
                time.sleep(0.001)

    def calculate_goal(self, motion, direction):
	# Obtain current robot transformation wrt odom frame
	(trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
	(roll,pitch,yaw) = euler_from_quaternion(rot)
	self.goal_trans = numpy.zeros(3)
	self.goal_yaw = numpy.zeros(1)

	# calculate desired robot transformation wrt odom frame
	if direction == "forward":
	    self.goal_yaw = yaw
	    self.goal_trans[0] = trans[0] + motion*math.cos(yaw)
	    self.goal_trans[1] = trans[1] + motion*math.sin(yaw)
	    self.goal_trans[2] = trans[2]
	    self._track_trans = True
	    self._track_rot = False
	elif direction == "backward":
	    self.goal_yaw = yaw
	    self.goal_trans[0] = trans[0] - motion*math.cos(yaw)
	    self.goal_trans[1] = trans[1] - motion*math.sin(yaw)
	    self.goal_trans[2] = trans[2]
	    self._track_trans = True
	    self._track_rot = False
	elif direction == "clockwise" or direction == "right":
	    self.goal_yaw = yaw - (float(motion)*math.pi)/180.0
	    self.goal_trans[0] = trans[0]
	    self.goal_trans[1] = trans[1]
	    self.goal_trans[2] = trans[2]
	    self._track_trans = False
	    self._track_rot = True
	else:
	    self.goal_yaw = yaw + (float(motion)*math.pi)/180.0
	    self.goal_trans[0] = trans[0]
	    self.goal_trans[1] = trans[1]
	    self.goal_trans[2] = trans[2]
	    self._track_trans = False
	    self._track_rot = True

	# fix the range of goal_yaw to be in [-pi, pi]
	if self.goal_yaw > math.pi:
	    self.goal_yaw = self.goal_yaw - 2*math.pi
	if self.goal_yaw <= -1*math.pi:
	    self.goal_yaw = self.goal_yaw + 2*math.pi

	# start tracking goal
	self._track_goal = True
	rospy.loginfo("Motion goal tracking active")

    @property
    def current_pose(self):
	(trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
	pose = numpy.zeros(7)
	pose[0:3] = trans
	pose[3:] = rot
	return pose

    @property
    def goal_completed(self):
	if self._goal_completed:
	    return 1
	else:
	    return 0

def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(
                            description='Initialize Wheelchair Controller.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on' + \
                           '(will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the Node name
    RR.RobotRaconteurNode.s.NodeName="WheelchairServer"

    #Initialize object
    wheelchair_obj = Wheelchair_impl()

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
    RR.RobotRaconteurNode.s.RegisterServiceType(wheelchair_servicedef)
    RR.RobotRaconteurNode.s.RegisterService("Wheelchair",
                      "Wheelchair_Interface.Wheelchair",
                                          wheelchair_obj)

    print "Service started, connect via"
    print "tcp://localhost:" + str(port) + "/WheelchairServer/Wheelchair"
    raw_input("press enter to quit...\r\n")
    
    wheelchair_obj.close()
    
    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main(sys.argv[1:])
