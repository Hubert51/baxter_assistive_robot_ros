#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 17 18:55:13 2017

@author: cunnia3
@description: Use artificial potential fields to go to goal with obstacle avoidance
"""

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import tf
from pid import PID
import numpy as np

def deltaAngle(x, y):
    return math.atan2(math.sin(x-y), math.cos(x-y))

class GoToGoal:
    """ Simple go to point controller.  Does nothing for obstacle avoidance"""
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=True)        
        rospy.Subscriber("/odometry", Odometry , self._odomCallback)
        rospy.Subscriber("/goal", PoseStamped, self._goalCallback)
        rospy.Subscriber("/scan", LaserScan, self._scanCallback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.obstacle_register_dist = .6 # distance at which obstacles are recognized
        self.obstacle_stop_dist = .6    # distance at which a hard stop is engaged
        
        self.w_pid = PID(P=2.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=2, Integrator_min=-2)

        self.v = 0
        self.w = 0

        self.v_max = .2
        self.w_max = .2
        
        self.current_x = 0
        self.current_y = 0
        self.current_bearing = 0
        
        self.desired_x = 0
        self.desired_y = 0
        self.desired_bearing = 0    
        
        self.repulsive_point_list = []
        
        ## CONTROL VARIABLES
        self.close_enough = False  # CLOSE ENOUGH TO TARGET, TURN IN PLACE
        self.stop = False          # STOP IF OBSTACLE

    def _goalCallback(self, raw_data):
        print "Got new goal"
        data = raw_data.pose
        self.desired_x = data.position.x
        self.desired_y = data.position.y
        quat = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.desired_bearing = euler[2]

    def _odomCallback(self, data):
        quat = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.current_bearing = euler[2]
        
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y

        if math.sqrt((self.current_x - self.desired_x)**2 + (self.current_y - self.desired_y)**2) > .15:
            self.close_enough = False
        else:
            self.close_enough = True            
        return

    def _scanCallback(self, data):
        self.repulsive_point_list = []
        scan_i=0
        stop = False
        for dist in data.ranges:
            if dist < self.obstacle_stop_dist:
                stop = True                
                
            elif dist > data.range_min and dist < data.range_max and dist < self.obstacle_register_dist:
                total_angle = data.range_min + scan_i * data.angle_increment
                point = np.zeros([2,1])
                point[1] = dist * math.cos(total_angle)
                point[0] = dist * math.sin(total_angle)
                
                # check to make sure this point isnt too close to an existing one
                add = True
                for r_point in self.repulsive_point_list:
                    if deltaAngle(total_angle, math.atan2(r_point[1], r_point[0])) < .05 or abs(np.dot(np.transpose(point), r_point)) < .01:
                        add = False
                        
                if add:
                    self.repulsive_point_list.append(point)
             
            scan_i +=1
        self.stop = stop
        
    def _compute_u(self):
        """ Compute the desired direction to travel in with APF's.  
        The goal exerts an attractive force while the obstacles exert a 
        repulsive force"""
        u = np.zeros([2,1])
        
        ## ADD ATTRACTIVE FORCE
        u[0] = self.desired_x - self.current_x
        u[1] = self.desired_y - self.current_y
        u = u/np.linalg.norm(u)
        
        ## ADD REPULSIVE FORCE
        r_force = np.zeros([2,1])
        if len(self.repulsive_point_list) > 0:
            print len(self.repulsive_point_list)
            for r_point in self.repulsive_point_list:
                r_force[0] -= (r_point[0]- self.current_x)/math.sqrt((r_point[0]- self.current_x)**2 + (r_point[1]- self.current_y)**2)**2
                r_force[1] -= (r_point[1]- self.current_y)/math.sqrt((r_point[0]- self.current_x)**2 + (r_point[1]- self.current_y)**2)**2
            
            r_force = r_force/len(self.repulsive_point_list)
            
        self.u = u + r_force
        print self.u
        return


    def _act(self):
        """ Control linear and angular velocity to acheive u (our desired direction
        to travel in"""
        
        act_bearing = math.atan2(self.u[1], self.u[0]) # where we should head
        self.w_pid.setPoint(act_bearing)
        ## STATE MACHINE
        print len(self.repulsive_point_list)
        self.v = 0
        self.w = 0
        ## STOP
        if self.stop or (self.desired_x == 0 and self.desired_y ==0):
            print "STOPPING"
            self.v = 0
            self.w = 0
        

        ## TURN IN PLACE
        elif self.close_enough and abs(deltaAngle(self.current_bearing, self.desired_bearing)) > .15:
            print "CLOSE ENOUGH, TURNING IN PLACE"
            self.w_pid.setPoint(self.desired_bearing)
            self.w = self.w_pid.update(self.current_bearing)
            self.v = 0


        ## MOVE TO GOAL
        # TURN ONLY
        elif abs(deltaAngle(self.current_bearing, act_bearing)) > .6 and not self.close_enough:
            print "TURNING ONLY"
            self.v = 0
            self.w = self.w_pid.update(self.current_bearing)
            
        # TURN AND GO FORWARD
        elif abs(deltaAngle(self.current_bearing, act_bearing)) < .6 and abs(deltaAngle(self.current_bearing, act_bearing)) > .2 and not self.close_enough:
            print "TURNING AND GOING FORWARD"
            self.v = .2
            self.w = self.w_pid.update(self.current_bearing)
            
        elif not self.close_enough:
            print "GOING FORWARD ONLY"
            self.v = .2
            self.w = 0
        

        if self.v > self.v_max:
            self.v = self.v_max
        if self.v < -self.v_max:
            self.v = -self.v_max

        if self.w > self.w_max:
            self.w = self.w_max
        if self.w < -self.w_max:
           self.w = -self.w_max

        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w
        self.pub.publish(msg)

        return

    def spin(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self._compute_u()
            self._act()
            r.sleep()

go_goal_node = GoToGoal()
go_goal_node.spin()
