#!/usr/bin/env python
"""
Created on Thu Feb 11 16:58:12 2016

@author: Andrew Cunningham
"""

import sys
import rospy
from baxter_pykdl import baxter_kinematics
import baxter_interface
import numpy as np
import math
import time

from sensor_msgs.msg import Joy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

# generate 3 x 3 rotation matrix for theta degrees about k
def rot(k, theta):
    I = np.identity(3)
    khat = hat(k)
    khat2 = khat.dot(khat)
    return I + math.sin(theta)*khat + (1.0 - math.cos(theta))*khat2
    
# generate rotation matrix for specified X,Y,Z rotations
def genRotationMatrix(x,y,z):
    return rot(np.array([[1],[0],[0]]),x)*rot(np.array([[0],[1],[0]]),y)*rot(np.array([[0],[0],[1]]),z)  
    
# returns jacobian relating angular velocity to quaternion velocity    
def quatjacobian(q):
    I = np.identity(3)
    J = 0.5 * np.concatenate((q[1:4].T, q[0]*I - hat(q[1:4])), axis = 0)
    return J

class xboxControl:
    def __init__(self):
        self.lxAxis = 0
        self.lyAxis = 0
        self.rxAxis = 0
        self.ryAxis = 0
        self.rightTrigger = 0
        self.leftTrigger = 0
        self.aPressed = 0
        self.bPressed = 0
        self.xPressed = 0
        self.yPressed = 0
        self.rightBumperPressed = 0
        self.leftBumperPressed = 0

        rospy.Subscriber("/joy", Joy,  self.controlCall)   
        self.limb = baxter_interface.Limb('right')
        self.kin = baxter_kinematics('right')
        self.gripper = baxter_interface.Gripper('right')
        self.gripper.calibrate()
        
        angles = self.limb.joint_angles()
        

    def controlCall(self,data):
        self.lxAxis = data.axes[0]
        self.lyAxis = data.axes[1]
        self.leftTrigger = data.axes[2]
        self.rxAxis = data.axes[3]
        self.ryAxis = data.axes[4]
        self.rightTrigger = data.axes[5]
        self.aPressed = data.buttons[0]
        self.bPressed = data.buttons[1]
        self.xPressed = data.buttons[2]
        self.yPressed = data.buttons[3]
        self.leftBumperPressed = data.buttons[4]
        self.rightBumperPressed = data.buttons[5]


    def update(self):
        angles = self.limb.joint_angles()
        x = 0
        y = 0
        z = 0
        print genRotationMatrix(0,0,0)
        if abs(self.lyAxis) > .1:
            z = self.lyAxis
        if abs(self.lxAxis) > .1:
            y= self.lxAxis
        if abs(self.ryAxis) > .1:
            x = self.ryAxis
            
        # go to home position
        if self.aPressed:
            angles['right_s0']=0.464412683532309
            angles['right_s1']=-0.5821457090025145
            angles['right_e0']=0.1491796316218565
            angles['right_e1']=1.7211264440074343
            angles['right_w0']=-0.22511168062218448
            angles['right_w1']=-1.1167380135805811
            angles['right_w2']=-0.19941750242510378
            self.limb.move_to_joint_positions(angles)
            return
        
        maxJointVelocity = .2        
        
        jacobian=self.kin.jacobian()
        desiredEndEffectorVelocities = np.array([[x],[y],[z],[0],[0],[0]])
        jacobianInv=self.kin.jacobian_pseudo_inverse()        
              
        jointVelocities = jacobianInv*desiredEndEffectorVelocities
        
        if abs(np.max(jointVelocities)) > maxJointVelocity:
            jointVelocities=jointVelocities/(float(np.max(jointVelocities)/maxJointVelocity))
        
        # be super safe
        jointVelocities=np.clip(jointVelocities,-maxJointVelocity,maxJointVelocity)
        
        angles['right_s0']=jointVelocities[0]
        angles['right_s1']=jointVelocities[1]
        angles['right_e0']=jointVelocities[2]
        angles['right_e1']=jointVelocities[3]
        angles['right_w0']=jointVelocities[4]
        angles['right_w1']=jointVelocities[5]
        angles['right_w2']=jointVelocities[6]
        
        # last minute changes
        if self.xPressed:
            self.gripper.close()
        
        if self.bPressed:
            self.gripper.open()
        
        if self.rightBumperPressed:
            jointVelocities[6]=1
        
        if self.leftBumperPressed:
            jointVelocities[6]=-1
            
        if self.leftTrigger < 0:
            jointVelocities[5]=1
        
        if self.rightTrigger < 0:
            jointVelocities[5]=-1

        self.limb.set_joint_velocities(angles)
        print jointVelocities

def cartesianController():
    rospy.init_node('xboxJacobianControl', anonymous=True)
    myXboxControl = xboxControl()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
       myXboxControl.update()
       rate.sleep()

if __name__ == '__main__':
    cartesianController()