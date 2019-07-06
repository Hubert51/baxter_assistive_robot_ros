#!/usr/bin/env python
import roslib
roslib.load_manifest('baxter_rr_bridge')
import rospy
import baxter_interface
from std_msgs.msg import Empty

import sys, argparse
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
import geometry_msgs

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from sensor_msgs.msg import Range
from sensor_msgs.msg import JointState


from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import moveit_commander
# moveit_commander.roscpp_initialize(sys.argv)
import baxter_interface
from moveit_python import PlanningSceneInterface, MoveGroupInterface

from operator import add

# both_arms = moveit_commander.MoveGroupCommander('both_arms')
# right_arm = moveit_commander.MoveGroupCommander('right_arm')
# left_arm = moveit_commander.MoveGroupCommander('left_arm')



baxter_servicedef="""
#Service to provide simple interface to Baxter
service BaxterJoint_Interface

option version 0.4

struct JointValue
    field string{list} name
    field double[] values
end struct

struct Pose
    field double[] pos
    field double[] ori
end struct

object Baxter



property double[] joint_positions
property double[] joint_velocities
property double[] joint_torques
property string{list} joint_names
property double[] IR_values
property double[] endeffector_positions
property double[] endeffector_orientations
property double[] endeffector_twists
property double[] endeffector_wrenches

function double[] getForces(string limb)
function double[] getForcesDot(string limb)
function double[] getPositions(string limb)
function double[] getOrientations(string limb)
function double[] getJointPositions(string limb)
function double[] getJointVelocities(string limb)


function void setControlMode(uint8 mode)

# the following two functions have same functionality, the moveitSetJointCommand
# will consider the collision between arms themselves and environment obstacles
function void setJointCommand(string limb, double[] command)
function void moveitSetJointCommand(string limb, double[] command)
function void moveitSetJointCommand2(string limb, double[] command)
# function void set_joint_value_target(double[] command)
function void setPoseTarget(double[] pos, double[] ori)
function void go()
# function double[] moveCartesianPaths(string limb, double[] value)
function void moveCartesianPaths(string limb, Pose[] waypoint)

function void plan(string limb, double[] command)

function void setMaxVelocityScalingFactor(double value)
function void moveitStop(string limb)
function double[] getJointValueTarget(string limb)
function void clearPathConstraints()
function void clearTrajectoryConstraints()

function void setPlanningTime(int32 seconds)
function void setNumPlanningAttempts(int32 attempts)

function void remember_joint_values(string name)
function string{list} remembered()
function void forget_joint_values(string name)
# function  get_named_target_values(self, target):


function void addBox(string name, double[] dim, double[] pos)
function void removeScene(string name)
function void attachBox(string jointName, string boxName)
function void removeAttachedObject(string link, string name=Null)

function void setPositionModeSpeed(double speed)
function double[] solveIKfast(double[] positions, double[] quaternions, string limb_choice)

function void testFunction()
end object

"""


class Baxter_impl(object):
    def __init__(self):
        print "Initializing Node"
        rospy.init_node('baxter_jointstates')
        moveit_commander.roscpp_initialize(sys.argv)

        print "Enabling Robot"
        rs = baxter_interface.RobotEnable()
        rs.enable()
        
        
        self._valid_limb_names = {'left': 'left', 
                                    'l': 'left', 
                                    'right': 'right',
                                    'r': 'right',
                                    'b': 'both',
                                    'both': 'both'}
        
        # get information from the SDK
        self._left = baxter_interface.Limb('left')
        self._right = baxter_interface.Limb('right')
        self._l_jnames = self._left.joint_names()
        self._r_jnames = self._right.joint_names()
        
        # data initializations
        self._jointpos = [0]*14
        self._jointvel = [0]*14
        self._jointtor = [0]*14
        self._jointname = self._l_jnames + self._r_jnames
        self._ee_pos = [0]*6
        self._ee_or = [0]*8
        self._ee_tw = [0]*12
        self._ee_wr = [0]*12
        self._ee_wr_dot = [0]*12
        self._l_joint_command = dict(zip(self._l_jnames,[0.0]*7))
        self._r_joint_command = dict(zip(self._r_jnames,[0.0]*7))
        self.MODE_POSITION = 0;
        self.MODE_VELOCITY = 1;
        self.MODE_TORQUE = 2;
        self._mode = self.MODE_POSITION

        # IR_values
        self._IR_values = [0] * 2

        # initialize the moveit object
        self.both_arms = moveit_commander.MoveGroupCommander('both_arms')
        self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
        self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
        rospy.sleep(2)

        # initialize move_group parameter
        self.setMaxVelocityScalingFactor(0.6)
        self.setPlanningTime(10)
        self.setNumPlanningAttempts(100)

        # initial joint command is current pose
        self.readJointPositions()
        self.setJointCommand('left',self._jointpos[0:7])
        self.setJointCommand('right',self._jointpos[7:14])

        # # Initialize the planning scene interface. from moveit_python package
        self.p = PlanningSceneInterface("base")
        # Allow replanning to increase the odds of a solution.
        self.right_arm.allow_replanning(True)
        self.left_arm.allow_replanning(True)
        # Set the arms reference frames.
        self.right_arm.set_pose_reference_frame('base')
        self.left_arm.set_pose_reference_frame('base')
        # Create baxter_interface limb instance.
        self.leftarm = baxter_interface.limb.Limb('left')
        self.rightarm = baxter_interface.limb.Limb('right')

        # # Create baxter_interface gripper instance.
        self.leftgripper = baxter_interface.Gripper('left')
        self.rightgripper = baxter_interface.Gripper('right')

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        rospy.sleep(2)

                
        # Start background threads
        self._running = True
        self._t_joints = threading.Thread(target=self.jointspace_worker)
        self._t_joints.daemon = True
        self._t_joints.start()
        
        self._t_effector = threading.Thread(target=self.endeffector_worker)
        self._t_effector.daemon = True
        self._t_effector.start()
        
        self._t_command = threading.Thread(target=self.command_worker)
        self._t_command.daemon = True
        self._t_command.start()

        self._t_IR_values = threading.Thread(target=self.IR_values_worker)
        self._t_IR_values.daemon = True
        self._t_IR_values.start()

    def close(self):
        self._running = False
        self._t_joints.join()
        self._t_effector.join()
        self._t_command.join()
        self._t_IR_values.join()
        
        if (self._mode != self.MODE_POSITION):
            self._left.exit_control_mode()
            self._right.exit_control_mode()

    @property	
    def joint_positions(self):
        return self._jointpos
    
    @property	
    def joint_velocities(self):
        return self._jointvel

    @property	
    def joint_torques(self):
        return self._jointtor

    @property   
    def joint_names(self):
        return self._jointname

    @property
    def IR_values(self):
        return self.readIRValues()
    
    @property 
    def endeffector_positions(self):
        return self._ee_pos
    
    @property 
    def endeffector_orientations(self):
        return self._ee_or
    
    @property 
    def endeffector_twists(self):
        return self._ee_tw
    
    @property 
    def endeffector_wrenches(self):
        return self._ee_wr

    def getForces(self, limb):
        if self._valid_limb_names[limb] == 'left':
            return self._ee_wr[3:6]

        elif self._valid_limb_names[limb] == 'right':
            return self._ee_wr[9:12]

        elif self._valid_limb_names[limb] == 'both':
            return self._ee_wr[3:6] + self._ee_wr[9:12]

    def getForcesDot(self, limb):
        if self._valid_limb_names[limb] == 'left':
            return self._ee_wr_dot[3:6]

        elif self._valid_limb_names[limb] == 'right':
            return self._ee_wr_dot[9:12]

        elif self._valid_limb_names[limb] == 'both':
            return self._ee_wr_dot[3:6] + self._ee_wr_dot[9:12]

    def getPositions(self, limb):
        if self._valid_limb_names[limb] == 'left':
            return self._ee_pos[0:3]

        elif self._valid_limb_names[limb] == 'right':
            return self._ee_pos[3:6]

        elif self._valid_limb_names[limb] == 'both':
            return self._ee_pos

    def getOrientations(self, limb):
        if self._valid_limb_names[limb] == 'left':
            return self._ee_or[0:4]

        elif self._valid_limb_names[limb] == 'right':
            return self._ee_or[4:8]

        elif self._valid_limb_names[limb] == 'both':
            return self._ee_or

    def getJointPositions(self, limb):
        if self._valid_limb_names[limb] == 'left':
            return self._jointpos[0:7]

        elif self._valid_limb_names[limb] == 'right':
            return self._jointpos[7:14]

        elif self._valid_limb_names[limb] == 'both':
            return self._jointpos

    def getJointVelocities(self, limb):
        if self._valid_limb_names[limb] == 'left':
            return self._jointvel[0:7]

        elif self._valid_limb_names[limb] == 'right':
            return self._jointvel[7:14]

        elif self._valid_limb_names[limb] == 'both':
            return self._jointpos

    
    def readJointPositions(self):
        l_angles = self._left.joint_angles()
        r_angles = self._right.joint_angles()
        if l_angles:
            for i in xrange(0,len(self._l_jnames)):
                self._jointpos[i] = l_angles[self._l_jnames[i]]
        if r_angles:
            for i in xrange(0,len(self._r_jnames)):
                self._jointpos[i+7] = r_angles[self._r_jnames[i]]
            
    def readJointVelocities(self):
        l_velocities = self._left.joint_velocities()
        r_velocities = self._right.joint_velocities()
        if l_velocities:
            for i in xrange(0,len(self._l_jnames)):
                self._jointvel[i] = l_velocities[self._l_jnames[i]]
        if r_velocities:
            for i in xrange(0,len(self._r_jnames)):
                self._jointvel[i+7] = r_velocities[self._r_jnames[i]]
            
    def readJointTorques(self):
        l_efforts = self._left.joint_efforts()
        r_efforts = self._right.joint_efforts()
        if l_efforts:
            for i in xrange(0,len(self._l_jnames)):
                self._jointtor[i] = l_efforts[self._l_jnames[i]]
        if r_efforts:
            for i in xrange(0,len(self._r_jnames)):
                self._jointtor[i+7] = r_efforts[self._r_jnames[i]]
    
    def readEndEffectorPoses(self):
        l_pose = self._left.endpoint_pose()
        if l_pose:
            self._ee_pos[0] = l_pose['position'].x
            self._ee_pos[1] = l_pose['position'].y
            self._ee_pos[2] = l_pose['position'].z
            self._ee_or[0] = l_pose['orientation'].w
            self._ee_or[1] = l_pose['orientation'].x
            self._ee_or[2] = l_pose['orientation'].y
            self._ee_or[3] = l_pose['orientation'].z
        r_pose = self._right.endpoint_pose()
        if r_pose:
            self._ee_pos[3] = r_pose['position'].x
            self._ee_pos[4] = r_pose['position'].y
            self._ee_pos[5] = r_pose['position'].z
            self._ee_or[4] = r_pose['orientation'].w
            self._ee_or[5] = r_pose['orientation'].x
            self._ee_or[6] = r_pose['orientation'].y
            self._ee_or[7] = r_pose['orientation'].z
        
    def readEndEffectorTwists(self):
        l_twist = self._left.endpoint_velocity()
        if l_twist:
            self._ee_tw[0] = l_twist['angular'].x
            self._ee_tw[1] = l_twist['angular'].y
            self._ee_tw[2] = l_twist['angular'].z
            self._ee_tw[3] = l_twist['linear'].x
            self._ee_tw[4] = l_twist['linear'].y
            self._ee_tw[5] = l_twist['linear'].z
        r_twist = self._right.endpoint_velocity()
        if r_twist:
            self._ee_tw[6] = r_twist['angular'].x
            self._ee_tw[7] = r_twist['angular'].y
            self._ee_tw[8] = r_twist['angular'].z
            self._ee_tw[9] = r_twist['linear'].x
            self._ee_tw[10] = r_twist['linear'].y
            self._ee_tw[11] = r_twist['linear'].z
    def readEndEffectorWrenches(self):
        old_ee_wr = self._ee_wr[:]
        l_wrench = self._left.endpoint_effort()
        if l_wrench:
            self._ee_wr[0] = l_wrench['torque'].x
            self._ee_wr[1] = l_wrench['torque'].y
            self._ee_wr[2] = l_wrench['torque'].z
            self._ee_wr[3] = l_wrench['force'].x
            self._ee_wr[4] = l_wrench['force'].y
            self._ee_wr[5] = l_wrench['force'].z
        r_wrench = self._right.endpoint_effort()
        if r_wrench:
            self._ee_wr[6] = r_wrench['torque'].x
            self._ee_wr[7] = r_wrench['torque'].y
            self._ee_wr[8] = r_wrench['torque'].z
            self._ee_wr[9] = r_wrench['force'].x
            self._ee_wr[10] = r_wrench['force'].y
            self._ee_wr[11] = r_wrench['force'].z
        self._ee_wr_dot = [a_i - b_i for a_i, b_i in zip(self._ee_wr, old_ee_wr)]

    def readIRValues(self):
        return [
                rospy.wait_for_message("/robot/range/left_hand_range/state", Range).range,
                rospy.wait_for_message("/robot/range/right_hand_range/state", Range).range
            ]
    
    
    def setControlMode(self, mode):
        if mode != self.MODE_POSITION and \
                mode != self.MODE_VELOCITY and \
                mode != self.MODE_TORQUE:
            return
        if mode == self.MODE_POSITION:
            self._left.exit_control_mode()
            self._right.exit_control_mode()
            # set command to current joint positions
            self.setJointCommand('left',self._jointpos[0:7])
            self.setJointCommand('right',self._jointpos[7:14])
        elif mode == self.MODE_VELOCITY:
            # set command to zeros
            self.setJointCommand('left',[0]*7)
            self.setJointCommand('right',[0]*7)
        elif mode == self.MODE_TORQUE:
            # set command to zeros
            self.setJointCommand('left',[0]*7)
            self.setJointCommand('right',[0]*7)
        
        self._mode = mode

    # This function calls RSDK ikFast Service
    def solveIKfast(self, positions, quaternions, limb_choice):
        ns = "ExternalTools/" + limb_choice + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {}
        if (limb_choice == 'left' or limb_choice == 'l'):
            limb_choice = 'left'
            poses = {
                'left': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position = Point(
                            x = positions[0],
                            y = positions[1],
                            z = positions[2],
                        ),
                        orientation = Quaternion(
                            x = quaternions[1],
                            y = quaternions[2],
                            z = quaternions[3],
                            w = quaternions[0],
                        ),
                    ),
                ),
                'right': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position = Point(
                            x = self._ee_pos[3],
                            y = self._ee_pos[4],
                            z = self._ee_pos[5],
                        ),
                        orientation = Quaternion(
                            x = self._ee_or[5],
                            y = self._ee_or[6],
                            z = self._ee_or[7],
                            w = self._ee_or[4],
                        ),
                    ),
                ),
            }
        elif (limb_choice == 'right' or limb_choice == 'r'):
            limb_choice = 'right'
            poses = {
                'left': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position = Point(
                            x = self._ee_pos[0],
                            y = self._ee_pos[1],
                            z = self._ee_pos[2],
                        ),
                        orientation = Quaternion(
                            x = self._ee_or[1],
                            y = self._ee_or[2],
                            z = self._ee_or[3],
                            w = self._ee_or[0],
                        ),
                    ),
                ),
                'right': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position = Point(
                            x = positions[0],
                            y = positions[1],
                            z = positions[2],
                        ),
                        orientation = Quaternion(
                            x = quaternions[1],
                            y = quaternions[2],
                            z = quaternions[3],
                            w = quaternions[0],
                        ),
                    ),
                ),
            }
        else:
            print "Not a valid arm"
            return 

        # begin the solvinng process
        ikreq.pose_stamp.append(poses[limb_choice])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)

        seed_dict = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }

        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = seed_dict.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  (seed_str,))

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print "\nIK Joint Solution:\n", limb_joints
            print "------------------"
            print "Response Message:\n", resp
        # if no valid solution was found
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
        # self.inner_test_function(resp.joints)
        return resp.joints[0].position


    # def moveCartesianPaths(self, limb, value):
    #     if self._valid_limb_names[limb] == 'left':
    #         pos = list( map(add, self._ee_pos[0:3], value) )
    #         ori = self._ee_or[0:4]
    #         joint_angle = self.solveIKfast(pos, ori, 'left')

    #     elif self._valid_limb_names[limb] == 'right':
    #         pos = list( map(add, self._ee_pos[3:], value) )
    #         ori = self._ee_or[4:]
    #         joint_angle = self.solveIKfast(pos, ori, 'right')
    #     return joint_angle

    def moveCartesianPaths(self, limb, value):
        print value
        (plan, fraction) = self.right_arm.compute_cartesian_path( value,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0) 
        print plan
        self.right_arm.execute(plan, wait=True)
        # if self._valid_limb_names[limb] == 'left':
        #     pos = list( map(add, self._ee_pos[0:3], value) )
        #     ori = self._ee_or[0:4]
        #     joint_angle = self.solveIKfast(pos, ori, 'left')

        # elif self._valid_limb_names[limb] == 'right':
        #     pos = list( map(add, self._ee_pos[3:], value) )
        #     ori = self._ee_or[4:]
        #     joint_angle = self.solveIKfast(pos, ori, 'right')
        # return joint_angle



    def inner_test_function(self, joints):
        print "total number of solution is ", len(joints)
        for joint in joints:
            print joint


    def setJointCommand(self, limb, command):
        limb = limb.lower()
        if not limb in self._valid_limb_names.keys():
            return
        
        target_joint = {}

        if self._valid_limb_names[limb] == 'left':
            for i in xrange(0,len(self._l_jnames)):
                self._l_joint_command[self._l_jnames[i]] = command[i]
                target_joint[self._l_jnames[i]] = 0.0
            self.left_arm.set_joint_value_target(target_joint)

        elif self._valid_limb_names[limb] == 'right':
            for i in xrange(0,len(self._r_jnames)):
                self._r_joint_command[self._r_jnames[i]] = command[i]
                target_joint[self._r_jnames[i]] = 0.0
            self.right_arm.set_joint_value_target(target_joint)
     
        # to reset the both arm
        target_joint = {}
        for i in xrange(0,len(self._l_jnames)):
            target_joint[self._l_jnames[i]] = 0.0

        for i in xrange(0,len(self._r_jnames)):
            # self._r_joint_command[self._r_jnames[i]] = command[i]
            target_joint[self._r_jnames[i]] = 0.0
        self.both_arms.set_joint_value_target(target_joint)

    def moveitSetJointCommand2(self, limb, command):
        self.both_arms.set_joint_value_target(None)
        self.left_arm.set_joint_value_target(None)
        self.right_arm.set_joint_value_target(None)

        """
       Specify a target joint configuration for the group.
       - if the type of arg1 is one of the following: dict, list, JointState message, then no other arguments should be provided.
       The dict should specify pairs of joint variable names and their target values, the list should specify all the variable values
       for the group. The JointState message specifies the positions of some single-dof joints.
       - if the type of arg1 is string, then arg2 is expected to be defined and be either a real value or a list of real values. This is
       interpreted as setting a particular joint to a particular value.
       - if the type of arg1 is Pose or PoseStamped, both arg2 and arg3 could be defined. If arg2 or arg3 are defined, their types must
       be either string or bool. The string type argument is interpreted as the end-effector the pose is specified for (default is to use
       the default end-effector), and the bool is used to decide whether the pose specified is approximate (default is false). This situation
       allows setting the joint target of the group by calling IK. This does not send a pose to the planner and the planner will do no IK.
       Instead, one IK solution will be computed first, and that will be sent to the planner.
       """
    def set_joint_value_target(self, command):
        pass
            


    def moveitSetJointCommand(self, limb, command):
        limb = limb.lower()
        pos_name = ['left_w0', 'left_w1', 'left_w2', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'right_e0', 'right_e1']
        init_pos = {'left_w0': -2.223505152039907, 'left_w1': 0.01802427425765361, 'left_w2': -1.190369091399081, 'right_s0': 0.4663301595171658, 'right_s1': 1.0365875174135684, 'right_w0': 2.1828546611609436, 'right_w1': 1.6616846884768743, 'right_w2': -2.28102943158561, 'left_e0': 0.027611654181937447, 'left_e1': 1.5627429276582652, 'left_s0': 0.858262250821889, 'left_s1': -0.04486893804564835, 'right_e0': 1.6110633224766557, 'right_e1': 2.249582825433959}
        
        # print type(command) == str 
        print command == []
        print command == None
        if command == []:
            self.both_arms.set_joint_value_target(None)
            self.left_arm.set_joint_value_target(None)
            self.right_arm.set_joint_value_target(None)
            return

        target_joint = {}
        if limb == "both_arm" or self._valid_limb_names[limb] == 'both':
            for i in range(len(pos_name)):
                target_joint[pos_name[i]] = command[i]
            self.both_arms.set_joint_value_target(target_joint)
            self.both_arms.plan()
            self.both_arms.go(wait=True)
            # self.both_arms.stop()
        elif self._valid_limb_names[limb] == 'left':
            for i in xrange(0,len(self._l_jnames)):
                target_joint[self._l_jnames[i]] = command[i]
            self.left_arm.set_joint_value_target(target_joint)
            self.left_arm.plan()
            self.left_arm.go(wait=True)
            # self.left_arm.stop()
        elif self._valid_limb_names[limb] == 'right':
            for i in xrange(0,len(self._r_jnames)):
                target_joint[self._r_jnames[i]] = command[i]
            self.right_arm.set_joint_value_target(target_joint)
            self.right_arm.plan()
            self.right_arm.go(wait=True)
            # self.right_arm.stop()
        rospy.sleep(2)
        # self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
        # self.both_arms = moveit_commander.MoveGroupCommander('both_arms')

    def plan(self, limb, command):
        limb = limb.lower()
        pos_name = ['left_w0', 'left_w1', 'left_w2', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'right_e0', 'right_e1']
        init_pos = {'left_w0': -2.223505152039907, 'left_w1': 0.01802427425765361, 'left_w2': -1.190369091399081, 'right_s0': 0.4663301595171658, 'right_s1': 1.0365875174135684, 'right_w0': 2.1828546611609436, 'right_w1': 1.6616846884768743, 'right_w2': -2.28102943158561, 'left_e0': 0.027611654181937447, 'left_e1': 1.5627429276582652, 'left_s0': 0.858262250821889, 'left_s1': -0.04486893804564835, 'right_e0': 1.6110633224766557, 'right_e1': 2.249582825433959}
        
        # print type(command) == str 
        print command == []
        print command == None
        if command == []:
            self.both_arms.set_joint_value_target(None)
            self.left_arm.set_joint_value_target(None)
            self.right_arm.set_joint_value_target(None)
            return

        target_joint = {}
        if self._valid_limb_names[limb] == 'left':
            for i in xrange(0,len(self._l_jnames)):
                target_joint[self._l_jnames[i]] = command[i]
            self.left_arm.set_joint_value_target(target_joint)
            plan = self.left_arm.plan()
            # self.left_arm.stop()
        elif self._valid_limb_names[limb] == 'right':
            for i in xrange(0,len(self._r_jnames)):
                target_joint[self._r_jnames[i]] = command[i]
            self.right_arm.set_joint_value_target(target_joint)
            plan = self.right_arm.plan()

        print 'the plan is '
        print plan
        for item in plan:
            slef.setJointCommand(limb, item.positions)
            rospy.sleep(0.3)

    def setPoseTarget(self, pos, ori):
        # print self._jointpos

        # print self.left_arm.get_current_joint_values()
        # print self.right_arm.get_current_joint_values()
        # print self.both_arms.get_current_joint_values()
        # print left_arm.get_current_joint_values()
        # print right_arm.get_current_joint_values()
        # print both_arms.get_current_joint_values()
        end_effector = self.left_arm.get_end_effector_link()
        wpose = self.left_arm.get_current_pose(end_effector).pose
        print wpose
        # print self.left_arm.get_current_pose()
        # print self.both_arms.get_current_pose()

        # # print self.both_arms.get_current_pose()
        # print self.left_arm.get_current_rpy()
        # print self.both_arms.get_current_rpy()
        # # print self.left_arm.get_current_state()
        # print self.robot.get_current_state()
        # print self.both_arm.get_current_state()



        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pos[0]
        pose_goal.position.y = pos[1]
        pose_goal.position.z = pos[2]
        pose_goal.orientation.w = ori[0]
        pose_goal.orientation.x = ori[1]
        pose_goal.orientation.y = ori[2]
        pose_goal.orientation.z = ori[3]
        self.right_arm.set_pose_target(pose_goal)

        plan = self.right_arm.go(wait=True)
        print plan

        self.left_arm.set_pose_target(pose_goal)


    def go(self):
        self.left_arm.go(wait=True)


    ## @brief Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1]. 
    def setMaxVelocityScalingFactor(self, value):
        self.both_arms.set_max_velocity_scaling_factor(value)
        self.left_arm.set_max_velocity_scaling_factor(value)
        self.right_arm.set_max_velocity_scaling_factor(value)
        self.both_arms.set_max_acceleration_scaling_factor(value)
        self.left_arm.set_max_acceleration_scaling_factor(value)
        self.right_arm.set_max_acceleration_scaling_factor(value)


    def setPlanningTime(self, seconds):
        self.both_arms.set_planning_time(seconds)
        self.left_arm.set_planning_time(seconds)
        self.right_arm.set_planning_time(seconds)

    def setNumPlanningAttempts(self, attempts):
        self.both_arms.set_num_planning_attempts(attempts)
        self.left_arm.set_num_planning_attempts(attempts)
        self.right_arm.set_num_planning_attempts(attempts)

    def moveitStop(self, limb):
        limb = limb.lower()
        if limb == "":
            self.both_arms.stop()
            self.left_arm.stop()
            self.right_arm.stop()

        elif self._valid_limb_names[limb] == 'left':
            self.left_arm.stop()

        elif self._valid_limb_names[limb] == 'right':
            self.right_arm.stop() 

        elif self._valid_limb_names[limb] == 'both':
            self.both_arms.stop() 

    def getJointValueTarget(self, limb):
        limb = limb.lower()
        if self._valid_limb_names[limb] == 'left':
            return self.left_arm.get_joint_value_target()

        elif self._valid_limb_names[limb] == 'right':
            return self.right_arm.get_joint_value_target()

        elif self._valid_limb_names[limb] == 'both':
            return self.both_arms.get_joint_value_target()

    def clearPathConstraints(self):
        self.left_arm.clear_path_constraints()
        self.both_arms.clear_path_constraints()


    def clearTrajectoryConstraints(self):
        self.left_arm.clear_trajectory_constraints()


    def remember_joint_values(self, name):
        pass


    def remembered(self):
        return_list = self.left_arm.get_remembered_joint_values().keys() + self.both_arms.get_remembered_joint_values().keys()
        return self.left_arm.get_remembered_joint_values().keys()


    def forget_joint_values(self, name):
        self.left_arm.forget_joint_values(name)


    ## @brief if two scenes share same name, the second one will replace the first one
    ## @param name: name of the scene
    ## @param dim: one by three array, dimension of the scene
    ## @param pos: one by three array, position of the scene
    def addBox(self, name, dim, pos):
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]
        self.scene.add_box(name, p, dim)


    ## @param name: the name of the scene will be removed
    def removeScene(self, name):
        self.p.removeCollisionObject(name)


    ## @brief attach the box onto one specific joint
    ## @param name: name of the scene
    ## @param dim: one by three array, dimension of the scene
    ## @param pos: one by three array, position of the scene
    def attachBox(self, jointName, boxName):
        self.scene.attach_box(jointName, boxName)

    def removeAttachedObject(self, link, name=None):
        if name == '':
            name = None
        self.scene.remove_attached_object(link, name)


    def testFunction(self):
        right_ir_sensor =rospy.wait_for_message("/robot/range/left_hand_range/state", Range) 
        distance=right_ir_sensor.range
        print "-----> The distance to table:", distance,"m"
        print right_ir_sensor
        # print self.rightgripper.get_current_pose().pose
    

    def setPositionModeSpeed(self, speed):
        if speed < 0.0:
            speed = 0.0
        elif speed > 1.0:
            speed = 1.0
        
        self._left.set_joint_position_speed(speed)
        self._right.set_joint_position_speed(speed)
        
    
    # worker function to request and update joint data for baxter
    # maintain 100 Hz read rate
    # TODO: INCORPORATE USER-DEFINED JOINT PUBLISH RATE
    def jointspace_worker(self):
        while self._running:
            t1 = time.time()
            self.readJointPositions()
            self.readJointVelocities()
            self.readJointTorques()
            while (time.time() - t1 < 0.01):
                # idle
                time.sleep(0.001)
            
    
    # worker function to request and update end effector data for baxter
    # Try to maintain 100 Hz operation
    def endeffector_worker(self):
        while self._running:
            t1 = time.time()
            self.readEndEffectorPoses()
            self.readEndEffectorTwists()
            self.readEndEffectorWrenches()
            while (time.time() - t1 < 0.01):
                # idle
                time.sleep(0.001)
            
    # worker function to continuously issue commands to baxter
    # Try to maintain 100 Hz operation
    # TODO: INCLUDE CLOCK JITTER CORRECTION
    def command_worker(self):
        while self._running:
            t1 = time.time()
            
            if (self._mode == self.MODE_POSITION):
                self._left.set_joint_positions(self._l_joint_command)
                self._right.set_joint_positions(self._r_joint_command)
            elif (self._mode == self.MODE_VELOCITY):
                self._left.set_joint_velocities(self._l_joint_command)
                self._right.set_joint_velocities(self._r_joint_command)
            elif (self._mode == self.MODE_TORQUE):
                #self._supp_cuff_int_pubs['left'].publish()
                #self._supp_cuff_int_pubs['right'].publish()
                self._left.set_joint_torques(self._l_joint_command)
                self._right.set_joint_torques(self._r_joint_command)
            while (time.time() - t1 < 0.01):
                # idle
                time.sleep(0.001)


    def IR_values_worker(self):
        while self._running:
            t1 = time.time()
            self._IR_values = [
                rospy.wait_for_message("/robot/range/left_hand_range/state", Range).range,
                rospy.wait_for_message("/robot/range/right_hand_range/state", Range).range
            ]
            while (time.time() - t1 < 0.01):
                # idle
                time.sleep(0.001)


def main(argv):
    # parse command line arguments
    parser = argparse.ArgumentParser(
                            description='Initialize Joint Controller.')
    parser.add_argument('--port', type=int, default = 0,
                    help='TCP port to host service on' + \
                           '(will auto-generate if not specified)')
    args = parser.parse_args(argv)

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the Node name
    RR.RobotRaconteurNode.s.NodeName="BaxterJointServer"

    #Initialize object
    baxter_obj = Baxter_impl()

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
    
    baxter_obj.close()
    
    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()
    moveit_commander.os._exit(0)

    # time.sleep(5)


if __name__ == '__main__':
    # Only need two arguments
    main(sys.argv[1:3])
