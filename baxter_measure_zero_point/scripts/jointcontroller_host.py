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

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import moveit_commander


baxter_servicedef="""
#Service to provide simple interface to Baxter
service BaxterJoint_Interface

option version 0.4

object Baxter
property double[] joint_positions
property double[] joint_velocities
property double[] joint_torques
property double[] endeffector_positions
property double[] endeffector_orientations
property double[] endeffector_twists
property double[] endeffector_wrenches

function void setControlMode(uint8 mode)

# the following two functions have same functionality, the moveitSetJointCommand
# will consider the collision between arms themselves and environment obstacles
function void setJointCommand(string limb, double[] command)
function void moveitSetJointCommand(string limb, double[] command)

function void setPositionModeSpeed(double speed)
function double[] solveIKfast(double[] positions, double[] quaternions, string limb_choice)
end object

"""


class Baxter_impl(object):
    def __init__(self):
        print "Initializing Node"
        # rospy.init_node('baxter_jointstates')
        
        print "Enabling Robot"
        rs = baxter_interface.RobotEnable()
        rs.enable()
        
        
        self._valid_limb_names = {'left': 'left', 
                                    'l': 'left', 
                                    'right': 'right',
                                    'r': 'right'}
        
        # get information from the SDK
        self._left = baxter_interface.Limb('left')
        self._right = baxter_interface.Limb('right')
        self._l_jnames = self._left.joint_names()
        self._r_jnames = self._right.joint_names()
        
        # data initializations
        self._jointpos = [0]*14
        self._jointvel = [0]*14
        self._jointtor = [0]*14
        self._ee_pos = [0]*6
        self._ee_or = [0]*8
        self._ee_tw = [0]*12
        self._ee_wr = [0]*12
        self._l_joint_command = dict(zip(self._l_jnames,[0.0]*7))
        self._r_joint_command = dict(zip(self._r_jnames,[0.0]*7))
        self.MODE_POSITION = 0;
        self.MODE_VELOCITY = 1;
        self.MODE_TORQUE = 2;
        self._mode = self.MODE_POSITION
        
        # initial joint command is current pose
        self.readJointPositions()
        self.setJointCommand('left',self._jointpos[0:7])
        self.setJointCommand('right',self._jointpos[7:14])

        # initialize the moveit object
        self.both_arms = moveit_commander.MoveGroupCommander('both_arms')

                
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

    def close(self):
        self._running = False
        self._t_joints.join()
        self._t_effector.join()
        self._t_command.join()
        
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

        return resp.joints[0].position



    def setJointCommand(self, limb, command):
        limb = limb.lower()
        if not limb in self._valid_limb_names.keys():
            return
        
        if self._valid_limb_names[limb] == 'left':
            for i in xrange(0,len(self._l_jnames)):
                self._l_joint_command[self._l_jnames[i]] = command[i]
        elif self._valid_limb_names[limb] == 'right':
            for i in xrange(0,len(self._r_jnames)):
                self._r_joint_command[self._r_jnames[i]] = command[i]

    def moveitSetJointCommand(self, limb, command):
        limb = limb.lower()
        if limb == "botharm":
            print command
            # self.both_arms.set_joint_value_target(init_pos)
            # both_arms.plan()
            # both_arms.go(wait=True)
    
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

    def hello(self):
        print("hello world!")

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

if __name__ == '__main__':
    main(sys.argv[1:])
