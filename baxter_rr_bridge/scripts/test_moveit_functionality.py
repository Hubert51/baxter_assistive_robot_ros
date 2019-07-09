#!/usr/bin/env python
import sys
import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize
import moveit_commander
import copy
joint_state_topic = ['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('foo', anonymous=False)
roscpp_initialize(sys.argv)

both_arms = MoveGroupCommander('both_arms')
right_arm = MoveGroupCommander('right_arm')

pose1 = right_arm.get_current_pose().pose
pose2 = copy.deepcopy(pose1)
pose2.position.z -= 0.1
print pose1
print pose2
waypoints = [pose1, pose2]
(plan, fraction) = right_arm.compute_cartesian_path( waypoints,   # waypoints to follow
                           0.01,        # eef_step
                           0.0) 
print plan
print fraction
right_arm.execute(plan, wait=True)
rospy.sleep(0.5)
raw_input("please input")

# [00000000]
print right_arm.get_joint_value_target()
print both_arms.get_joint_value_target()
# no this functions
# print right_arm.get_named_targets()

print right_arm.get_remembered_joint_values()
print both_arms.get_remembered_joint_values()

print right_arm.get_path_constraints()
print both_arms.get_path_constraints()


print right_arm.get_active_joints()
print both_arms.get_active_joints()

print right_arm.get_current_joint_values()
print right_arm.get_current_pose()
print right_arm.get_current_rpy()
print both_arms.get_current_joint_values()
print both_arms.get_current_pose()
print both_arms.get_current_rpy()


right_arm.clear_pose_targets()

left_current_pose = both_arms.get_current_pose(end_effector_link='left_gripper').pose
right_current_pose = both_arms.get_current_pose(end_effector_link='right_gripper').pose
print left_current_pose