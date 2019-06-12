import rospy
from sensor_msgs.msg import JointState
import geometry_msgs.msg

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


def get_current_joint_state():
    joint_states =rospy.wait_for_message("/robot/joint_states", JointState) 
    position = joint_states.position
    joint_name = joint_states.name 
    pos = {}
    for i in range(2,len(position)-1):
        pos[joint_name[i]] = position[i]

    print(pos)


def explore_pose_goal(both_arms):

    eef_link = both_arms.get_end_effector_link()
    print("the eef is ", eef_link)

    pose_goal = geometry_msgs.msg.Pose()
    print("the pose of the arm", pose_goal)