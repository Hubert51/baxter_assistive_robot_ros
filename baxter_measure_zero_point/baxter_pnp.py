#!/usr/bin/env python
# -*- coding: iso-8859-1 -*-
import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import moveit_commander
import moveit_msgs.msg
import baxter_interface
import geometry_msgs.msg
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import Range
from sensor_msgs.msg import JointState

import rospkg
import geometry_msgs.msg
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import utils
from jointcontroller_host import Baxter_impl

# Define initial parameters.
rospy.init_node('pnp', anonymous=True)
# Initialize the move_group API.
moveit_commander.roscpp_initialize(sys.argv)
# Connect the arms to the move group.
both_arms = moveit_commander.MoveGroupCommander('both_arms')
right_arm = moveit_commander.MoveGroupCommander('right_arm')
left_arm = moveit_commander.MoveGroupCommander('left_arm')
# Allow replanning to increase the odds of a solution.
right_arm.allow_replanning(True)
left_arm.allow_replanning(True)
# Set the arms reference frames.
right_arm.set_pose_reference_frame('base')
left_arm.set_pose_reference_frame('base')
# Create baxter_interface limb instance.
leftarm = baxter_interface.limb.Limb('left')
rightarm = baxter_interface.limb.Limb('right')
# Initialize the planning scene interface.
p = PlanningSceneInterface("base")
# Create baxter_interface gripper instance.
leftgripper = baxter_interface.Gripper('left')
rightgripper = baxter_interface.Gripper('right')
leftgripper.calibrate()
rightgripper.calibrate()
leftgripper.open()
rightgripper.open()

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

rospy.sleep(2)

# box_pose = geometry_msgs.msg.PoseStamped()
# box_pose.header.frame_id = robot.get_planning_frame()
# box_pose.pose.position.x = 1.
# box_pose.pose.position.y = 0.5
# box_pose.pose.position.z = 0.5
# scene.add_box("table", box_pose, (0.5, 1.5, 0.6))

# name size and position
p.addBox("first part", 0.8, 0.43, 0.02, 1.23, 0, 0.34)
p.addBox("under part", 0.8, 0.43, 0.04, 1.23, 0, 0.0)

p.addBox("second part", 0.8, 0.01, 1.6, 1.23, -0.22, 0)
p.addBox("third part", 0.8, 0.01, 1.6, 1.23, 0.22, 0)

p.waitForSync()

def load_gazebo_models(table_pose=Pose(position=Point(x=0.7, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose1=Pose(position=Point(x=0.6725, y=0.0765, z=-0.135)),
                       block_pose2=Pose(position=Point(x=0.55, y=-0.2, z=-0.135)),
                       block_pose3=Pose(position=Point(x=0.7, y=-0.1, z=-0.135)),
                       block_pose4=Pose(position=Point(x=0.58, y=-0.03, z=-0.135)),
                       block_reference_frame="base"):
    pass
    # Get Models' Path.
    model_path = rospkg.RosPack().get_path('baxter_pnp_one_arm_cartesian_sim')+"/models/"
    # Load Table SDF.
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block1 URDF.
    block_xml1 = ''
    with open (model_path + "block/model1.urdf", "r") as block_file:
        block_xml1=block_file.read().replace('\n', '')
    # Load Block2 URDF.
    block_xml2 = ''
    with open (model_path + "block/model2.urdf", "r") as block_file:
        block_xml2=block_file.read().replace('\n', '')
    # Load Block3 URDF.
    block_xml3 = ''
    with open (model_path + "block/model3.urdf", "r") as block_file:
        block_xml3=block_file.read().replace('\n', '')
    # Load Block4 URDF.
    block_xml4 = ''
    with open (model_path + "block/model4.urdf", "r") as block_file:
        block_xml4=block_file.read().replace('\n', '')
    # Spawn Table SDF.
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block1 URDF.
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block1", block_xml1, "/",
                               block_pose1, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    # Spawn Block2 URDF.
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block2", block_xml2, "/",
                               block_pose2, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    # Spawn Block3 URDF.
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block3", block_xml3, "/",
                               block_pose3, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    # Spawn Block4 URDF.
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block4", block_xml4, "/",
                               block_pose4, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))


def set_current_position(arm, *arg):
    # Function to add the current position as the first point for a movement.
    if(arm=='left'):
        current_position=left_arm.get_current_pose()
    if(arm=='right'):
        current_position=right_arm.get_current_pose()

    current_pos = geometry_msgs.msg.Pose()
    current_pos.position.x = current_position.pose.position.x
    current_pos.position.y = current_position.pose.position.y
    current_pos.position.z = current_position.pose.position.z
    current_pos.orientation.x = current_position.pose.orientation.x
    current_pos.orientation.y = current_position.pose.orientation.y
    current_pos.orientation.z = current_position.pose.orientation.z
    current_pos.orientation.w = current_position.pose.orientation.w
    i=len(arg)
    if(i==1):
	waypoints=arg[0]
        waypoints.append(current_pos)
    return current_pos

def move(arm, *arg):
    # The cartesian path will be interpolated at a resolution of 0.1 cm
    # which is why the eef_step in cartesian translation is specify as 0.001. 
    # The jump threshold is specify as 0.0, effectively disabled.
    # This function is limited to 3 points but more can be added.
    fraction = 0
    attempts=0
    state=0
    waypoints = []
    set_current_position(arm, waypoints)
    # i is the number of waypoints.
    i=len(arg)
    waypoints.append(arg[0])
    # "goal" is the endposition of the movement, if there are more points then it will contain the last one.
    goal=arg[0]
    goal_x=goal.position.x
    goal_y=goal.position.y
    goal_z=goal.position.z
    if(i>1):
        goal=arg[1]
        goal_x=goal.position.x
        goal_y=goal.position.y
        goal_z=goal.position.z
        waypoints.append(arg[1])
    if(i>2):
        goal=arg[2]
        goal_x=goal.position.x
        goal_y=goal.position.y
        goal_z=goal.position.z
        waypoints.append(arg[2])

    if(arm=='right'):
        right_arm.set_start_state_to_current_state()
        # This function computes a cartesian path for the waypoints. It calculates points with a
        # maximum step size of 1 mm between the waypoints. It return the plan and the fraction
        # which says how good it followed the requested trajectory.
        # (example: fraction= 0.95.454545 -> followed 95.454545% of requested trajectory)
        (plan, fraction) = right_arm.compute_cartesian_path (waypoints, 0.001, 0.0, True)
        right_arm.execute(plan, wait=True) 
        # Read the position of the right arm to compare it with the goal.
        a=right_arm.get_current_pose()
	x_pos= a.pose.position.x
	y_pos= a.pose.position.y
	z_pos= a.pose.position.z
        # Waiting up to 3 seconds that the goal position is reached. (If it fail state=1)
	# It is also required to check that the movement is finished because it continues directly
        # after the command right_arm.execute() with the next code lines.
        while not((abs(z_pos-goal_z)< 0.01) and (abs(y_pos-goal_y)< 0.01) and (abs(x_pos-goal_x)< 0.01)):
            a=right_arm.get_current_pose()
	    x_pos= a.pose.position.x
	    y_pos= a.pose.position.y
	    z_pos= a.pose.position.z        
            time.sleep(0.5)
            if(attempts>6):
                print("----->cartesian path failed!<-----")
                state=1
            attempts +=1
        time.sleep(1)
        return state       

    if(arm=='left'):
        left_arm.set_start_state_to_current_state()
        (plan, fraction) = left_arm.compute_cartesian_path (waypoints, 0.001, 0.0, True)
        left_arm.execute(plan, wait=True)
        # Read the position of the left arm to compare it with the goal.
        a=left_arm.get_current_pose()
	x_pos= a.pose.position.x
	y_pos= a.pose.position.y
	z_pos= a.pose.position.z

        while not((abs(z_pos-goal_z)< 0.01) and (abs(y_pos-goal_y)< 0.01) and (abs(x_pos-goal_x)< 0.01)):
            a=left_arm.get_current_pose()
	    x_pos= a.pose.position.x
	    y_pos= a.pose.position.y
	    z_pos= a.pose.position.z
            time.sleep(0.5)
            if(attempts>6):
                print("----->cartesian path failed!<-----") 
                state=1      
            attempts +=1
        time.sleep(1)
        return state       

def measure_zero_point():

    moveit_commander.os._exit(0)

    
    # This function find the height from the ground to the zero point in MoveIt with the base frame. (in my case 0.903 m)
    # It is necessary to know the height of the table or the object which stands in front of the robot.
    # Add the real table size in z direction.
    table_size_z = 0
    # Define positions.
    pos1 = {'left_e0': -1.69483279891317, 'left_e1':  1.8669726956453, 'left_s0': 0.472137005716569, 'left_s1': -0.38852045702393034, 'left_w0': -1.9770933862776057, 'left_w1': -1.5701993084642143, 'left_w2': -0.6339059781326424, 'right_e0': 1.7238109084167481, 'right_e1': 1.7169079948791506, 'right_s0': 0.36930587426147465, 'right_s1': -0.33249033539428713, 'right_w0': -1.2160632682067871, 'right_w1': 1.668587600115967, 'right_w2': -1.810097327636719}
    lpos1 = {'left_e0': -1.69483279891317, 'left_e1':  1.8669726956453, 'left_s0': 0.472137005716569, 'left_s1': -0.38852045702393034, 'left_w0': -1.9770933862776057, 'left_w1': -1.5701993084642143, 'left_w2': -0.6339059781326424}
    rpos1 = {'right_e0': 1.7238109084167481, 'right_e1': 1.7169079948791506, 'right_s0': 0.36930587426147465, 'right_s1': -0.33249033539428713, 'right_w0': -1.2160632682067871, 'right_w1': 1.668587600115967, 'right_w2': -1.810097327636719}

    m_z_start = geometry_msgs.msg.Pose()
    m_z_start.position.x = 0.55
    m_z_start.position.y = 0
    m_z_start.position.z = 0.0
    m_z_start.orientation.x = 1.0
    m_z_start.orientation.y = 0.0
    m_z_start.orientation.z = 0.0
    m_z_start.orientation.w = 0.0

 
    both_arms.set_joint_value_target(pos1)
    both_arms.plan()
    both_arms.go(wait=True)

    print " after move "    
    # cProfile to measure the performance (time) of the task.
    pr = cProfile.Profile()
    pr.enable()

    '''
    print "In123In "
    # add by Ruijie to attach the object
    # load_gazebo_models()
    # sys.stdout.flush()
    print "dead1"
    # Remove models from the scene on shutdown.
    rospy.on_shutdown(delete_gazebo_models)
    print "dead2"
    # Wait for the All Clear from emulator startup.
    rospy.wait_for_message("/robot/sim/started", Empty)
    print "dead3"
    # Clear planning scene.
    p.clear()
    # Add table as attached object.
    p.attachBox('table', table_size_x, table_size_y, table_size_z, center_x, center_y, center_z, 'base', touch_links=['pedestal'])
    print "dead4"
    p.waitForSync()
    sys.stdout.flush()

    
    except:
        print "dead5"
        moveit_commander.os._exit(0)
    print "sleep"
    '''

    # Start to measure the height of the table.
    state=move("right", m_z_start)
    right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range)
    # At first move with 10 cm steps and if the range is smaller then 25 cm with 1 cm steps.
    while(right_ir_sensor.range> 0.25):
        if not state:
            m_z_start.position.z -=0.1
        state=move("right", m_z_start)
        right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
        print "-----> The distance to table:",right_ir_sensor.range,"m"
    while(right_ir_sensor.range> 0.12):
        if not state:
            m_z_start.position.z -=0.01
        state=move("right", m_z_start)
        right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
        print "-----> The distance to table:",right_ir_sensor.range,"m"
    time.sleep(3)
    right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
    distance=right_ir_sensor.range
    print "-----> The distance to table:", distance,"m"
    a=right_arm.get_current_pose()
    z_pos= a.pose.position.z
    # The z position of the tip of the gripper and the distance between
    # the tip and the table must be added.
    # (zpos is negative for table < 90 cm)
    # The value 0.099 is the distance from the ir sensor to the gripper tip.
    offset_zero_point= table_size_z-z_pos+(distance-0.099)
    print "-----> The distance from the ground to the zero point is:", offset_zero_point," m"
    both_arms.set_joint_value_target(pos1)
    both_arms.plan()
    both_arms.go(wait=True)
    time.sleep(100)
    moveit_commander.roscpp_shutdown()
    # Exit MoveIt.
    moveit_commander.os._exit(0)


def move_arm():

    # explore the API
    # utils.explore_pose_goal(both_arms)

    #######
    # the functionality in jointcontroller_host
    # robotArm = Baxter_impl() 
    # print robotArm.joint_positions()
    ########

    # rospy.Subscriber("chatter", String, callback)
    # joint_state =rospy.wait_for_message("/robot/joint_states", JointState) 
    # micro_front_door3 = {'left_w0': -1.4733885467639398, 'left_w1': -1.570029336400721, 'left_w2': 1.3433836749906511, 'right_s0': 0.45405831321408247, 'right_s1': -0.7765777738669907, 'right_w0': 0.5767767762449155, 'right_w1': -1.2540292940963258, 'right_w2': 0.3466796580621035, 'left_e0': -1.7161410064468066, 'left_e1': 2.5322187856018465, 'left_s0': 1.5523885573400387, 'left_s1': 0.033364082136507746, 'right_e0': 1.2505778373235836, 'right_e1': 1.8952332634324287}

    # both_arms.set_joint_value_target(micro_front_door3)
    # both_arms.plan()
    # both_arms.go(wait=True)
    # raw_input("plz")
    utils.get_current_joint_state()
    #print(joint_state)
    
    moveit_commander.os._exit(0)


    # one set of workable path
    # init_pos = {'left_w0': -2.223505152039907, 'left_w1': 0.01802427425765361, 'left_w2': -1.190369091399081, 'right_s0': 0.4663301595171658, 'right_s1': 1.0365875174135684, 'right_w0': 2.1828546611609436, 'right_w1': 1.6616846884768743, 'right_w2': -2.28102943158561, 'left_e0': 0.027611654181937447, 'left_e1': 1.5627429276582652, 'left_s0': 0.858262250821889, 'left_s1': -0.04486893804564835, 'right_e0': 1.6110633224766557, 'right_e1': 2.249582825433959}
    # fridge_pos = {'left_w0': -2.222354666448993, 'left_w1': 0.01802427425765361, 'left_w2': -1.190369091399081, 'right_s0': 0.9664078963678107, 'right_s1': 0.40382044241083515, 'right_w0': 2.277577974812868, 'right_w1': 0.6143593055481082, 'right_w2': -1.3395487230209375, 'left_e0': 0.027228158984966094, 'left_e1': 1.5600584612794657, 'left_s0': 0.858262250821889, 'left_s1': -0.04410194765170564, 'right_e0': 2.0470973614330847, 'right_e1': 0.3834951969713534}
    # microwave = {'left_w0': -2.222354666448993, 'left_w1': 0.01802427425765361, 'left_w2': -1.1896021010051383, 'right_s0': 0.7033301912454623, 'right_s1': 0.49125734732030374, 'right_w0': 2.790694548360539, 'right_w1': 1.1635244276110863, 'right_w2': -2.3834226491769614, 'left_e0': 0.026461168591023387, 'left_e1': 1.5627429276582652, 'left_s0': 0.8578787556249177, 'left_s1': -0.04601942363656241, 'right_e0': 2.3431556534949696, 'right_e1': 1.504068162521648}
    
    # matlab workable path
    init_pos = {'left_w0': -1.472238061173026, 'left_w1': -1.5707963267946636, 'left_w2': 1.3430001797936797, 'right_s0': 0.4973932704718454, 'right_s1': 0.019941750242510378, 'right_w0': 2.9575149590430776, 'right_w1': 1.9209274416295095, 'right_w2': -2.656087734223594, 'left_e0': -1.716524501643778, 'left_e1': 2.5322187856018465, 'left_s0': 1.5527720525370101, 'left_s1': 0.03413107253045045, 'right_e0': 1.0074418824437454, 'right_e1': 2.1360682471304386}
    fridge_pos = {'left_w0': -1.472238061173026, 'left_w1': -1.5707963267946636, 'left_w2': 1.3430001797936797, 'right_s0': 0.6250971710633061, 'right_s1': -0.0395000052880494, 'right_w0': 3.046869339937403, 'right_w1': 1.151252581308003, 'right_w2': -2.6488013254811382, 'left_e0': -1.7161410064468066, 'left_e1': 2.5318352904048753, 'left_s0': 1.5527720525370101, 'left_s1': 0.0337475773334791, 'right_e0': 1.0193302335498575, 'right_e1': 1.2870098810358621}
    microwave = {'left_w0': -1.4733885467639398, 'left_w1': -1.5692623460067783, 'left_w2': 1.3433836749906511, 'right_s0': 0.7056311624272903, 'right_s1': -0.5867476513661708, 'right_w0': 0.6040049352298816, 'right_w1': -0.6768690226544388, 'right_w2': 0.1721893434401377, 'left_e0': -1.7157575112498353, 'left_e1': 2.531451795207904, 'left_s0': 1.551621566946096, 'left_s1': 0.033364082136507746, 'right_e0': 0.8950777897311389, 'right_e1': 1.112136071216925}
    micro_front_door = {'left_w0': -1.4726215563699971, 'left_w1': -1.571179821991635, 'left_w2': 1.3430001797936797, 'right_s0': 0.400368985638093, 'right_s1': -0.06557767868210143, 'right_w0': 3.0591411862404865, 'right_w1': 1.6570827461132183, 'right_w2': -3.0579907006495723, 'left_e0': -1.716524501643778, 'left_e1': 2.5322187856018465, 'left_s0': 1.5531555477339813, 'left_s1': 0.034514567727421806, 'right_e0': 1.507136124097419, 'right_e1': 2.0563012461603973}
    micro_front_door2 = {'left_w0': -1.472238061173026, 'left_w1': -1.5707963267946636, 'left_w2': 1.3437671701876224, 'right_s0': 0.4356505437594575, 'right_s1': 0.05253884198507542, 'right_w0': 3.0595246814374577, 'right_w1': 1.5251603983550726, 'right_w2': -0.17180584824316633, 'left_e0': -1.7157575112498353, 'left_e1': 2.5322187856018465, 'left_s0': 1.5520050621430674, 'left_s1': 0.03413107253045045, 'right_e0': 1.7307138239317181, 'right_e1': 1.880276950750546}
    
    micro_front_door3 = {'left_w0': -1.4733885467639398, 'left_w1': -1.570029336400721, 'left_w2': 1.3433836749906511, 'right_s0': 0.45405831321408247, 'right_s1': -0.7765777738669907, 'right_w0': 0.5767767762449155, 'right_w1': -1.2540292940963258, 'right_w2': 0.3466796580621035, 'left_e0': -1.7161410064468066, 'left_e1': 2.5322187856018465, 'left_s0': 1.5523885573400387, 'left_s1': 0.033364082136507746, 'right_e0': 1.2505778373235836, 'right_e1': 1.8952332634324287}

    print "-----> move to the initial position"

    both_arms.set_joint_value_target(init_pos)
    both_arms.plan()
    both_arms.go(wait=True)

    print "-----> move inside the fridge position"

    both_arms.set_joint_value_target(fridge_pos)
    both_arms.plan()
    both_arms.go(wait=True)

    rightgripper.close()
    rospy.sleep(2)

    both_arms.set_joint_value_target(init_pos)
    both_arms.plan()
    both_arms.go(wait=True)

    both_arms.set_joint_value_target(micro_front_door3)
    both_arms.plan()
    both_arms.go(wait=True)

    print "-----> move inside the microwave position"

    both_arms.set_joint_value_target(microwave)
    both_arms.plan()
    both_arms.go(wait=True)

    moveit_commander.os._exit(0)

    both_arms.set_joint_value_target(pos1)
    both_arms.plan()
    both_arms.go(wait=True)
    time.sleep(10)

    moveit_commander.os._exit(0)
    

if __name__=='__main__':
    try:
        rospy.init_node('pnp', anonymous=True)
        # old function
        # measure_zero_point()
        
        # test function from front door to the fridge and from from fridge to the microwave
        move_arm()

    except rospy.ROSInterruptException:
        pass
