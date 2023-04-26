# Trajectory Part of the tiago for now - converted to py

import exception   
import string

from boost import shared_ptr

import ros
from actionlib import client.simple_action_client
from control_msgs import FollowJointTrajectoryAction
from ros import topic

#  typedef part - have to convert to rospy
# typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
# typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

def createArmClient(action_client, arm_controller_name):
    
    rospy.loginfo("Creating action client to"+ str(arm_controller_name))

    action_client_name = "/" + str(arm_controller_name) + "/follow_joint_trajectory"

    action_client.reset(arm_control_client(action_client_name))

    iterations = 0
    # max iterations = ?

    # while(!action_client.waitForServer(rospy.Duration(2)) and !(rospy.is_shutdown() ''' and iterations < max_iterations ''')):
        # 
        # ROS_DEBUG("Waiting for arm_controller_action server to come up")
        # iterations = iterations+1
        # ROS_DEBUG("Error: arm controller action server not available")

    if (iterations = max_iterations):
        throw RuntimeError("Error: Arm Controller Action server not available")
    
def waypointsArmGoal(goal):
    goal.trajectory.points.resize(2)
    index = 0

    goal.trajectory.points[index].positions.resize(7)``
    goal.trajectory.points[index].positions[0] = 0.00
    goal.trajectory.points[index].positions[1] = 0.59
    goal.trajectory.points[index].positions[2] = 0.06
    goal.trajectory.points[index].positions[3] = 1.00
    goal.trajectory.points[index].positions[4] = -1.70
    goal.trajectory.points[index].positions[5] = 0.0
    goal.trajectory.points[index].positions[6] = 0.0
    #  Velocities
    goal.trajectory.points[index].velocities.resize(7)

    for j in range(0,7):
        goal.trajectory.points[index].velocities[j] = 1.0
    
    # To be reached 4 second after starting along the trajectory
    
    goal.trajectory.points[index].time_from_start = rospy.Duration(4.0)
    # Second trajectory point
    # Positions
    index = index + 1
    goal.trajectory.points[index].positions.resize(7)
    goal.trajectory.points[index].positions[0] = 1.20
    goal.trajectory.points[index].positions[1] = 0.59
    goal.trajectory.points[index].positions[2] = 0.06
    goal.trajectory.points[index].positions[3] = 1.00
    goal.trajectory.points[index].positions[4] = -1.70
    goal.trajectory.points[index].positions[5] = 0.0
    goal.trajectory.points[index].positions[6] = 0.0
    # Velocities
    goal.trajectory.points[index].velocities.resize(7)
    for j in range(0,7):
        goal.trajectory.points[index].velocities[j] = 0.0
    #   To be reached 8 seconds after starting along the trajectory
    goal.trajectory.points[index].time_from_start = rospy.Duration(8.0)

def waypointsArmLeftGoal(goal):
    goal.trajectory.joint_names.append("arm_left_1_joint")
    goal.trajectory.joint_names.append("arm_left_2_joint")
    goal.trajectory.joint_names.append("arm_left_3_joint")
    goal.trajectory.joint_names.append("arm_left_4_joint")
    goal.trajectory.joint_names.append("arm_left_5_joint")
    goal.trajectory.joint_names.append("arm_left_6_joint")
    goal.trajectory.joint_names.append("arm_left_7_joint")
    
    waypointsArmGoal(goal)

def waypointsArmRightGoal(goal):

    goal.trajectory.joint_names.append("arm_right_1_joint")
    goal.trajectory.joint_names.append("arm_right_2_joint")
    goal.trajectory.joint_names.append("arm_right_3_joint")
    goal.trajectory.joint_names.append("arm_right_4_joint")
    goal.trajectory.joint_names.append("arm_right_5_joint")
    goal.trajectory.joint_names.append("arm_right_6_joint")
    goal.trajectory.joint_names.append("arm_right_7_joint")
    
    waypointsArmGoal(goal)

def __main__(argc, argv):
    rospy.init_node(argc, argv, "run_dual_traj_control")
    rospy.loginfo("Starting Dual Trajectory")

    # ros::NodeHandle nh

    # if (!ros::Time::waitForValid(ros::WallDuration(10.0))) - need to find rospy code
    # {
    #   ROS_FATAL("Timed-out waiting for valid time.");
    #   return EXIT_FAILURE;
    # }

    # arm_control_client_ptr arm_left_client # - rospy code for object/class

    arm_left_client = arm_control_client_ptr()

    createArmClient(arm_left_client, "arm_left_controller")

    # arm_control_client_ptr arm_right_client # - rospy code for object/class

    arm_right_client = arm_control_client_ptr()

    createArmClient(arm_right_client, "arm_right_controller")

    waypointsArmLeftGoal(arm_left_goal)

    arm_left_client.trajectory.header.stamp = rospy.get_time() + rospy.Duration(1)

    arm_left_client.sendGoal(arm_left_goal)

    # rospy.sleep(4)

    waypointsArmrightGoal(arm_right_goal)

    arm_right_client.trajectory.header.stamp = rospy.get_time() + rospy.Duration(1)

    arm_right_client.sendGoal(arm_right_goal)

    # rospy.sleep(4)
