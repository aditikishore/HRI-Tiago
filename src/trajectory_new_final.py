# # Trajectory Part of the tiago for now - converted to py

# import exception   
# import string

# from boost import shared_ptr

# import ros;
# from actionlib import client; '''.simple_action_client'''
# from control_msgs import FollowJointTrajectoryAction
# from ros import topic

# #  typedef part - have to convert to rospy
# # typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
# # typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;
# '''
# class move_arm(self):
#     def __init__(self,goal):

# '''

# def createArmClient(action_client, arm_controller_name):
    
#     rospy.loginfo("Creating action client to"+ str(arm_controller_name))

#     action_client_name = "/" + str(arm_controller_name) + "/follow_joint_trajectory"

#     action_client.reset(arm_control_client(action_client_name))

#     iterations = 0
#     # max iterations = ?

#     # while(!action_client.waitForServer(rospy.Duration(2)) and !(rospy.is_shutdown() ''' and iterations < max_iterations ''')):
#         # 
#         # ROS_DEBUG("Waiting for arm_controller_action server to come up")
#         # iterations = iterations+1
#         # ROS_DEBUG("Error: arm controller action server not available")

#     # if (iterations = max_iterations):
#     #     throw RuntimeError("Error: Arm Controller Action server not available")
    
# def waypointsArmGoal(goal):
#     goal.trajectory.points.resize(2)
#     index = 0

#     goal.trajectory.points[index].positions.resize(7)
#     goal.trajectory.points[index].positions[0] = 0.00
#     goal.trajectory.points[index].positions[1] = 0.59
#     goal.trajectory.points[index].positions[2] = 0.06
#     goal.trajectory.points[index].positions[3] = 1.00
#     goal.trajectory.points[index].positions[4] = -1.70
#     goal.trajectory.points[index].positions[5] = 0.0
#     goal.trajectory.points[index].positions[6] = 0.0
#     #  Velocities
#     goal.trajectory.points[index].velocities.resize(7)

#     for j in range(0,7):
#         goal.trajectory.points[index].velocities[j] = 1.0
    
#     # To be reached 4 second after starting along the trajectory
    
#     goal.trajectory.points[index].time_from_start = rospy.Duration(4.0)
#     # Second trajectory point
#     # Positions
#     index = index + 1
#     goal.trajectory.points[index].positions.resize(7)
#     goal.trajectory.points[index].positions[0] = 1.20
#     goal.trajectory.points[index].positions[1] = 0.59
#     goal.trajectory.points[index].positions[2] = 0.06
#     goal.trajectory.points[index].positions[3] = 1.00
#     goal.trajectory.points[index].positions[4] = -1.70
#     goal.trajectory.points[index].positions[5] = 0.0
#     goal.trajectory.points[index].positions[6] = 0.0
#     # Velocities
#     goal.trajectory.points[index].velocities.resize(7)
#     for j in range(0,7):
#         goal.trajectory.points[index].velocities[j] = 0.0
#     #   To be reached 8 seconds after starting along the trajectory
#     goal.trajectory.points[index].time_from_start = rospy.Duration(8.0)

# def waypointsArmLeftGoal(goal):
#     goal.trajectory.joint_names.append("arm_left_1_joint")
#     goal.trajectory.joint_names.append("arm_left_2_joint")
#     goal.trajectory.joint_names.append("arm_left_3_joint")
#     goal.trajectory.joint_names.append("arm_left_4_joint")
#     goal.trajectory.joint_names.append("arm_left_5_joint")
#     goal.trajectory.joint_names.append("arm_left_6_joint")
#     goal.trajectory.joint_names.append("arm_left_7_joint")
    
#     waypointsArmGoal(goal)

# def waypointsArmRightGoal(goal):

#     goal.trajectory.joint_names.append("arm_right_1_joint")
#     goal.trajectory.joint_names.append("arm_right_2_joint")
#     goal.trajectory.joint_names.append("arm_right_3_joint")
#     goal.trajectory.joint_names.append("arm_right_4_joint")
#     goal.trajectory.joint_names.append("arm_right_5_joint")
#     goal.trajectory.joint_names.append("arm_right_6_joint")
#     goal.trajectory.joint_names.append("arm_right_7_joint")
    
#     waypointsArmGoal(goal)

# def main():
#     rospy.init_node(argc, argv, "run_dual_traj_control")
#     rospy.loginfo("Starting Dual Trajectory")

#     # ros::NodeHandle nh

#     # if (!ros::Time::waitForValid(ros::WallDuration(10.0))) - need to find rospy code
#     # {
#     #   ROS_FATAL("Timed-out waiting for valid time.");
#     #   return EXIT_FAILURE;
#     # }

#     # arm_control_client_ptr arm_left_client # - rospy code for object/class

#     arm_left_client = arm_control_client_ptr()

#     createArmClient(arm_left_client, "arm_left_controller")

#     # arm_control_client_ptr arm_right_client # - rospy code for object/class

#     arm_right_client = arm_control_client_ptr()

#     createArmClient(arm_right_client, "arm_right_controller")

#     waypointsArmLeftGoal(arm_left_goal)

#     arm_left_client.trajectory.header.stamp = rospy.get_time() + rospy.Duration(1)

#     arm_left_client.sendGoal(arm_left_goal)

#     # rospy.sleep(4)

#     waypointsArmrightGoal(arm_right_goal)

#     arm_right_client.trajectory.header.stamp = rospy.get_time() + rospy.Duration(1)

#     arm_right_client.sendGoal(arm_right_goal)

#     # rospy.sleep(4)


# if __name__ == "__main__":
#     main()


#  New code:

#!/usr/bin/env/python

# import rospy
# import time

# import actionlib
# from geometry_msgs.msg import PointStamped
# from control_msgs.msg import PointHeadAction, PointHeadGoal
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# class robot_arm():
#     def __init__(self):

# Test Code:

#!/usr/bin/env python

# Import ROS libraries and messages
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# Our Action interface type for moving TIAGo's head
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from rospy import Duration


# Create a ROS action client to move TIAGo's arm
def createArmClient(arm_controller_name):
    rospy.loginfo("Creating action client to %s ...", arm_controller_name)

    action_client_name = "/" + arm_controller_name + "/follow_joint_trajectory"
    action_client = SimpleActionClient(action_client_name, FollowJointTrajectoryAction)

    iterations = 0
    max_iterations = 3
    # Wait for arm controller action server to come up
    while (not action_client.wait_for_server(Duration(2.0))) and rospy.ok() and iterations < max_iterations:
        rospy.logdebug("Waiting for the arm_controller_action server to come up")
        iterations += 1

    if iterations == max_iterations:
        raise RuntimeError("Error in createArmClient: arm controller action server not available")

    return action_client


# Generates a simple trajectory with two waypoints to move TIAGo's arm
def waypointsArmGoal():
    # Create a FollowJointTrajectoryGoal object
    goal = FollowJointTrajectoryGoal()

    # Two waypoints in this goal trajectory
    goal.trajectory.points.resize(2)

    # First trajectory point
    # Positions
    index = 0
    goal.trajectory.points[index].positions.resize(7)
    goal.trajectory.points[index].positions[0] = 0.00
    goal.trajectory.points[index].positions[1] = 0.59
    goal.trajectory.points[index].positions[2] = 0.06
    goal.trajectory.points[index].positions[3] = 1.00
    goal.trajectory.points[index].positions[4] = -1.70
    goal.trajectory.points[index].positions[5] = 0.0
    goal.trajectory.points[index].positions[6] = 0.0
    # Velocities
    goal.trajectory.points[index].velocities.resize(7)
    for j in range(7):
        goal.trajectory.points[index].velocities[j] = 1.0
    # To be reached 4 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = Duration(4.0)

    # Second trajectory point
    # Positions
    index += 1
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
    for j in range(7):
        goal.trajectory.points[index].velocities[j] = 0.0
    # To be reached 8 seconds after starting along the trajectory
    goal.trajectory.points[index].time_from_start = Duration(8.0)

    return goal

# The joint names for right arm
ARM_JOINT_NAMES_LEFT = ["arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"]

# Generates a simple trajectory with two waypoints to move the left arm
def waypointsArmLeftGoal():
    # Create the goal object
    goal = FollowJointTrajectoryGoal()

    # Set the joint names for the trajectory
    goal.trajectory.joint_names = ARM_JOINT_NAMES_LEFT

    # Set the trajectory waypoints
    waypointsArmGoal(goal)

    return goal

# The joint names for right arm
ARM_JOINT_NAMES_RIGHT = ["arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"]

# Generates a simple trajectory with two waypoints to move the right arm
def waypointsArmRightGoal():
    # Create the goal object
    goal = FollowJointTrajectoryGoal()

    # Set the joint names for the trajectory
    goal.trajectory.joint_names = ARM_JOINT_NAMES_RIGHT

    # Set the trajectory waypoints
    waypointsArmGoal(goal)

    return goal

# Helper function to create an arm controller action client
def createArmClient(controller_name):
    client = actionlib.SimpleActionClient(controller_name + '/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    rospy.loginfo('Connected to %s', controller_name)
    return client

# # Generates the trajectory waypoints for the arms
# def waypointsArmGoal(goal):
#     # Set the trajectory waypoints
#     num_waypoints = 2
#     num_joints = len(ARM_JOINT_NAMES)
#     times = [0.0, 3.0]
#     positions = [[0.0] * num_joints] * num_waypoints
#     positions[1] = [-1.2, -1.2, 1.57, 1.57, 1.57, 1.57, 1.57]
#     goal.trajectory.points = []
#     for i in range(num_waypoints):
#         point = trajectory_msgs.msg.JointTrajectoryPoint()
#         point.positions = positions[i]
#         point.time_from_start = rospy.Duration.from_sec(times[i])
#         goal.trajectory.points.append(point)

# Entry point
if __name__ == '__main__':
    # Init the ROS node
    rospy.init_node('run_dual_traj_control')

    rospy.loginfo("Starting run_dual_traj_control application ...")

    # Precondition: Valid clock
    if not rospy.Time.now():
        rospy.logfatal("Timed-out waiting for valid time.")
        exit(1)

    # Create an arm left controller action client to move the left arm
    arm_left_client = createArmClient("arm_left_controller")

    # Create an arm right controller action client to move the right arm
    arm_right_client = createArmClient("arm_right_controller")

    # Generates the goal for the left arm
    arm_left_goal = waypointsArmLeftGoal()

    # Sends the command to start the given trajectory 1s from now
    arm_left_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(1.0)
    arm_left_client.send_goal(arm_left_goal)

    # Wait for trajectory execution
    while not rospy.is_shutdown() and not arm_left_client.wait_for_result(rospy.Duration.from_sec(4.0)):
        rospy.loginfo('Executing left arm trajectory...')

    # Generates the goal for the right arm
    arm_right_goal = waypointsArmRightGoal()

    # Sends the command to start the given trajectory 1s from now
    arm_right_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(1.0)
    arm_right_client.send_goal(arm_right_goal)

    # Wait for trajectory execution
    while not rospy.is_shutdown() and not arm_right_client.wait_for_result(rospy.Duration.from_sec(4.0)):
        rospy.loginfo('Executing right arm trajectory...')
